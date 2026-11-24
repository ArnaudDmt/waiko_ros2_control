#include <gmock/gmock.h>

#include <vector>

#include <state-observation/tools/definitions.hpp>

#include "waiko_ros2_control/waiko_estimator.hpp"

namespace waiko_ros2_control
{
namespace
{

constexpr double kDt = 0.005;
constexpr double kTolerance = 1e-9;

/// Norm of the difference, which unlike isApprox() is meaningful for vectors close to zero.
double distance(const Eigen::Vector3d & a, const Eigen::Vector3d & b) { return (a - b).norm(); }

/// Robot standing still: identity orientation, IMU 1 m above the world origin, contacts on the
/// ground below it.
class StandingRobot : public ::testing::Test
{
protected:
  const Eigen::Vector3d imu_position_{0.0, 0.0, 1.0};
  // Specific force measured by an accelerometer at rest.
  const Eigen::Vector3d accelerometer_{0.0, 0.0, stateObservation::cst::gravityConstant};
  const Eigen::Vector3d gyrometer_ = Eigen::Vector3d::Zero();

  /// Measurement of a contact at world_position, which is fixed in the world.
  ContactMeasurement contact(const std::string & name, const Eigen::Vector3d & world_position, bool active)
  {
    return {name, active, 1.0, world_position - imu_position_, Eigen::Quaterniond::Identity(),
            Eigen::Vector3d::Zero()};
  }

  WaikoEstimator make_estimator(bool reset_position_anchor)
  {
    waiko_estimator::Params params;
    params.reset_position_anchor_on_contact_change = reset_position_anchor;
    return WaikoEstimator(
      params, Eigen::Vector3d::Zero(),
      Eigen::Vector3d::UnitZ(), Eigen::Quaterniond::Identity(), imu_position_);
  }

  const Eigen::Vector3d left_{0.1, 0.1, 0.0};
  const Eigen::Vector3d right_{0.1, -0.1, 0.0};
  const Eigen::Vector3d hand_{0.5, 0.0, 0.8};
};

TEST_F(StandingRobot, StateStaysAtRest)
{
  WaikoEstimator estimator = make_estimator(false);
  const std::vector<ContactMeasurement> contacts = {
    contact("left", left_, true), contact("right", right_, true)};

  for (int i = 0; i < 400; ++i)
  {
    estimator.update(kDt, accelerometer_, gyrometer_, contacts);
  }

  const WaikoEstimate & estimate = estimator.estimate();
  EXPECT_LT(distance(estimate.position, imu_position_), kTolerance);
  EXPECT_LT(estimate.orientation.angularDistance(Eigen::Quaterniond::Identity()), kTolerance);
  EXPECT_LT(estimate.local_linear_velocity.norm(), kTolerance);
  EXPECT_EQ(estimate.maintained_contacts_count, 2u);
  EXPECT_EQ(estimate.contacts.size(), 2u);
}

TEST_F(StandingRobot, AnchorMovesToTheContactsWhenTheyChange)
{
  WaikoEstimator estimator = make_estimator(true);
  EXPECT_TRUE(estimator.estimate().position_anchor.isZero());

  // The first contacts move the anchor to their centroid.
  std::vector<ContactMeasurement> contacts = {
    contact("left", left_, true), contact("right", right_, true), contact("hand", hand_, false)};
  estimator.update(kDt, accelerometer_, gyrometer_, contacts);
  EXPECT_LT(distance(estimator.estimate().position_anchor, (left_ + right_) / 2.0), kTolerance);

  // Unchanged contacts keep the anchor.
  estimator.update(kDt, accelerometer_, gyrometer_, contacts);
  EXPECT_LT(distance(estimator.estimate().position_anchor, (left_ + right_) / 2.0), kTolerance);

  // A new contact moves it to the centroid of the three contacts.
  contacts[2].active = true;
  estimator.update(kDt, accelerometer_, gyrometer_, contacts);
  EXPECT_LT(
    distance(estimator.estimate().position_anchor, (left_ + right_ + hand_) / 3.0), kTolerance);

  // Without contact, the anchor is kept.
  for (ContactMeasurement & c : contacts)
  {
    c.active = false;
  }
  estimator.update(kDt, accelerometer_, gyrometer_, contacts);
  EXPECT_LT(
    distance(estimator.estimate().position_anchor, (left_ + right_ + hand_) / 3.0), kTolerance);
}

TEST_F(StandingRobot, AnchorResetKeepsTheWorldState)
{
  WaikoEstimator with_reset = make_estimator(true);
  WaikoEstimator without_reset = make_estimator(false);

  std::vector<ContactMeasurement> contacts = {
    contact("left", left_, true), contact("right", right_, false), contact("hand", hand_, false)};
  for (int i = 0; i < 300; ++i)
  {
    // The contacts change every 50 steps.
    contacts[1].active = (i / 50) % 2 == 1;
    contacts[2].active = i >= 150;
    with_reset.update(kDt, accelerometer_, gyrometer_, contacts);
    without_reset.update(kDt, accelerometer_, gyrometer_, contacts);

    const WaikoEstimate & a = with_reset.estimate();
    const WaikoEstimate & b = without_reset.estimate();
    ASSERT_LT(distance(a.position, b.position), kTolerance) << "step " << i;
    ASSERT_LT(a.orientation.angularDistance(b.orientation), kTolerance) << "step " << i;
    ASSERT_LT(distance(a.local_linear_velocity, b.local_linear_velocity), kTolerance)
      << "step " << i;
  }
  EXPECT_FALSE(with_reset.estimate().position_anchor.isZero());
  EXPECT_TRUE(without_reset.estimate().position_anchor.isZero());
}

}  // namespace
}  // namespace waiko_ros2_control
