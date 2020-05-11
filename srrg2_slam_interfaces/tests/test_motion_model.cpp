#include <srrg_test/test_helper.hpp>

#include "srrg2_slam_interfaces/motion_models/motion_model_constant_velocity.hpp"
#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"

using namespace srrg2_slam_interfaces;
using namespace srrg2_core;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(MotionModelConstantVelocity3D, Still) {
  MotionModelConstantVelocity3D model;

  // ds simulate motion
  Isometry3f previous_estimate(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    const Isometry3f current_estimate(Isometry3f::Identity());
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds motion model should not move
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity3D, UniformTranslation) {
  MotionModelConstantVelocity3D model;

  // ds simulate motion
  Isometry3f previous_estimate(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    Isometry3f current_estimate(previous_estimate);
    current_estimate.translation() += Vector3f::Ones();
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds motion model should move without error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity3D, UniformRotation) {
  MotionModelConstantVelocity3D model;

  // ds simulate motion
  Isometry3f previous_estimate(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    Isometry3f current_estimate(previous_estimate);
    current_estimate.rotate(AngleAxisf(0.1 * M_PI, Vector3f::UnitX()));
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity3D, UniformTranslationAndRotation) {
  MotionModelConstantVelocity3D model;

  // ds simulate motion
  Isometry3f previous_estimate(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    Isometry3f current_estimate(previous_estimate);
    current_estimate.translation() += Vector3f::Ones();
    current_estimate.rotate(AngleAxisf(0.1 * M_PI, Vector3f::UnitX()));
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity3D, Random) {
  MotionModelConstantVelocity3D model;

  // ds simulate motion
  Isometry3f previous_estimate(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    srand(i);
    Isometry3f current_estimate(previous_estimate);
    current_estimate.translation() += Vector3f::Random();
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity3D, NewLocalMap) {
  MotionModelConstantVelocity3D model;

  // ds simulate motion in local map
  Isometry3f previous_estimate(Isometry3f::Identity());
  for (size_t i = 0; i < 5; ++i) {
    srand(i);
    Isometry3f current_estimate(previous_estimate);
    current_estimate.translation() += Vector3f::Random();
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }

  // ds generate a local map (i.e. reset estimate)
  previous_estimate.setIdentity();
  model.shiftTrackerEstimate(previous_estimate);

  // ds simulate motion in new local map
  for (size_t i = 5; i < 10; ++i) {
    srand(i);
    Isometry3f current_estimate(previous_estimate);
    current_estimate.translation() += Vector3f::Random();
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity3D, RelocalizationInLocalMap) {
  MotionModelConstantVelocity3D model;

  // ds simulate motion in local map
  Isometry3f previous_estimate(Isometry3f::Identity());
  for (size_t i = 0; i < 5; ++i) {
    srand(i);
    Isometry3f current_estimate(previous_estimate);
    current_estimate.translation() += Vector3f::Random();
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }

  // ds relocalize into local map
  previous_estimate.setIdentity();
  previous_estimate.translation() += Vector3f::Random();
  previous_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
  previous_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
  previous_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
  model.shiftTrackerEstimate(previous_estimate);

  // ds simulate motion in new local map
  for (size_t i = 5; i < 10; ++i) {
    srand(i);
    Isometry3f current_estimate(previous_estimate);
    current_estimate.translation() += Vector3f::Random();
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    current_estimate.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry3f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector6f error         = geometry3d::t2v(true_motion) - geometry3d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity2D, NewLocalMap) {
  MotionModelConstantVelocity2D model;

  // ds simulate motion in local map
  Isometry2f previous_estimate(Isometry2f::Identity());
  for (size_t i = 0; i < 5; ++i) {
    srand(i);
    Isometry2f current_estimate(previous_estimate);
    current_estimate.translation() += Vector2f::Random();
    current_estimate.rotate(rand() * M_PI);
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry2f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector3f error         = geometry2d::t2v(true_motion) - geometry2d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }

  // ds generate a local map (i.e. reset estimate)
  previous_estimate.setIdentity();
  model.shiftTrackerEstimate(previous_estimate);

  // ds simulate motion in new local map
  for (size_t i = 5; i < 10; ++i) {
    srand(i);
    Isometry2f current_estimate(previous_estimate);
    current_estimate.translation() += Vector2f::Random();
    current_estimate.rotate(rand() * M_PI);
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry2f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector3f error         = geometry2d::t2v(true_motion) - geometry2d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}

TEST(MotionModelConstantVelocity2D, RelocalizationInLocalMap) {
  MotionModelConstantVelocity2D model;

  // ds simulate motion in local map
  Isometry2f previous_estimate(Isometry2f::Identity());
  for (size_t i = 0; i < 5; ++i) {
    srand(i);
    Isometry2f current_estimate(previous_estimate);
    current_estimate.translation() += Vector2f::Random();
    current_estimate.rotate(rand() * M_PI);
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry2f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector3f error         = geometry2d::t2v(true_motion) - geometry2d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }

  // ds relocalize into local map
  previous_estimate.setIdentity();
  previous_estimate.translation() += Vector2f::Random();
  previous_estimate.rotate(rand() * M_PI);
  model.shiftTrackerEstimate(previous_estimate);

  // ds simulate motion in new local map
  for (size_t i = 5; i < 10; ++i) {
    srand(i);
    Isometry2f current_estimate(previous_estimate);
    current_estimate.translation() += Vector2f::Random();
    current_estimate.rotate(rand() * M_PI);
    model.setRobotInLocalMap(current_estimate);
    model.compute();
    const Isometry2f true_motion = previous_estimate.inverse() * current_estimate;
    const Vector3f error         = geometry2d::t2v(true_motion) - geometry2d::t2v(model.estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), std::numeric_limits<float>::min());

    // ds propagate
    previous_estimate = current_estimate;
  }
}
