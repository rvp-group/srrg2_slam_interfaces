#include <srrg_test/test_helper.hpp>

#include "srrg_slam_interfaces/instances.h"

using namespace srrg2_slam_interfaces;
using namespace srrg2_core;

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(LocalMapSplittingCriterionDistance2D, Translation1D) {
  LocalMapSplittingCriterionDistance2D criterion;
  criterion.param_local_map_distance.setValue(5.0f /*meters*/);

  // ds populate a slam system
  MultiGraphSLAM2D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry2f robot_pose(Isometry2f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    if (i <= 5) {
      ASSERT_FALSE(criterion.hasToSplit());
    } else {
      ASSERT_TRUE(criterion.hasToSplit());
    }

    // ds advance robot
    robot_pose.translation() += Vector2f(1, 0);
  }
}

TEST(LocalMapSplittingCriterionDistance3D, Translation1D) {
  LocalMapSplittingCriterionDistance3D criterion;
  criterion.param_local_map_distance.setValue(5.0f /*meters*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    if (i <= 5) {
      ASSERT_FALSE(criterion.hasToSplit());
    } else {
      ASSERT_TRUE(criterion.hasToSplit());
    }

    // ds advance robot
    robot_pose.translation() += Vector3f(1, 0, 0);
  }
}

TEST(LocalMapSplittingCriterionRotation2D, RotationNegative) {
  LocalMapSplittingCriterionRotation2D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM2D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry2f robot_pose(Isometry2f::Identity());
  const float angle_step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(angle_step_size);
  }
  robot_pose.rotate(angle_step_size);
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation2D, RotationPositive) {
  LocalMapSplittingCriterionRotation2D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM2D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry2f robot_pose(Isometry2f::Identity());
  const float angle_step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(angle_step_size);
  }
  robot_pose.rotate(angle_step_size);
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationPositiveX) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  }
  robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationPositiveY) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitY()));
  }
  robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitY()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationPositiveZ) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitZ()));
  }
  robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitZ()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationPositiveXYNegativeZ) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 6; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()) *
                      AngleAxisf(angle_step_size, Vector3f::UnitY()) *
                      AngleAxisf(-angle_step_size, Vector3f::UnitZ()));
  }
  robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()) *
                    AngleAxisf(angle_step_size, Vector3f::UnitY()) *
                    AngleAxisf(-angle_step_size, Vector3f::UnitZ()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationNegativeX) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = -criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  }
  robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationNegativeY) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = -criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  }
  robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationNegativeZ) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = -criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  }
  robot_pose.rotate(AngleAxisf(angle_step_size, Vector3f::UnitX()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionRotation3D, RotationNegativeXYPositiveZ) {
  LocalMapSplittingCriterionRotation3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float angle_step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 6; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(-angle_step_size, Vector3f::UnitX()) *
                      AngleAxisf(-angle_step_size, Vector3f::UnitY()) *
                      AngleAxisf(angle_step_size, Vector3f::UnitZ()));
  }
  robot_pose.rotate(AngleAxisf(-angle_step_size, Vector3f::UnitX()) *
                    AngleAxisf(-angle_step_size, Vector3f::UnitY()) *
                    AngleAxisf(angle_step_size, Vector3f::UnitZ()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionViewpoint3D, RotationNegativeXYPositiveZ) {
  LocalMapSplittingCriterionViewpoint3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);
  criterion.param_local_map_distance.setValue(1 /*1 meter*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float step_size = criterion.param_local_map_angle_distance_radians.value() / 10;
  for (size_t i = 0; i < 6; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.rotate(AngleAxisf(-step_size, Vector3f::UnitX()) *
                      AngleAxisf(-step_size, Vector3f::UnitY()) *
                      AngleAxisf(step_size, Vector3f::UnitZ()));
  }
  robot_pose.rotate(AngleAxisf(-step_size, Vector3f::UnitX()) *
                    AngleAxisf(-step_size, Vector3f::UnitY()) *
                    AngleAxisf(step_size, Vector3f::UnitZ()));
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}

TEST(LocalMapSplittingCriterionViewpoint3D, TranslationPositiveX) {
  LocalMapSplittingCriterionViewpoint3D criterion;
  criterion.param_local_map_angle_distance_radians.setValue(M_PI / 6 /*30 degrees*/);
  criterion.param_local_map_distance.setValue(1 /*1 meter*/);

  // ds populate a slam system
  MultiGraphSLAM3D slam_system;

  // ds hook it up to the criterion
  criterion.setSlamAlgorithm(&slam_system);

  // ds check criterion continuously for a (simulated) moving robot
  Isometry3f robot_pose(Isometry3f::Identity());
  const float step_size = criterion.param_local_map_distance.value() / 10;
  for (size_t i = 0; i < 10; ++i) {
    slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
    criterion.compute();
    ASSERT_FALSE(criterion.hasToSplit());
    robot_pose.translation() += Vector3f(step_size, 0, 0);
  }
  robot_pose.translation() += Vector3f(step_size, 0, 0);
  slam_system.setRobotPoseInCurrentLocalMap(robot_pose);
  criterion.compute();
  ASSERT_TRUE(criterion.hasToSplit());
}
