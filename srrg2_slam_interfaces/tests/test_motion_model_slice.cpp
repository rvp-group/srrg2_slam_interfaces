#include <srrg_test/test_helper.hpp>

#include "srrg_geometry/geometry2d.h"
#include "srrg_geometry/geometry3d.h"
#include "srrg_slam_interfaces/instances.h"

using namespace srrg2_slam_interfaces;
using namespace srrg2_core;

// ds NOTE: below tests expect the SLAM processing sequence as of 2019/10/18
// ds if the sequence is modified (e.g. different order of set/populate/merge),
// ds these tests might fail and the motion model slice has to be adjusted!

// ds modified tracker for testing only
class MockedMultiTracker3D : public MultiTracker3D {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BaseType = MultiTracker3D;

  // ds set new empty scene and change into tracking state
  void setScene(SceneType* scene_) {
    BaseType::setScene(scene_);
    _status = TrackerBase::Tracking;
  }

  // ds adaption always leads to tracking state (with prior slice only)
  void adaptMeasurements() override {
    BaseType::adaptMeasurements();
    _status = TrackerBase::Tracking;
  }

  // ds align does not overwrite the tracker estimate based on motion model
  void align();
};
using MockedMultiTracker3DPtr = std::shared_ptr<MockedMultiTracker3D>;

// ds readability
void setup(MockedMultiTracker3DPtr& tracker_, MultiAligner3DQRPtr& aligner_);

int main(int argc_, char** argv_) {
  return srrg2_test::runTests(argc_, argv_);
}

TEST(MultiAlignerSliceMotionModel3D, Random) {
  // ds allocate tracker and aligner modules
  MockedMultiTracker3DPtr tracker;
  MultiAligner3DQRPtr aligner;
  setup(tracker, aligner);
  // ds local map container, starting pose and dummy measurement
  PropertyContainerDynamic local_map;
  Isometry3f previous_estimate(Isometry3f::Identity());
  BaseSensorMessagePtr message(new BaseSensorMessage());

  // ds process initial measurement
  tracker->setScene(&local_map);
  tracker->populateScene(local_map);
  tracker->setEstimate(previous_estimate);
  tracker->setMeasurement(message);
  tracker->compute();

  // ds simulate motion
  Isometry3f motion_previous(Isometry3f::Identity());
  for (size_t i = 0; i < 10; ++i) {
    srand(i);
    Isometry3f motion(Isometry3f::Identity());
    motion.translation() += Vector3f::Random();
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));

    // ds update tracker for new pose
    const Isometry3f current_estimate(previous_estimate * motion);
    tracker->setEstimate(current_estimate);
    tracker->setMeasurement(message);
    tracker->compute();
    
    // ds the motion model should correspond to the previous true motion
    ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);

    // ds recall that the aligner operates in flipped transform (i.e. moving -> fixed)
    const Vector6f error =
      geometry3d::t2v(motion_previous.inverse()) - geometry3d::t2v(aligner->estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), 1e-5 /*float inverses*/);
    previous_estimate = current_estimate;
    motion_previous   = motion;
  }
}

TEST(MultiAlignerSliceMotionModel3D, LocalMapCreation) {
  // ds allocate tracker and aligner modules
  MockedMultiTracker3DPtr tracker;
  MultiAligner3DQRPtr aligner;
  setup(tracker, aligner);

  // ds local map container, starting pose and dummy measurement
  PropertyContainerDynamic local_map;
  Isometry3f previous_estimate(Isometry3f::Identity());
  BaseSensorMessagePtr message(new BaseSensorMessage());

  // ds process initial measurement
  tracker->populateScene(local_map);
  tracker->setScene(&local_map);
  tracker->setEstimate(previous_estimate);
  tracker->setMeasurement(message);
  tracker->compute();

  // ds simulate motion
  Isometry3f motion_previous(Isometry3f::Identity());
  for (size_t i = 0; i < 100; ++i) {
    srand(i);
    Isometry3f motion(Isometry3f::Identity());
    motion.translation() += Vector3f::Random();
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    Isometry3f current_estimate(previous_estimate * motion);

    // ds advance tracker in current sequence (as in SLAM)
    tracker->setMeasurement(message);
    tracker->adaptMeasurements();
    tracker->align();

    // ds simulate multiple consecutive new local map creations
    if (i % 10 == 0) {
      current_estimate = motion;
      PropertyContainerDynamic new_local_map;
      tracker->populateScene(new_local_map);
      tracker->setScene(&new_local_map);
    }
    tracker->setEstimate(current_estimate);
    tracker->merge();

    // ds the motion model should correspond to the previous true motion
    ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);

    // ds recall that the aligner operates in flipped transform (i.e. moving -> fixed)
    const Vector6f error =
      geometry3d::t2v(motion_previous.inverse()) - geometry3d::t2v(aligner->estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), 1e-5 /*float inverses*/);
    previous_estimate = current_estimate;
    motion_previous   = motion;
  }
}

TEST(MultiAlignerSliceMotionModel3D, Relocalization) {
  // ds allocate tracker and aligner modules
  MockedMultiTracker3DPtr tracker;
  MultiAligner3DQRPtr aligner;
  setup(tracker, aligner);

  // ds local map container, starting pose and dummy measurement
  PropertyContainerDynamic local_map_a;
  Isometry3f estimate_origin_a(Isometry3f::Identity());
  BaseSensorMessagePtr message(new BaseSensorMessage());

  // ds process initial measurement
  tracker->populateScene(local_map_a);
  tracker->setScene(&local_map_a);
  tracker->setEstimate(estimate_origin_a);
  tracker->setMeasurement(message);
  tracker->compute();

  // ds simulate motion
  Isometry3f previous_estimate(Isometry3f::Identity());
  Isometry3f motion_previous(Isometry3f::Identity());
  for (size_t i = 0; i < 100; ++i) {
    srand(i);
    Isometry3f motion(Isometry3f::Identity());
    motion.translation() += Vector3f::Random();
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
    motion.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
    Isometry3f current_estimate(previous_estimate * motion);
    const Isometry3f motion_transform(current_estimate * previous_estimate.inverse());

    // ds advance tracker in current sequence (as in SLAM)
    tracker->setMeasurement(message);
    tracker->adaptMeasurements();
    tracker->setEstimate(current_estimate);
    tracker->align();

    // ds simulate multiple relocalization - move into another local map (b)
    if (i % 10 == 0) {
      // ds new local map to relocalize in
      PropertyContainerDynamic local_map_b;
      Isometry3f estimate_origin_b(Isometry3f::Identity());
      estimate_origin_b.translation() += Vector3f::Random();
      estimate_origin_b.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitX()));
      estimate_origin_b.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitY()));
      estimate_origin_b.rotate(AngleAxisf(rand() * M_PI, Vector3f::UnitZ()));
      const Isometry3f local_map_a_in_b = estimate_origin_b.inverse() * estimate_origin_a;

      // ds estimate moves into local map B's coordinate system
      const Isometry3f current_estimate_in_a(current_estimate);
      current_estimate = local_map_a_in_b * current_estimate_in_a;

      // ds compute the relocalization motion required for above transition from previous
      const Isometry3f relocalization_transform = local_map_a_in_b * motion_transform;

      // ds verify relocalization mockup
      const Isometry3f previous_moved_in_b_1 = relocalization_transform * previous_estimate;
      const Isometry3f previous_moved_in_b_2 = (local_map_a_in_b * previous_estimate) * motion;
      const Isometry3f local_map_a_in_b_rev  = current_estimate * current_estimate_in_a.inverse();
      ASSERT_NEAR_EIGEN(previous_moved_in_b_1, current_estimate, 1e-4);
      ASSERT_NEAR_EIGEN(previous_moved_in_b_2, current_estimate, 1e-4);
      ASSERT_NEAR_EIGEN(local_map_a_in_b_rev, local_map_a_in_b, 1e-4);
      tracker->setClosure(CorrespondenceVector(), relocalization_transform, current_estimate);
      tracker->setScene(&local_map_b);
    }
    tracker->setEstimate(current_estimate);
    tracker->merge();

    // ds the motion model should correspond to the previous true motion
    ASSERT_EQ(tracker->status(), MultiTracker3D::Status::Tracking);

    // ds recall that the aligner operates in flipped transform (i.e. moving -> fixed)
    const Vector6f error =
      geometry3d::t2v(motion_previous.inverse()) - geometry3d::t2v(aligner->estimate());

    // ds no noise, no error
    ASSERT_LT(error.norm(), 1e-4 /*float inverses*/);
    previous_estimate = current_estimate;
    motion_previous   = motion;
  }
}

// ---------------------------------------------------------------------------------
// ds test helper methods:
// ds align does not overwrite the tracker estimate based on motion model
void MockedMultiTracker3D::align() {
  // ds configure aligner
  AlignerPtrType aligner = param_aligner.value();
  aligner->setFixed(&_adapted_measurements);
  aligner->setMoving(&_clipped_scene);

  // ds compute relative transform between all fixed and moving slices
  aligner->compute();

  // ds store correspondences for merging
  aligner->storeCorrespondences();

  // ds evaluate tracker status based on aligner performance
  if (aligner->status() == AlignerBase::Success) {
    _status = TrackerBase::Tracking;

    // ds here the tracker estimate would be updated based on the alignment result

    // ds update tracker estimate with current one (changed externally)
    setEstimate(_tracker_estimate);
  } else {
    _status = TrackerBase::Lost;
  }
}

// ds tracker and aligner setup
void setup(MockedMultiTracker3DPtr& tracker_, MultiAligner3DQRPtr& aligner_) {
  slam_interfaces_registerTypes();

  // ds tracker configuration
  tracker_ = MockedMultiTracker3DPtr(new MockedMultiTracker3D());
  MultiTrackerSliceEstimationBuffer3DPtr tracker_slice(new MultiTrackerSliceEstimationBuffer3D());
  tracker_slice->param_measurement_slice_name.setValue("tracker_pose");
  tracker_slice->param_adaptor.setValue(
    MeasurementAdaptorTrackerEstimate3DPtr(new MeasurementAdaptorTrackerEstimate3D()));
  tracker_->param_slice_processors.pushBack(tracker_slice);

  // ds aligner configuration
  aligner_ = MultiAligner3DQRPtr(new MultiAligner3DQR());
  tracker_->param_aligner.setValue(aligner_);
  MultiAlignerSliceMotionModel3DPtr aligner_slice(new MultiAlignerSliceMotionModel3D());
  aligner_slice->param_motion_model.setValue(
    MotionModelConstantVelocity3DPtr(new MotionModelConstantVelocity3D()));
  aligner_slice->param_fixed_slice_name.setValue("tracker_pose");
  aligner_->param_slice_processors.pushBack(aligner_slice);

  // ds we're not performing an actual registration
  aligner_->param_min_num_inliers.setValue(0);
}
