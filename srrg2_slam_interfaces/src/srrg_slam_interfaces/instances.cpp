#include "instances.h"

#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>

#include "aligner_termination_criteria_standard_impl.cpp"
#include "multi_aligner_impl.cpp"
#include "multi_aligner_slice_prior_impl.cpp"
#include "multi_relocalizer_impl.cpp"

#include "local_map_selectors/local_map_selector_breadth_first_impl.cpp"
#include "local_map_selectors/local_map_selector_user_defined_impl.cpp"

#include "multi_tracker_impl.cpp"
#include "multi_tracker_slice_prior_impl.cpp"

#include "multi_graph_slam_impl.cpp"
#include "multi_loop_detector_brute_force_impl.cpp"
#include "multi_loop_detector_hbst_impl.cpp"

// ds reduce digital footprint and enforce linker sanity
#define BOSS_REGISTER_CLASS_LINKER_FRIENDLY(CLASS_) \
  CLASS_ dummy_##CLASS_;                            \
  BOSS_REGISTER_CLASS(CLASS_)

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  void MultiAligner2DSliceOdomPrior::setupFactor() {
    Isometry2f delta = Isometry2f::Identity();
    if (_round_num > 1) {
      delta = _fixed_slice->inverse() * (*_moving_slice);
    }
    this->_factor->setMeasurement(delta);
    this->_factor->setInformationMatrix(Matrix3f::Identity() * 1e2); //< hack //ds TODO explain hack
  }

  void MultiAligner2DSliceOdomPrior::init(MultiAligner2DSliceOdomPrior::AlignerType* aligner_) {
    _aligner = aligner_;
    setupFactor();
    _aligner->setEstimate(_factor->measurement());
    ++_round_num;
  }

  void MultiAligner3DSliceOdomPrior::setupFactor() {
    Isometry3f delta = Isometry3f::Identity();
    if (_round_num > 1) {
      delta = _fixed_slice->inverse() * (*_moving_slice);
    }
    this->_factor->setMeasurement(delta);
  }

  void MultiAligner3DSliceOdomPrior::init(MultiAligner3DSliceOdomPrior::AlignerType* aligner_) {
    _aligner = aligner_;
    setupFactor();
    _aligner->setEstimate(_factor->measurement());
    ++_round_num;
  }

  void slam_interfaces_registerTypes() {
    srrg2_solver::solver_registerTypes();
    srrg2_solver::registerTypes2D();
    srrg2_solver::registerTypes3D();

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MeasurementAdaptorOdom2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner2DSliceOdomPrior);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner3DSliceOdomPrior);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAlignerSliceMotionModel2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAlignerSliceMotionModel3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner2DTerminationCriteriaStandard);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner3DTerminationCriteriaStandard);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerCorrespondence2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerCorrespondence3D);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiTrackerSliceEstimationBuffer2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiTrackerSliceEstimationBuffer3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiTracker2DSliceOdomPrior);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiTracker3DSliceOdomPrior);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MeasurementAdaptorOdom3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MeasurementAdaptorTrackerEstimate2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MeasurementAdaptorTrackerEstimate3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiTracker2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiTracker3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner3DQR);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MotionModelConstantVelocity2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MotionModelConstantVelocity3D);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMap2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMap3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMapSplittingCriterionDistance2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMapSplittingCriterionDistance3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMapSplittingCriterionRotation3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMapSplittingCriterionViewpoint3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMapSplittingCriterionVisibility3D);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LoopClosure2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LoopClosure3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiGraphSLAM2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiGraphSLAM3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiRelocalizer2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiRelocalizer3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiLoopDetectorBruteForce2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiLoopDetectorBruteForce3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMapSelectorBreadthFirst2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(LocalMapSelectorBreadthFirst3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiLoopDetectorHBST2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiLoopDetectorHBST3D);

    // TODO recover this when we can convert ros msg camera matrix to 2D matrix
    //    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(InitializerCamera2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(InitializerCamera3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(InitializerStereoCamera3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiInitializer);
  }

} // namespace srrg2_slam_interfaces
