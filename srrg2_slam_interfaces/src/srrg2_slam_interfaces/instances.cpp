#include "instances.h"

#include "mapping/merger_correspondence_homo_impl.cpp"
#include "registration/aligners/aligner_slice_processor_impl.cpp"
#include "registration/aligners/aligner_termination_criteria_impl.cpp"
#include "registration/aligners/multi_aligner_impl.cpp"
#include "registration/local_map_selectors/local_map_selector_breadth_first_impl.cpp"
#include "registration/local_map_selectors/local_map_selector_user_defined_impl.cpp"
#include "registration/loop_detector/multi_loop_detector_brute_force_impl.cpp"
#include "registration/loop_detector/multi_loop_detector_hbst_impl.cpp"
#include "registration/relocalization/multi_relocalizer_impl.cpp"
#include "system/multi_graph_slam_impl.cpp"
#include "trackers/multi_tracker_impl.cpp"
#include "trackers/tracker_slice_processor_impl.cpp"
#include <srrg_config/configurable_visual_shell.h>
#include <srrg_solver/solver_core/instances.h>
#include <srrg_solver/variables_and_factors/types_2d/instances.h>
#include <srrg_solver/variables_and_factors/types_3d/instances.h>

// ds reduce digital footprint and enforce linker sanity
#define BOSS_REGISTER_CLASS_LINKER_FRIENDLY(CLASS_) \
  CLASS_ dummy_##CLASS_;                            \
  BOSS_REGISTER_CLASS(CLASS_)

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  void srrg2_slam_interfaces_registerTypes() {
    srrg2_solver::solver_registerTypes();
    srrg2_solver::variables_and_factors_2d_registerTypes();
    srrg2_solver::variables_and_factors_3d_registerTypes();

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MultiAligner2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(RawDataPreprocessorOdom2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(AlignerSliceOdom2DPrior);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(AlignerSliceOdom3DPrior);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(AlignerSliceMotionModel2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(AlignerSliceMotionModel3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(AlignerTerminationCriteriaStandard2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(AlignerTerminationCriteriaStandard3DQR);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerCorrespondencePointNormal2f);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(MergerCorrespondencePointIntensityDescriptor3f);

    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(TrackerSliceProcessorEstimationBuffer2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(TrackerSliceProcessorEstimationBuffer3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(TrackerSliceProcessorPriorOdom2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(TrackerSliceProcessorPriorOdom3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(RawDataPreprocessorOdom3D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(RawDataPreprocessorTrackerEstimate2D);
    BOSS_REGISTER_CLASS_LINKER_FRIENDLY(RawDataPreprocessorTrackerEstimate3D);
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
