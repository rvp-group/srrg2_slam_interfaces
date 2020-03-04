#pragma once
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_point2point_error_factor.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_prior_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_point2point_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_prior_error_factor_ad.h>

#include "aligner_termination_criteria_standard.h"
#include "measurement_adaptor_odom.h"
#include "measurement_adaptor_tracker_estimate.hpp"
#include "multi_aligner.h"
#include "multi_aligner_slice_motion_model.hpp"
#include "multi_aligner_slice_prior.h"

#include "multi_tracker.h"
#include "multi_tracker_slice_estimation_buffer.hpp"
#include "multi_tracker_slice_prior.h"

#include "local_map.h"
#include "local_map_selectors/local_map_selector_breadth_first.h"
#include "local_map_selectors/local_map_selector_user_defined.h"

#include "initializers/initializer_camera.h"

#include "local_map_splitting_criterions/local_map_splitting_criterion_rotation.hpp"
#include "local_map_splitting_criterions/local_map_splitting_criterion_translation.hpp"
#include "local_map_splitting_criterions/local_map_splitting_criterion_viewpoint.hpp"
#include "local_map_splitting_criterions/local_map_splitting_criterion_visibility.hpp"

#include "motion_models/motion_model_constant_velocity.hpp"

#include "loop_closure.h"
#include "multi_graph_slam.h"
#include "multi_loop_detector_brute_force.h"
#include "multi_loop_detector_hbst.h"
#include "multi_relocalizer.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_solver;

  // term crits for aligners
  using MultiAligner2DTerminationCriteriaStandard =
    AlignerTerminationCriteriaStandard_<MultiAligner2D>;
  using MultiAligner3DTerminationCriteriaStandard =
    AlignerTerminationCriteriaStandard_<MultiAligner3DQR>;

  // priors
  class MultiAligner2DSliceOdomPrior
    : public MultiAlignerSlicePrior_<SE2PriorErrorFactor, Isometry2f, Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void init(AlignerType* aligner_) override;
    size_t _round_num = 0;

  protected:
    void setupFactor() override;
  };

  using MultiAligner2DSliceOdomPriorPtr = std::shared_ptr<MultiAligner2DSliceOdomPrior>;

  class MultiTracker2DSliceOdomPrior
    : public MultiTrackerSlicePrior_<Isometry2f, Isometry2f, Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MultiTracker2DSliceOdomPrior() {
      this->param_adaptor.setValue(
        std::shared_ptr<MeasurementAdaptorOdom2D>(new MeasurementAdaptorOdom2D));
    }
  };

  using MultiTracker2DSliceOdomPriorPtr = std::shared_ptr<MultiTracker2DSliceOdomPrior>;

  class MultiAligner3DSliceOdomPrior
    : public MultiAlignerSlicePrior_<SE3PriorErrorFactorAD, Isometry3f, Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    void init(AlignerType* aligner_) override;
    size_t _round_num = 0;

  protected:
    void setupFactor() override;
  };

  using MultiAligner3DSliceOdomPriorPtr = std::shared_ptr<MultiAligner2DSliceOdomPrior>;

  class MultiTracker3DSliceOdomPrior
    : public MultiTrackerSlicePrior_<Isometry3f, Isometry3f, Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MultiTracker3DSliceOdomPrior() {
      this->param_adaptor.setValue(
        std::shared_ptr<MeasurementAdaptorOdom3D>(new MeasurementAdaptorOdom3D));
    }
  };

  using MultiTracker3DSliceOdomPriorPtr = std::shared_ptr<MultiTracker3DSliceOdomPrior>;

  // graph slams
  using MultiGraphSLAM2D    = MultiGraphSLAM_<LoopClosure2D>;
  using MultiGraphSLAM3D    = MultiGraphSLAM_<LoopClosure3D>;
  using MultiGraphSLAM2DPtr = std::shared_ptr<MultiGraphSLAM2D>;
  using MultiGraphSLAM3DPtr = std::shared_ptr<MultiGraphSLAM3D>;

  using MultiRelocalizer2D = MultiRelocalizer_<MultiGraphSLAM2D>;
  using MultiRelocalizer3D = MultiRelocalizer_<MultiGraphSLAM3D>;

  using LocalMapSelectorBreadthFirst2D = LocalMapSelectorBreadthFirst_<MultiGraphSLAM2D>;
  using LocalMapSelectorBreadthFirst3D = LocalMapSelectorBreadthFirst_<MultiGraphSLAM3D>;

  using LocalMapSplittingCriterionDistance2D =
    LocalMapSplittingCriterionDistance_<MultiGraphSLAM2D>;
  using LocalMapSplittingCriterionDistance3D =
    LocalMapSplittingCriterionDistance_<MultiGraphSLAM3D>;
  using LocalMapSplittingCriterionRotation2D =
    LocalMapSplittingCriterionRotation_<MultiGraphSLAM2D>;
  using LocalMapSplittingCriterionRotation3D =
    LocalMapSplittingCriterionRotation_<MultiGraphSLAM3D>;
  using LocalMapSplittingCriterionViewpoint3D =
    LocalMapSplittingCriterionViewpoint_<MultiGraphSLAM3D>;
  using LocalMapSplittingCriterionVisibility3D =
    LocalMapSplittingCriterionVisbility_<MultiGraphSLAM3D>;

  using MultiLoopDetectorBruteForce2D =
    MultiLoopDetectorBruteForce_<MultiGraphSLAM2D, MultiAligner2D>;
  using MultiLoopDetectorBruteForce3D =
    MultiLoopDetectorBruteForce_<MultiGraphSLAM3D, MultiAligner3DQR>;

  using MultiLoopDetectorHBST2D =
    MultiLoopDetectorHBST_<MultiGraphSLAM2D, MultiAligner2D, SE2Point2PointErrorFactor>;
  using MultiLoopDetectorHBST3D =
    MultiLoopDetectorHBST_<MultiGraphSLAM3D, MultiAligner3DQR, SE3Point2PointErrorFactor>;
  using MultiLoopDetectorHBST2DPtr = std::shared_ptr<MultiLoopDetectorHBST2D>;
  using MultiLoopDetectorHBST3DPtr = std::shared_ptr<MultiLoopDetectorHBST3D>;

  void slam_interfaces_registerTypes() __attribute__((constructor));

} // namespace srrg2_slam_interfaces
