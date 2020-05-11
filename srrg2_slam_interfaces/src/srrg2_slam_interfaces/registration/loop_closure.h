#pragma once
#include "srrg2_slam_interfaces/mapping/local_map.h"
#include <srrg_data_structures/correspondence.h>
#include <srrg_solver/solver_core/factor_base.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h>

namespace srrg2_solver {
  class SE2PosePoseMultiErrorFactorAD;
  class SE3PosePoseMultiErrorFactorAD;
} // namespace srrg2_solver

namespace srrg2_slam_interfaces {

  /**
   * @brief a loop closure struct
   * Connects two local maps with a factor.
   * We assume that the target local map comes from a FIXED part of the graph (or a detached graph
   * in relocalization processes)
   */
  template <typename LocalMapType_, typename FactorType_>
  struct LoopClosure_ : public FactorType_ {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // fetch types
    using FactorType            = FactorType_;
    using LocalMapType          = LocalMapType_;
    using ThisType              = LoopClosure_<LocalMapType, FactorType>;
    using MeasurementType       = typename FactorType::MeasurementType;
    using InformationMatrixType = typename FactorType::InformationMatrixType;
    using EstimateType          = typename LocalMapType::EstimateType;

    //! default constructor used for serialization ONLY TODO buargh
    LoopClosure_() :
      pose_in_target(EstimateType::Identity()),
      chi_inliers(0),
      chi(0),
      num_inliers(0),
      num_correspondences(0),
      correspondences(srrg2_core::CorrespondenceVector()),
      source_graph_id(-1),
      target_graph_id(-1) {
    }

    //! minimal closure constructor with identity constraint
    LoopClosure_(
      srrg2_solver::FactorBase::Id graph_id_ /**< if of the factor in the graph*/,
      LocalMapType* source_ /**< source local map (the moving one)*/,
      LocalMapType* target_ /**< target local map (the fixed one)*/,
      const MeasurementType& measurement_ =
        MeasurementType::Identity() /**< local maps computed displacement */,
      const InformationMatrixType& info_ =
        InformationMatrixType::Identity() /**< information matrixe related to the measurement */,
      const EstimateType& pose_in_target_ =
        EstimateType::Identity() /**< pose of the robot in the target (fixed) local map*/,
      const float& chi_inliers_          = 1e9 /**< stats, value of the chi for inliers */,
      const size_t& num_inliers_         = 0 /**< stats, number of inliers */,
      const size_t& num_correspondences_ = 0 /**< stats, number of correspondences*/,
      srrg2_core::CorrespondenceVector correspondences_source_target_ =
        srrg2_core::CorrespondenceVector()) /**< correspondence vector*/ :
      pose_in_target(pose_in_target_),
      chi_inliers(chi_inliers_),
      chi(1e9),
      num_inliers(num_inliers_),
      num_correspondences(num_correspondences_),
      correspondences(std::move(correspondences_source_target_)),
      source_graph_id(source_ != nullptr ? source_->graphId() : -1),
      target_graph_id(target_ != nullptr ? target_->graphId() : -1) {
      ThisType::setGraphId(graph_id_);
      ThisType::setMeasurement(measurement_);
      ThisType::setInformationMatrix(info_);
      ThisType::setEnabled(false);

      // ds set the variables id in the graph
      // ds at this point the variables are not necessarily already present in the graph
      assert(source_);
      ThisType::setVariableId(0, source_->graphId());
      assert(target_);
      ThisType::setVariableId(1, target_->graphId());
    }

    /**
     * @brief get the source (moving) local map
     * @return the source (moving) local map
     */
    LocalMapType* source() {
      // ds can be null in case the closure is not instantiated in the graph
      return dynamic_cast<LocalMapType*>(ThisType::variable(0));
    }

    /**
     * @brief get the target (fixed) local map
     * @return the target (fixed) local map
     */
    LocalMapType* target() {
      // ds can be null in case the closure is not instantiated in the graph
      return dynamic_cast<LocalMapType*>(ThisType::variable(1));
    }

    const EstimateType pose_in_target; /**< pose of the robot in the target (fixed) local map*/
    const float chi_inliers;           /**< stats, value of the chi for inliers */
    const float chi;                   /**< ???!!! this is not used and fixed to 1e9 */
    const size_t num_inliers;          /**< stats, number of inliers */
    const size_t num_correspondences;  /**< stats, number of correspondences, is this needed?? can't
                                          be computed by corr.size()?*/
    const srrg2_core::CorrespondenceVector correspondences; /**< correspondence vector*/
    const int64_t source_graph_id;                          /**< graph ID for moving local map */
    const int64_t target_graph_id;                          /**< graph ID for fixed local map */
  };

  using LoopClosure2D    = LoopClosure_<LocalMap2D, srrg2_solver::SE2PosePoseGeodesicErrorFactor>;
  using LoopClosure3D    = LoopClosure_<LocalMap3D, srrg2_solver::SE3PosePoseGeodesicErrorFactor>;
  using LoopClosure2DPtr = std::shared_ptr<LoopClosure2D>;
  using LoopClosure3DPtr = std::shared_ptr<LoopClosure3D>;

} // namespace srrg2_slam_interfaces
