#pragma once
#include "local_map.h"
#include <srrg_data_structures/correspondence.h>
#include <srrg_solver/solver_core/factor_base.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h>

namespace srrg2_solver {
  class SE2PosePoseMultiErrorFactorAD;
  class SE3PosePoseMultiErrorFactorAD;
} // namespace srrg2_solver

namespace srrg2_slam_interfaces {

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
    LoopClosure_(srrg2_solver::FactorBase::Id graph_id_,
                 LocalMapType* source_,
                 LocalMapType* target_,
                 const MeasurementType& measurement_ = MeasurementType::Identity(),
                 const InformationMatrixType& info_  = InformationMatrixType::Identity(),
                 const EstimateType& pose_in_to_     = EstimateType::Identity(),
                 const float& chi_inliers_           = 1e9,
                 const size_t& num_inliers_          = 0,
                 const size_t& num_correspondences_  = 0,
                 srrg2_core::CorrespondenceVector correspondences_source_target_ =
                   srrg2_core::CorrespondenceVector()) :
      pose_in_target(pose_in_to_),
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

    LocalMapType* source() {
      // ds can be null in case the closure is not instantiated in the graph
      return dynamic_cast<LocalMapType*>(ThisType::variable(0));
    }
    LocalMapType* target() {
      // ds can be null in case the closure is not instantiated in the graph
      return dynamic_cast<LocalMapType*>(ThisType::variable(1));
    }

    const EstimateType pose_in_target;
    const float chi_inliers;
    const float chi;
    const size_t num_inliers;
    const size_t num_correspondences;
    const srrg2_core::CorrespondenceVector correspondences;
    const int64_t source_graph_id;
    const int64_t target_graph_id;
  };

  using LoopClosure2D    = LoopClosure_<LocalMap2D, srrg2_solver::SE2PosePoseGeodesicErrorFactor>;
  using LoopClosure3D    = LoopClosure_<LocalMap3D, srrg2_solver::SE3PosePoseGeodesicErrorFactor>;
  using LoopClosure2DPtr = std::shared_ptr<LoopClosure2D>;
  using LoopClosure3DPtr = std::shared_ptr<LoopClosure3D>;

} // namespace srrg2_slam_interfaces
