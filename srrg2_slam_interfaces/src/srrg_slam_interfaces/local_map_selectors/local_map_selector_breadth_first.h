#pragma once
#include "local_map_selector.h"
#include <srrg_solver/utils/factor_graph_utils/factor_graph_visit.h>

namespace srrg2_slam_interfaces {

  template <typename SLAMAlgorithmType_>
  class LocalMapSelectorBreadthFirst_ : public LocalMapSelector_<SLAMAlgorithmType_> {
  public:
    using SLAMAlgorithmType     = SLAMAlgorithmType_;
    using LoopClosureType       = typename SLAMAlgorithmType::LoopClosureType;
    using LocalMapType          = typename LoopClosureType::LocalMapType;
    using EstimateType          = typename LocalMapType::EstimateType;
    using InformationMatrixType = typename LoopClosureType::InformationMatrixType;
    using ClosureHint           = typename LocalMapSelector_<SLAMAlgorithmType>::ClosureHint;
    using ClosureHintPtr        = typename LocalMapSelector_<SLAMAlgorithmType>::ClosureHintPtr;
    using ClosureHintPtrSet     = typename LocalMapSelector_<SLAMAlgorithmType>::ClosureHintPtrSet;

    PARAM(srrg2_core::PropertyInt,
          relocalize_range_scale,
          "distance of candidate closures [int, magnitude]",
          2,
          0);
    PARAM(srrg2_core::PropertyInt,
          aggressive_relocalize_graph_distance,
          "min distance between local maps to start aggressive [int, distance on graph]",
          10,
          0);
    PARAM(srrg2_core::PropertyInt,
          aggressive_relocalize_graph_max_range,
          "max distance to attempt global relocalize [int, distance on graph]",
          20,
          0);
    PARAM(srrg2_core::PropertyFloat,
          aggressive_relocalize_range_increase_per_edge,
          "factor to pimp the chi2 threshold depending on the lenght [float, magnitude]",
          0.1,
          0);
    PARAM(srrg2_core::PropertyFloat,
          max_local_map_distance,
          "max distance in meters [float, magnitude]",
          1.0,
          0);

    void compute() override;

    LocalMapSelectorBreadthFirst_() {
      std::shared_ptr<srrg2_solver::FactorGraphVisitPolicyBase> policy(
        new srrg2_solver::FactorGraphVisitPolicyBase);
      std::shared_ptr<srrg2_solver::FactorGraphVisitCostUniform> cost_uniform(
        new srrg2_solver::FactorGraphVisitCostUniform);
      policy->param_cost_function.setValue(cost_uniform);
      _visit.param_cost_policies.pushBack(policy);
    }

  protected:
    srrg2_solver::FactorGraphVisit _visit;
  };
} // namespace srrg2_slam_interfaces
