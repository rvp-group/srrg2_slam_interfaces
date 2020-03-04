#pragma once
#include "local_map_selectors/local_map_selector.h"
#include "srrg_solver/utils/factor_graph_utils/factor_graph_visit.h"
#include "srrg_solver/solver_core/solver.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename GraphItemPtr_>
  struct GraphIdPtrComparator_ {
    inline bool operator()(const GraphItemPtr_& a, const GraphItemPtr_& b) const {
      return a->graphId() < b->graphId();
    }
  };

  template <typename GraphItemPtr_>
  using GraphItemPtrSet_ = std::set<GraphItemPtr_, GraphIdPtrComparator_<GraphItemPtr_>>;

  template <typename SLAMAlgorithmType_>
  class LoopDetector_ : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SLAMAlgorithmType       = SLAMAlgorithmType_;
    using ThisType                = LoopDetector_<SLAMAlgorithmType>;
    using LoopClosureType         = typename SLAMAlgorithmType::LoopClosureType;
    using LocalMapType            = typename LoopClosureType::LocalMapType;
    using InformationMatrixType   = typename LoopClosureType::InformationMatrixType;
    using LocalMapSelectorType    = LocalMapSelector_<SLAMAlgorithmType>;
    using LoopClosurePtrContainer = std::vector<std::shared_ptr<LoopClosureType>>;
    using LocalMapRawPtrContainer = GraphItemPtrSet_<LocalMapType*>;
    using MeasurementType         = typename LoopClosureType::MeasurementType;

    PARAM(PropertyConfigurable_<LocalMapSelectorType>,
          local_map_selector,
          "module used to figure out which local maps should be checked",
          nullptr,
          nullptr);

    virtual void compute() = 0;

    inline LocalMapRawPtrContainer& attemptedClosures() {
      return _attempted_closures;
    }

    inline LoopClosurePtrContainer& detectedClosures() {
      return _detected_closures;
    }

    void setSLAMAlgorithm(SLAMAlgorithmType* slam_) {
      _slam = slam_;
    }

    //! for loop detectors with internal database we need an additional channel of interaction to
    //! tell the detector whether to integrate the last local map query or not
    //! (e.g. no integration would be desirable for relocalization, hence train is not called)
    virtual void addPreviousQuery() {
      // ds default is not required
    }

  protected:
    SLAMAlgorithmType* _slam = nullptr;
    LocalMapRawPtrContainer _attempted_closures; // closures tried against
    LoopClosurePtrContainer _detected_closures;  // updated after detection
  };

} // namespace srrg2_slam_interfaces
