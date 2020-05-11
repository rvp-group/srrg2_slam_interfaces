#pragma once
#include "srrg2_slam_interfaces/registration/local_map_selectors/local_map_selector.h"
#include <srrg_solver/solver_core/solver.h>
#include <srrg_solver/utils/factor_graph_utils/factor_graph_visit.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief comparator to reduce indeterminism in sets
   */
  template <typename GraphItemPtr_>
  struct GraphIdPtrComparator_ {
    inline bool operator()(const GraphItemPtr_& a, const GraphItemPtr_& b) const {
      return a->graphId() < b->graphId();
    }
  };

  template <typename GraphItemPtr_>
  using GraphItemPtrSet_ = std::set<GraphItemPtr_, GraphIdPtrComparator_<GraphItemPtr_>>;

  /**
   * @brief loop detector base class
   * The loop detector finds a set of closure candidates between similar local maps
   */
  template <typename SLAMAlgorithmType_>
  class LoopDetector_ : public srrg2_core::Configurable {
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

    PARAM(srrg2_core::PropertyConfigurable_<LocalMapSelectorType>,
          local_map_selector,
          "module used to figure out which local maps should be checked",
          nullptr,
          nullptr);

    /**
     * @brief compute a set of closure candidates
     */
    virtual void compute() = 0;

    /**
     * @brief it returns a set of local maps that has been checked for loop closure
     * @return attempted local maps for closure
     */
    inline LocalMapRawPtrContainer& attemptedClosures() {
      return _attempted_closures;
    }

    /**
     * @brief the detected loop closures
     * @return detected loop closures
     */
    inline LoopClosurePtrContainer& detectedClosures() {
      return _detected_closures;
    }

    /**
     * @brief get access to the whole SLAM algorithm
     * @param[in] slam algorithm
     */
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
    SLAMAlgorithmType* _slam = nullptr;          /**< the slam algorithm*/
    LocalMapRawPtrContainer _attempted_closures; //< closures tried against
    LoopClosurePtrContainer _detected_closures;  //< updated after detection
  };

} // namespace srrg2_slam_interfaces
