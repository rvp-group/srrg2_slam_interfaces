#pragma once
#include <srrg_config/configurable.h>
#include <srrg_solver/solver_core/factor_graph.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief base class for local map selector
   * creates a set of closure candidates
   */
  template <typename SLAMAlgorithmType_>
  class LocalMapSelector_ : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SLAMAlgorithmType     = SLAMAlgorithmType_;
    using LoopClosureType       = typename SLAMAlgorithmType::LoopClosureType;
    using LocalMapType          = typename LoopClosureType::LocalMapType;
    using EstimateType          = typename LocalMapType::EstimateType;
    using InformationMatrixType = typename LoopClosureType::InformationMatrixType;

    /**
     * @brief possible local map to attempt loop closure
     */
    struct ClosureHint {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      LocalMapType* local_map;
      EstimateType initial_guess;
      // if zero it performs an exhaustive search
      InformationMatrixType information_matrix;

      ClosureHint(LocalMapType* lmap,
                  const EstimateType& guess         = EstimateType::Identity(),
                  const InformationMatrixType& info = InformationMatrixType::Zero()) :
        local_map(lmap),
        initial_guess(guess),
        information_matrix(info) {
      }
    };
    using ClosureHintPtr = std::shared_ptr<ClosureHint>;

    /**
     * @brief organizers the set via graphID, to reduce indeterminism
     */
    struct ClosureHintPtrComparator {
      inline bool operator()(const ClosureHintPtr& a, const ClosureHintPtr& b) {
        return a->local_map->graphId() < b->local_map->graphId();
      }
    };
    using ClosureHintPtrSet = std::set<ClosureHintPtr, ClosureHintPtrComparator>;

    /**
     * @brief get access to the whole SLAM algorithm
     * @param[in] slam algorithm
     */
    void setSLAMAlgorithm(SLAMAlgorithmType* slam_) {
      _slam = slam_;
    }

    /**
     * @brief produces a set of loop candidates
     */
    virtual void compute() = 0;
    /**
     * @brief returns the set of candidates for loop closures, to be further checked
     */
    ClosureHintPtrSet& hints() {
      return _hints;
    }

  protected:
    ClosureHintPtrSet _hints;     /**< set of closure where to check*/
    SLAMAlgorithmType* _slam = 0; /**< the slam algorithm*/
  };
} // namespace srrg2_slam_interfaces
