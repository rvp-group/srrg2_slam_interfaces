#pragma once
#include "srrg2_slam_interfaces/registration/loop_detector/loop_detector.h"

namespace srrg2_slam_interfaces {
  /**
   * @brief base class for relocalizer/global localizer
   * built upon a slam algorithm
   * The relocalizer finds a loop closure given the actual sensor reading
   */
  template <typename SLAMAlgorithmType_>
  class RelocalizerBase_ : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SLAMAlgorithmType  = SLAMAlgorithmType_;
    using LoopClosureType    = typename SLAMAlgorithmType::LoopClosureType;
    using LocalMapType       = typename SLAMAlgorithmType::LocalMapType;
    using EstimateType       = typename SLAMAlgorithmType::EstimateType;
    using TrackerType        = typename SLAMAlgorithmType::TrackerType;
    using LoopClosureTypePtr = std::shared_ptr<LoopClosureType>;
    using LoopClosurePtrSet  = GraphItemPtrSet_<LoopClosureTypePtr>;

    PARAM(PropertyFloat, max_translation, "max translation to attempt a jump", 3, 0);

    /**
     * @brief compute the local map where the robot is moving and its pose in the local map
     */
    virtual void compute() = 0;

    /**
     * @brief map where the robot is relocalized
     * @return pointer to the local map
     */
    inline LocalMapType* relocalizationMap() const {
      return _relocalization_map;
    }

    /**
     * @brief pose of the robot in local map
     * @return robot in local map
     */
    inline EstimateType robotInLocalMap() const {
      return _robot_in_local_map;
    }

    /**
     * @brief loop closure that links the current and relocalization map
     */
    inline LoopClosureTypePtr relocalizedClosure() const {
      return _relocalized_closure;
    }

    /**
     * @brief get access to the whole SLAM algorithm
     * @param[in] slam algorithm
     */
    inline void setSLAMAlgorithm(SLAMAlgorithmType* slam_) {
      _slam = slam_;
    }

    /**
     * @brief set a pool of closure candidates
     * @param[in] closure candidates
     */
    void setClosureCandidates(const LoopClosurePtrSet& closures_) {
      _closure_candidates = &closures_;
    }

  protected:
    const LoopClosurePtrSet* _closure_candidates =
      nullptr; /**< set of closure candidates where to check for relocalization*/
    EstimateType _robot_in_local_map =
      EstimateType::Identity(); /**< pose of the robot in relocalization local map*/
    LocalMapType* _relocalization_map       = nullptr; /**< relocalization local map*/
    LoopClosureTypePtr _relocalized_closure = nullptr; /**< best closure for relocalization*/
    SLAMAlgorithmType* _slam                = nullptr; /**< the slam algorithm*/
  };

} // namespace srrg2_slam_interfaces
