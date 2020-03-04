#pragma once
#include "loop_detector.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  template <typename SlamAlgorithmType_>
  class RelocalizerBase_ : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SlamAlgorithmType  = SlamAlgorithmType_;
    using LoopClosureType    = typename SlamAlgorithmType::LoopClosureType;
    using LocalMapType       = typename SlamAlgorithmType::LocalMapType;
    using EstimateType       = typename SlamAlgorithmType::EstimateType;
    using TrackerType        = typename SlamAlgorithmType::TrackerType;
    using LoopClosurePtrType = std::shared_ptr<LoopClosureType>;
    using LoopClosurePtrSet  = GraphItemPtrSet_<LoopClosurePtrType>;

    PARAM(PropertyFloat, max_translation, "max translation to attempt a jump", 3, 0);

    virtual void compute() = 0;

    inline LocalMapType* relocalizeMap() const {
      return _relocalize_map;
    }
    inline EstimateType poseInLocalMap() const {
      return _pose_in_local_map;
    }
    inline LoopClosurePtrType relocalizedClosure() const {
      return _relocalized_closure;
    }

    inline void setSlamAlgorithm(SlamAlgorithmType* slam_) {
      _slam = slam_;
    }

    void setClosureCandidates(const LoopClosurePtrSet& closures_) {
      _closure_candidates = &closures_;
    }

  protected:
    const LoopClosurePtrSet* _closure_candidates = nullptr;
    EstimateType _pose_in_local_map              = EstimateType::Identity();
    LocalMapType* _relocalize_map                = nullptr;
    LoopClosurePtrType _relocalized_closure      = nullptr;
    SlamAlgorithmType* _slam                     = nullptr;
  };

} // namespace srrg2_slam_interfaces
