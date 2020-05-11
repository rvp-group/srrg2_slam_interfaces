#pragma once

#include "local_map_selector.h"

namespace srrg2_slam_interfaces {

  /**
   * @brief local map selector structure for already known closure candidates
   */
  template <typename SLAMAlgorithmType_>
  class LocalMapSelectorUserDefined_ : public LocalMapSelector_<SLAMAlgorithmType_> {
  public:
    using SLAMAlgorithmType = SLAMAlgorithmType_;
    using LoopClosureType   = typename SLAMAlgorithmType::LoopClosureType;
    using LocalMapType      = typename LoopClosureType::LocalMapType;
    using EstimateType      = typename LocalMapType::EstimateType;
    using ClosureHint       = typename LocalMapSelector_<SLAMAlgorithmType>::ClosureHint;
    using ClosureHintPtr    = typename LocalMapSelector_<SLAMAlgorithmType>::ClosureHintPtr;
    using ClosureHintPtrSet = typename LocalMapSelector_<SLAMAlgorithmType>::ClosureHintPtrSet;
    using ThisType          = LocalMapSelectorUserDefined_<SLAMAlgorithmType>;
    using BaseType          = LocalMapSelector_<SLAMAlgorithmType>;

    void compute() override;
  };

} // namespace srrg2_slam_interfaces
