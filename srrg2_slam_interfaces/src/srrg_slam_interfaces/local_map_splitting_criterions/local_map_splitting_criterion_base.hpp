#pragma once
#include <srrg_config/configurable.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename SlamAlgorithmType_>
  class LocalMapSplittingCriterionBase_ : public Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void compute() = 0;
    bool hasToSplit() {
      return _has_to_split;
    }
    void setSlamAlgorithm(SlamAlgorithmType_* slam_) {
      _slam = slam_;
    }

  protected:
    SlamAlgorithmType_* _slam = nullptr;
    bool _has_to_split        = false;
  };

} // namespace srrg2_slam_interfaces
