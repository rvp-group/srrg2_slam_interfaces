#pragma once
#include <srrg_config/configurable.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief This module decides whether the conditions are sufficient to split the current map
   * and create a new local map
   * Builds on top of the SLAM algorithm
   */
  template <typename SlamAlgorithmType_>
  class LocalMapSplittingCriterionBase_ : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief performs to computation to understand if it is required a split in the current local
     * map
     */
    virtual void compute() = 0;

    /**
     * @brief says if it is needed to split
     * @return true if the current local map has to split
     */
    bool hasToSplit() {
      return _has_to_split;
    }

    /**
     * @brief sets the SLAM algorithm
     * @param[in] pointer to the slam algorithm
     */
    void setSLAMAlgorithm(SlamAlgorithmType_* slam_) {
      _slam = slam_;
    }

  protected:
    SlamAlgorithmType_* _slam = nullptr;
    bool _has_to_split        = false;
  };

} // namespace srrg2_slam_interfaces
