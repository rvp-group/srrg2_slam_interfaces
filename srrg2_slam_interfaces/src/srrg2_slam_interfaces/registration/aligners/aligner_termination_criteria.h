#pragma once
#include <srrg_config/configurable.h>
#include <srrg_solver/solver_core/behavior_analyzer.h>

namespace srrg2_slam_interfaces {

  class AlignerBase;

  /**
   * @brief Termination criteria for an aligner. Decides when a
   * Must be specialized implementing the hasToStop function
   *
   */
  class AlignerTerminationCriteriaBase : public srrg2_core::Configurable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    friend class AlignerBase;
    /**
     * @brief initialize the criteria by storing the aligner raw pointer
     * @param[in] aligner_: raw pointer of the aligner
     */
    virtual void init(AlignerBase* aligner_);
    /**
     * @brief Implement the decision function for aligner run termination
     * @return true if the criterion is satisfied
     */
    virtual bool hasToStop() = 0;

  protected:
    AlignerBase* _aligner = nullptr; /**< base aligner class, to be casted in the derived classes */
  };

  template <typename AlignerType_>
  class AlignerTerminationCriteriaStandard_ : public AlignerTerminationCriteriaBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType    = AlignerTerminationCriteriaStandard_<AlignerType_>;
    using AlignerType = AlignerType_;

    PARAM(srrg2_core::PropertyInt, window_size, "window used to compute the thresholds", 5, 0);
    PARAM(srrg2_core::PropertyInt,
          num_correspondences_range,
          "range of correspondences at steady state",
          20,
          0);
    PARAM(srrg2_core::PropertyInt,
          num_inliers_range,
          "range of num inliers at steady state",
          20,
          0);
    PARAM(srrg2_core::PropertyInt,
          num_outliers_range,
          "range of num outliers at steady state",
          20,
          0);
    PARAM(srrg2_core::PropertyFloat, chi_epsilon, "chi decay", 0.2, 0);

    void init(AlignerBase* aligner_) override;

    bool hasToStop() override;

  protected:
    srrg2_solver::BehaviorAnalyzer_<int>
      _num_correspondences;                             /**< correspondence buffer to be analyzed */
    srrg2_solver::BehaviorAnalyzer_<int> _num_outliers; /**< outliers buffer to be analyzed */
    srrg2_solver::BehaviorAnalyzer_<int> _num_inliers;  /**< inliers buffer to be analyzed */
    srrg2_solver::BehaviorAnalyzer_<float> _chi_norm;   /**< chi buffer to be analyzed */

    AlignerType* _typed_aligner = nullptr; /**< aligner casted to effective type */
    int _num_iteration          = 0;       /**< cached number of iterations */
  };

} // namespace srrg2_slam_interfaces
