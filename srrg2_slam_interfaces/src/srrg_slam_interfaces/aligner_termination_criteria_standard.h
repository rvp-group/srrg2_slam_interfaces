#pragma once
#include "aligner.h"
#include <srrg_solver/solver_core/behavior_analyzer.h>

namespace srrg2_slam_interfaces {

  template <typename AlignerType_>
  class AlignerTerminationCriteriaStandard_ : public AlignerTerminationCriteriaBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType    = AlignerTerminationCriteriaStandard_<AlignerType_>;
    using AlignerType = AlignerType_;

    PARAM(PropertyInt, window_size, "window used to compute the thresholds", 5, 0);
    PARAM(PropertyInt,
          num_correspondences_range,
          "range of correspondences at steady state",
          20,
          0);
    PARAM(PropertyInt, num_inliers_range, "range of num inliers at steady state", 20, 0);
    PARAM(PropertyInt, num_outliers_range, "range of num outliers at steady state", 20, 0);
    PARAM(PropertyFloat, chi_epsilon, "chi decay", 0.2, 0);

    void init(AlignerBase* aligner_) override;

    bool hasToStop() override;

  protected:
    srrg2_solver::BehaviorAnalyzer_<int> _num_correspondences_anal;
    srrg2_solver::BehaviorAnalyzer_<int> _num_outliers_anal;
    srrg2_solver::BehaviorAnalyzer_<int> _num_inliers_anal;
    srrg2_solver::BehaviorAnalyzer_<float> _chi_norm_anal;

    AlignerType* _typed_aligner                      = 0;
    CorrespondenceFinderBase* _correspondence_finder = 0;
    int _num_iteration                               = 0;
  };

} // namespace srrg2_slam_interfaces
