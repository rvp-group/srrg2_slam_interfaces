#pragma once
#include "relocalizer.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  /**
   * @brief Multi-cue relocalizer
   */
  template <typename SlamAlgorithmType_>
  class MultiRelocalizer_ : public RelocalizerBase_<SlamAlgorithmType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SlamAlgorithmType        = SlamAlgorithmType_;
    using LoopClosureType          = typename SlamAlgorithmType::LoopClosureType;
    using LocalMapType             = typename SlamAlgorithmType::LocalMapType;
    using EstimateType             = typename SlamAlgorithmType::EstimateType;
    using TrackerType              = typename SlamAlgorithmType::TrackerType;
    using AlignerType              = typename TrackerType::AlignerType;
    using MeasurementContainerType = typename TrackerType::MeasurementContainerType;
    using ThisType                 = MultiRelocalizer_<SlamAlgorithmType>;

    PARAM(PropertyConfigurable_<AlignerType>,
          aligner,
          "aligner unit used to determine the best nearby local map for relocalization",
          nullptr,
          nullptr);
    PARAM(PropertyInt,
          relocalize_min_inliers,
          "minimum number of inliers for sucessful relocalization",
          500,
          nullptr);
    PARAM(PropertyFloat,
          relocalize_max_chi_inliers,
          "maximum chi per inlier for sucessful relocalization",
          0.005,
          nullptr);
    PARAM(PropertyFloat,
          relocalize_min_inliers_ratio,
          "minimum fraction of inliers out of the total correspondences",
          0.7,
          nullptr);

    void compute() override;
  };

} // namespace srrg2_slam_interfaces
