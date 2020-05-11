#pragma once
#include "loop_detector.h"
#include "srrg2_slam_interfaces/mapping/local_map.h"
#include "srrg2_slam_interfaces/registration/local_map_selectors/local_map_selector_breadth_first.h"

namespace srrg2_slam_interfaces {

  template <typename SLAMAlgorithmType_, typename AlignerType_>
  class MultiLoopDetectorBruteForce_ : public LoopDetector_<SLAMAlgorithmType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SLAMAlgorithmType    = SLAMAlgorithmType_;
    using AlignerType          = AlignerType_;
    using LoopClosureType      = typename SLAMAlgorithmType::LoopClosureType;
    using LocalMapType         = typename SLAMAlgorithmType::LocalMapType;
    using EstimateType         = typename LocalMapType::EstimateType;
    using LocalMapSelectorType = typename LoopDetector_<SLAMAlgorithmType>::LocalMapSelectorType;
    using ThisType             = MultiLoopDetectorBruteForce_<SLAMAlgorithmType, AlignerType>;
    using BaseType             = LoopDetector_<SLAMAlgorithmType>;

    PARAM(srrg2_core::PropertyConfigurable_<AlignerType>,
          relocalize_aligner,
          "aligner used to register loop closures",
          0,
          nullptr);
    PARAM(srrg2_core::PropertyUnsignedInt,
          relocalize_min_inliers,
          "minimum number of inliers for success [int]",
          500,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          relocalize_max_chi_inliers,
          "maximum chi per inlier for success [chi]",
          0.005,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          relocalize_min_inliers_ratio,
          "minimum fraction of inliers over total correspondences "
          "[num_inliers/num_correspondences]",
          0.7,
          nullptr);

    virtual ~MultiLoopDetectorBruteForce_() {
    }

    virtual void compute() override;
  };

} // namespace srrg2_slam_interfaces
