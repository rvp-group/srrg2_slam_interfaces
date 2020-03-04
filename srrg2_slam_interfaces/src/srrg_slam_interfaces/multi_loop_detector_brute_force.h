#pragma once
#include "local_map.h"
#include "local_map_selectors/local_map_selector_breadth_first.h"
#include "loop_detector.h"

namespace srrg2_slam_interfaces {

  template <typename SLAMAlgorithmType_, typename AlignerType_>
  class MultiLoopDetectorBruteForce_ : public LoopDetector_<SLAMAlgorithmType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using SLAMAlgorithmType    = SLAMAlgorithmType_;
    using LoopClosureType      = typename SLAMAlgorithmType::LoopClosureType;
    using LocalMapType         = typename SLAMAlgorithmType::LocalMapType;
    using EstimateType         = typename LocalMapType::EstimateType;
    using LocalMapSelectorType = typename LoopDetector_<SLAMAlgorithmType_>::LocalMapSelectorType;
    using AlignerType          = AlignerType_;
    using ThisType             = MultiLoopDetectorBruteForce_<SLAMAlgorithmType, AlignerType_>;
    using BaseType             = LoopDetector_<SLAMAlgorithmType_>;

    PARAM(PropertyConfigurable_<AlignerType>,
          relocalize_aligner,
          "aligner used to register loop closures",
          0,
          nullptr);
    PARAM(PropertyUnsignedInt,
          relocalize_min_inliers,
          "minimum number of inliers for success [int]",
          500,
          nullptr);
    PARAM(PropertyFloat,
          relocalize_max_chi_inliers,
          "maximum chi per inlier for success [chi]",
          0.005,
          nullptr);
    PARAM(PropertyFloat,
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
