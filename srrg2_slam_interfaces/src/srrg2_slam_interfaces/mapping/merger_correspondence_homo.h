#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_types.h>

#include "merger.h"

namespace srrg2_slam_interfaces {

  //! merger for identical point clouds of type SceneType_
  template <typename EstimateType_, typename SceneType_>
  class MergerCorrespondenceHomo_
    : public MergerCorrespondence_<EstimateType_, SceneType_, SceneType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType               = EstimateType_;
    using PointCloudType             = SceneType_;
    using ThisType                   = MergerCorrespondenceHomo_<EstimateType, PointCloudType>;
    using PointType                  = typename PointCloudType::value_type;
    using VectorType                 = typename PointType::VectorType;
    static constexpr size_t DepthDim = PointType::Dim - 1;

    PARAM(srrg2_core::PropertyFloat,
          maximum_response,
          "maximum permitted correspondence response for merging a point",
          50,
          nullptr);
    PARAM(srrg2_core::PropertyFloat,
          maximum_distance_geometry_squared,
          "maximum distance in geometry (i.e. 3D point distance norm) in meters (squared)",
          0.25,
          nullptr);

    void compute() override;
  };

  using MergerCorrespondencePointNormal2f =
    MergerCorrespondenceHomo_<srrg2_core::Isometry2f, srrg2_core::PointNormal2fVectorCloud>;
  using MergerCorrespondencePointIntensityDescriptor3f =
    MergerCorrespondenceHomo_<srrg2_core::Isometry3f,
                              srrg2_core::PointIntensityDescriptor3fVectorCloud>;

  using MergerCorrespondencePointNormal2fPtr = std::shared_ptr<MergerCorrespondencePointNormal2f>;
  using MergerCorrespondencePointIntensityDescriptor3fPtr =
    std::shared_ptr<MergerCorrespondencePointIntensityDescriptor3f>;

} // namespace srrg2_slam_interfaces
