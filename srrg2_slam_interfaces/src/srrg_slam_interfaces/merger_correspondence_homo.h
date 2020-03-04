#pragma once
#include <srrg_config/property_configurable.h>
#include <srrg_pcl/point_types.h>

#include "merger.h"

namespace srrg2_slam_interfaces {

  //! merger for identical point clouds of type SceneType_
  template <typename TransformType_, typename SceneType_>
  class MergerCorrespondenceHomo_
    : public MergerCorrespondence_<TransformType_, SceneType_, SceneType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using TransformType              = TransformType_;
    using PointCloudType             = SceneType_;
    using ThisType                   = MergerCorrespondenceHomo_<TransformType, PointCloudType>;
    using PointType                  = typename PointCloudType::value_type;
    using VectorType                 = typename PointType::VectorType;
    static constexpr size_t DepthDim = PointType::Dim - 1;

    PARAM(PropertyFloat,
          maximum_response,
          "maximum permitted correspondence response for merging a point",
          50,
          nullptr);
    PARAM(PropertyFloat,
          maximum_distance_geometry_squared,
          "maximum distance in geometry (i.e. 3D point distance norm) in meters (squared)",
          0.25,
          nullptr);

    void compute() override;
  };

  // ds TODO needs less generic naming since there will be more point types
  using MergerCorrespondence2D = MergerCorrespondenceHomo_<Isometry2f, PointNormal2fVectorCloud>;
  using MergerCorrespondence3D =
    MergerCorrespondenceHomo_<Isometry3f, PointIntensityDescriptor3fVectorCloud>;

  using MergerCorrespondence2DPtr = std::shared_ptr<MergerCorrespondence2D>;
  using MergerCorrespondence3DPtr = std::shared_ptr<MergerCorrespondence3D>;

} // namespace srrg2_slam_interfaces
