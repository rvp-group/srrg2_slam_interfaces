#include "srrg2_slam_interfaces/raw_data_preprocessors/raw_data_preprocessor_odom.h"
#include "tracker_slice_processor_prior.h"
namespace srrg2_slam_interfaces {
  class TrackerSliceProcessorPriorOdom2D
    : public TrackerSliceProcessorPrior_<Isometry2f, Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = TrackerSliceProcessorPrior_<Isometry2f, Isometry2f>;
    TrackerSliceProcessorPriorOdom2D() {
      BaseType::param_adaptor.setValue(RawDataPreprocessorOdom2DPtr(new RawDataPreprocessorOdom2D));
    }
  };

  using TrackerSliceProcessorPriorOdom2DPtr = std::shared_ptr<TrackerSliceProcessorPriorOdom2D>;

  class TrackerSliceProcessorPriorOdom3D
    : public TrackerSliceProcessorPrior_<Isometry3f, Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = TrackerSliceProcessorPrior_<Isometry3f, Isometry3f>;

    TrackerSliceProcessorPriorOdom3D() {
      BaseType::param_adaptor.setValue(RawDataPreprocessorOdom3DPtr(new RawDataPreprocessorOdom3D));
    }
  };

  using TrackerSliceProcessorPriorOdom3DPtr = std::shared_ptr<TrackerSliceProcessorPriorOdom3D>;

} // namespace srrg2_slam_interfaces
