#pragma once
#include "measurement_adaptor.h"

namespace srrg2_slam_interfaces {

  // ds buffered (i.e. has short-term memory) tracker estimate adaptor
  // ds the DestContainerType_ must support pop_front and push_back
  template <typename DestContainerType_>
  class MeasurementAdaptorTrackerEstimate_ : public MeasurementAdaptor_<DestContainerType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType      = MeasurementAdaptorTrackerEstimate_<DestContainerType_>;
    using ContainerType = DestContainerType_;
    using EstimateType  = typename DestContainerType_::value_type;

    PARAM(srrg2_core::PropertyUnsignedInt,
          number_of_poses_to_keep,
          "maximum size of the pose buffer",
          5,
          nullptr);

    MeasurementAdaptorTrackerEstimate_() {
      // ds ready by construction
      ThisType::_status = ThisType::Ready;
      _tracker_pose_estimates.clear();

      // ds add the initial estimate (the adaptor will be updated in merge phase with a new pose)
      // ds hence we have a "lagged" adaptor with memory
      _tracker_pose_estimate_last = EstimateType::Identity();
    }

    //! updates the pose buffer
    void compute() override {
      assert(ThisType::_dest);

      // ds if the buffer is saturated
      if (_tracker_pose_estimates.size() == param_number_of_poses_to_keep.value()) {
        // ds roll ahead
        _tracker_pose_estimates.pop_front();
        _tracker_pose_estimates.push_back(_tracker_pose_estimate_last);
        assert(_tracker_pose_estimates.size() == param_number_of_poses_to_keep.value());
      } else {
        // ds grow buffer
        _tracker_pose_estimates.push_back(_tracker_pose_estimate_last);
      }
      *ThisType::_dest = _tracker_pose_estimates;
    }

    //! dummy
    bool setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) override {
      // ds unused
      return true;
    }

    //! as we are not processing an actual measurement, we obtain directly the compute result
    //! @param[in] tracker_pose_estimate_ current tracker pose estimate w.r.t local map origin
    void setEstimate(const EstimateType& tracker_pose_estimate_) {
      _tracker_pose_estimate_last = tracker_pose_estimate_;
    }

    //! recenter the current pose buffer behind the provided transform without addition
    //! @param[in] tracker_pose_estimate_ current tracker pose estimate w.r.t local map origin
    void setCoordinateFrameOrigin(const EstimateType& origin_) {
      const EstimateType transform_into_new_origin(origin_.inverse());

      // ds shift all past estimates behind the current one
      // ds this preserves buffer consistency between local map changes (new/relocalization)
      for (EstimateType& tracker_pose_estimate : _tracker_pose_estimates) {
        tracker_pose_estimate = transform_into_new_origin * tracker_pose_estimate;
      }
    }

  protected:
    //! last set estimate (effectively stored after compute)
    EstimateType _tracker_pose_estimate_last;

    //! fixed size buffer of past tracker pose estimates
    ContainerType _tracker_pose_estimates;
  };

  using MeasurementAdaptorTrackerEstimate2D =
    MeasurementAdaptorTrackerEstimate_<srrg2_core::StdDequeEigenIsometry2f>;
  using MeasurementAdaptorTrackerEstimate3D =
    MeasurementAdaptorTrackerEstimate_<srrg2_core::StdDequeEigenIsometry3f>;
  using MeasurementAdaptorTrackerEstimate2DPtr =
    std::shared_ptr<MeasurementAdaptorTrackerEstimate2D>;
  using MeasurementAdaptorTrackerEstimate3DPtr =
    std::shared_ptr<MeasurementAdaptorTrackerEstimate3D>;

} // namespace srrg2_slam_interfaces
