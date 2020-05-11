#pragma once
#include "raw_data_preprocessor.h"

namespace srrg2_slam_interfaces {

  // ds buffered (i.e. has short-term memory) tracker estimate adaptor
  // ds the DestContainerType_ must support pop_front and push_back
  template <typename DestContainerType_>
  class RawDataPreprocessorTrackerEstimate_ : public RawDataPreprocessor_<DestContainerType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using ThisType      = RawDataPreprocessorTrackerEstimate_<DestContainerType_>;
    using ContainerType = DestContainerType_;
    using EstimateType  = typename DestContainerType_::value_type;

    PARAM(srrg2_core::PropertyUnsignedInt,
          number_of_poses_to_keep,
          "maximum size of the pose buffer",
          5,
          nullptr);

    RawDataPreprocessorTrackerEstimate_() {
      // ds ready by construction
      ThisType::_status = ThisType::Ready;
      _robot_in_local_map_estimates.clear();
      //      ThisType::_meas.reset(new ContainerType);
    }

    //! updates the pose buffer
    void compute() override {
      assert(ThisType::_meas);

      // ds if the buffer is saturated
      if (_robot_in_local_map_estimates.size() == param_number_of_poses_to_keep.value()) {
        // ds roll ahead
        _robot_in_local_map_estimates.pop_front();
        _robot_in_local_map_estimates.push_back(_robot_in_local_map);
        assert(_robot_in_local_map_estimates.size() == param_number_of_poses_to_keep.value());
      } else {
        // ds grow buffer
        _robot_in_local_map_estimates.push_back(_robot_in_local_map);
      }
      *ThisType::_meas = _robot_in_local_map_estimates;
    }

    //! dummy
    bool setRawData(srrg2_core::BaseSensorMessagePtr message_) override {
      // ds unused
      return true;
    }

    //! as we are not processing an actual measurement, we obtain directly the compute result
    //! @param[in] tracker_pose_estimate_ current tracker pose estimate w.r.t local map origin
    void setRobotInLocalMap(const EstimateType& robot_in_local_map_) {
      _robot_in_local_map = robot_in_local_map_;
    }

    //! recenter the current pose buffer behind the provided transform without addition
    //! @param[in] tracker_pose_estimate_ current tracker pose estimate w.r.t local map origin
    void setCoordinateFrameOrigin(const EstimateType& origin_) {
      const EstimateType transform_into_new_origin(origin_.inverse());

      // ds shift all past estimates behind the current one
      // ds this preserves buffer consistency between local map changes (new/relocalization)
      for (EstimateType& tracker_pose_estimate : _robot_in_local_map_estimates) {
        tracker_pose_estimate = transform_into_new_origin * tracker_pose_estimate;
      }
    }

  protected:
    //! last set estimate (effectively stored after compute)
    EstimateType _robot_in_local_map = EstimateType::Identity();

    //! fixed size buffer of past tracker pose estimates
    ContainerType _robot_in_local_map_estimates;
  };

  using RawDataPreprocessorTrackerEstimate2D =
    RawDataPreprocessorTrackerEstimate_<srrg2_core::StdDequeEigenIsometry2f>;
  using RawDataPreprocessorTrackerEstimate3D =
    RawDataPreprocessorTrackerEstimate_<srrg2_core::StdDequeEigenIsometry3f>;
  using RawDataPreprocessorTrackerEstimate2DPtr =
    std::shared_ptr<RawDataPreprocessorTrackerEstimate2D>;
  using RawDataPreprocessorTrackerEstimate3DPtr =
    std::shared_ptr<RawDataPreprocessorTrackerEstimate3D>;

} // namespace srrg2_slam_interfaces
