#pragma once
#include <srrg_config/configurable.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/messages/base_sensor_message.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  //! this class clips the scene around a desired location
  template <typename TransformType_, typename SceneType_>
  class SceneClipper_ : public Configurable, public PlatformUser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    enum Status { Ready, Error };
    using ThisType      = SceneClipper_<TransformType_, SceneType_>;
    using BaseType      = Configurable;
    using TransformType = TransformType_;
    using SceneType     = SceneType_;

    virtual void compute() = 0;

    virtual void reset() {
      _local_scene  = 0;
      _global_scene = 0;
      _transform.setIdentity();
      _status = Error;
    }

    // the scene that will be modified
    // the ar
    inline void setLocalScene(SceneType* scene_) {
      _local_scene = scene_;
    }

    inline void setGlobalScene(SceneType* scene_) {
      _global_scene = scene_;
    }

    //! set dynamic container for carrying auxiliary information (e.g. projections)
    inline void setAuxiliaryData(PropertyContainerDynamic* auxiliary_data_,
                                 const std::string& prefix_ = "") {
      _auxiliary_data        = auxiliary_data_;
      _prefix_auxiliary_data = prefix_;
    }

    inline void setTransform(const TransformType& transform_) {
      _transform = transform_;
    }

    inline void setSensorInRobot(const TransformType& sensor_in_robot_) {
      _sensor_in_robot = sensor_in_robot_;
      _robot_in_sensor = _sensor_in_robot.inverse();
    }

    inline Status status() const {
      return _status;
    }

    //! mapping from local (clipped scene) to global (scene) indices
    //! required for e.g. aligner-based correspondence-based merging
    virtual const std::vector<int> globalIndices() const {
      // ds implementation is not enforced
      return std::vector<int>(0);
    }

  protected:
    SceneType* _local_scene                   = nullptr;
    SceneType* _global_scene                  = nullptr;
    PropertyContainerDynamic* _auxiliary_data = nullptr;
    std::string _prefix_auxiliary_data        = "";
    TransformType _transform                  = TransformType::Identity();
    TransformType _sensor_in_robot            = TransformType::Identity();
    TransformType _robot_in_sensor            = TransformType::Identity();
    Status _status                            = Error;
  };

} // namespace srrg2_slam_interfaces
