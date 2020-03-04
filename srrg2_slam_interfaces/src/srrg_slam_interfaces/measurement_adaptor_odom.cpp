#include "measurement_adaptor_odom.h"
#include <srrg_geometry/geometry3d.h>
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_messages/messages/odometry_message.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  void MeasurementAdaptorOdom2D::compute() {
    ThisType::_status = ThisType::Status::Error;
    if (!_saved_odom) {
      throw std::runtime_error("no odom!");
    }
    if (!_dest) {
      throw std::runtime_error("no dest!");
    }
    *_dest            = geometry3d::get2dFrom3dPose(_saved_odom->pose.value());
    ThisType::_status = ThisType::Status::Ready;
  }

  void MeasurementAdaptorOdom3D::compute() {
    ThisType::_status = ThisType::Status::Error;
    if (!_saved_odom) {
      throw std::runtime_error("no odom!");
    }
    if (!_dest) {
      throw std::runtime_error("no dest!");
    }
    *_dest            = _saved_odom->pose.value();
    ThisType::_status = ThisType::Status::Ready;
  }

} // namespace srrg2_slam_interfaces
