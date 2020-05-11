#include "local_map.h"

#include <srrg_geometry/geometry3d.h>
#include <srrg_pcl/point_types_data.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  void DynamicPropertyContainerOwner::draw(ViewerCanvasPtr canvas_) const {
    if (!canvas_) {
      return;
    }

    // ds TODO re-think this logic to support generic point clouds with less code UAGH
    for (auto it : dynamic_properties.properties()) {
      // ia TODO laser slam case :)
      Property_<PointNormal2fVectorCloud*>* points_2d =
        dynamic_cast<Property_<PointNormal2fVectorCloud*>*>(it.second);
      if (points_2d && points_2d->value()) {
        canvas_->putPoints(*points_2d->value());
        continue; // ds this property has been handled
      }

      // ia TODO proslam case :)
      Property_<PointIntensityDescriptor3fVectorCloud*>* points_3d =
        dynamic_cast<Property_<PointIntensityDescriptor3fVectorCloud*>*>(it.second);
      if (points_3d && points_3d->value()) {
        canvas_->putPoints(*points_3d->value());
        continue; // ds this property has been handled
      }

      // ia TODO shaslam case :)
      Property_<VisualMatchablefVector*>* matchables =
        dynamic_cast<Property_<VisualMatchablefVector*>*>(it.second);
      if (matchables && matchables->value()) {
        canvas_->putVisualMatchables(*matchables->value());
        continue; // ia this property has been handled
      }
    }
  }

  size_t DynamicPropertyContainerOwner::numberOfPoints() const {
    size_t number_of_points = 0;
    // ds TODO re-think this logic to support generic point clouds with less code UAGH
    for (auto it : dynamic_properties.properties()) {
      Property_<PointNormal2fVectorCloud*>* points_2d =
        dynamic_cast<Property_<PointNormal2fVectorCloud*>*>(it.second);
      if (points_2d && points_2d->value()) {
        number_of_points += points_2d->value()->size();
        continue; // ds this property has been handled
      }
      Property_<PointIntensityDescriptor3fVectorCloud*>* points_3d =
        dynamic_cast<Property_<PointIntensityDescriptor3fVectorCloud*>*>(it.second);
      if (points_3d && points_3d->value()) {
        number_of_points += points_3d->value()->size();
        continue; // ds this property has been handled
      }
    }
    return number_of_points;
  }

  template <typename VariableType_>
  void LocalMap_<VariableType_>::draw(srrg2_core::ViewerCanvasPtr canvas) const {
    if (!canvas) {
      return;
    }
    canvas->pushMatrix();
    canvas->pushColor();

    switch (map_status) {
      case Current:
        canvas->setColor(Eigen::Vector3f(0.2, 0.2, 0.8));
        break;
      case LoopChecked:
        canvas->setColor(Eigen::Vector3f(0.8, 0.2, 0.2));
        break;
      default:
        canvas->setColor(Eigen::Vector3f(0.2, 0.2, 0.2));
    }

    setDrawingReferenceFrame(canvas, ThisType::estimate());
    DynamicPropertyContainerOwner::draw(canvas);
    Eigen::Matrix4f m; // ds TODO make configurable in visualization parameters
    m << 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
    //    canvas->multMatrix(m);
    canvas->putSphere(size_local_map_sphere);
    canvas->popAttribute();
    canvas->popMatrix();
  }

  void LocalMap2D::setDrawingReferenceFrame(ViewerCanvasPtr canvas,
                                            const EstimateType& pose_) const {
    canvas->multMatrix(geometry3d::get3dFrom2dPose(pose_).matrix());
  }

  void LocalMap3D::setDrawingReferenceFrame(ViewerCanvasPtr canvas,
                                            const EstimateType& pose_) const {
    canvas->multMatrix(pose_.matrix());
  }

} // namespace srrg2_slam_interfaces
