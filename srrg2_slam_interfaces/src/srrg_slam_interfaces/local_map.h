#pragma once
#include <srrg_property/property_container.h>
#include <srrg_solver/variables_and_factors/types_2d/variable_se2_ad.h>
#include <srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h>
#include <srrg_viewer/viewer_canvas.h>

namespace srrg2_slam_interfaces {

  enum LocalMapStatus { Current = 0x0, Idle = 0x1, LoopChecked = 0x2 };

  class DynamicPropertyContainerOwner : public srrg2_core::DrawableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    srrg2_core::PropertyContainerDynamic dynamic_properties;
    void draw(srrg2_core::ViewerCanvasPtr canvas) const override;

    //! wrap
    const srrg2_core::StringPropertyPtrMap& properties() const {
      return dynamic_properties.properties();
    }
    srrg2_core::PropertyBase* property(const std::string& name_) {
      return dynamic_properties.property(name_);
    }

    // ds ugly, expensive way of determining the current number of points in the local map
    // ds TODO we should be able to make this more efficient through interfaced property addition
    size_t numberOfPoints() const;
  };

  template <typename VariableType_>
  class LocalMap_ : public VariableType_, public DynamicPropertyContainerOwner {
  public:
    using ThisType     = LocalMap_<VariableType_>;
    using VariableType = VariableType_;
    using EstimateType = typename VariableType::EstimateType;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // created when SLAM makes a new local map, populated the fields
    LocalMapStatus map_status = Idle;

    virtual void instantiate() {
    } // will be blasted
    virtual ~LocalMap_() {
    }
    void draw(srrg2_core::ViewerCanvasPtr canvas) const override;

    virtual void setDrawingReferenceFrame(srrg2_core::ViewerCanvasPtr canvas,
                                          const EstimateType& pose_) const {};

    // ds visualization properties TODO parametrize visualization?
    float size_local_map_shere = 0.2;
  };

  class LocalMap2D : public LocalMap_<srrg2_solver::VariableSE2RightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = typename srrg2_solver::VariableSE2RightAD::EstimateType;
    LocalMap2D() {
      size_local_map_shere = 0.2; // ds default for laser
    }
    void setDrawingReferenceFrame(srrg2_core::ViewerCanvasPtr canvas,
                                  const EstimateType& pose_) const override;
  };

  class LocalMap3D : public LocalMap_<srrg2_solver::VariableSE3QuaternionRightAD> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = typename srrg2_solver::VariableSE3QuaternionRightAD::EstimateType;
    LocalMap3D() {
      size_local_map_shere = 0.05; // ds indoor scenarios TODO meh
    }
    void setDrawingReferenceFrame(srrg2_core::ViewerCanvasPtr canvas,
                                  const EstimateType& pose_) const override;
  };

} // namespace srrg2_slam_interfaces
