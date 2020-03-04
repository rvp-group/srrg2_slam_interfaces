#pragma once
#include "measurement_adaptor.h"
#include "merger.h"
#include "merger_correspondence_homo.h"
#include "multi_aligner_slice.h"
#include "scene_clipper.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename T>
  class MultiTrackerBase_;

  // abstract class that handles a cue for the multi slice tracker
  template <typename EstimateType_>
  class MultiTrackerSliceBase_ : public srrg2_core::Configurable,
                                 public srrg2_core::PlatformUser,
                                 public srrg2_core::DrawableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = EstimateType_;

    template <typename T>
    friend class MultiTrackerBase_;

    PARAM(PropertyString, scene_slice_name, "name of the slice in the fixed scene", "", nullptr);

    PARAM(PropertyString,
          measurement_slice_name,
          "name of the slice in the moving scene",
          "",
          nullptr);
    PARAM(PropertyString, frame_id, "name of the sensor frame in the tf tree", "", nullptr);
    PARAM(PropertyString, base_frame_id, "name of the base frame in the tf tree", "", nullptr);

  protected:
    virtual void setEstimate(const EstimateType& est_) {
      if (this->param_frame_id.value().length() && this->param_base_frame_id.value().length() &&
          this->_platform) {
        EstimateType sensor_in_robot = EstimateType::Identity();
        if (!this->_platform->getTransform(
              sensor_in_robot, this->param_frame_id.value(), this->param_base_frame_id.value())) {
          throw std::runtime_error("unable to get the transformation");
        }
        setSensorInRobot(sensor_in_robot);
      }

      _tracker_estimate = est_;
    }

    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _sensor_in_robot = sensor_in_robot_;
      _robot_in_sensor = _sensor_in_robot.inverse();
    }

    //! @brief set current sensor data pack (will be processed here)
    virtual void setMeasurement(BaseSensorMessagePtr meas) = 0;

    //! @brief basic tracking functions - process current sensor data
    virtual void adapt() = 0;

    //! @brief clips the map around current robot pose, to perform registration
    virtual void clip() = 0;

    //! @brief basic tracking functions - merges new guys in the current map, if any
    virtual void merge() = 0;

    //! @brief the scene is a dynamic container that contains everything (populated here)
    virtual void setScene(PropertyContainerBase* scene_) = 0;

    //! @brief checks whether the current scene is empty
    virtual bool isSceneSliceEmpty() const = 0;

    virtual void addMeasurementProperty()                        = 0;
    virtual void addSceneProperty(PropertyContainerBase* scene_) = 0;

    virtual void populateScene(PropertyContainerDynamic& container) = 0;

    //! prepares slice for closure correspondence based merge TODO purge it hopefully
    virtual void setClosure(const CorrespondenceVector& correspondences_,
                            const EstimateType& relative_transform_,
                            const EstimateType& tracker_estimate_) {
    }

    //! sets current global tracker estimate (used for landmark estimation)
    void setTrackerInWorld(const EstimateType& tracker_in_world_) {
      _tracker_in_world = tracker_in_world_;
      _world_in_tracker = _tracker_in_world.inverse();
    }

    inline void bindAdaptedMeasurements(PropertyContainerDynamic& adapted_measurements_) {
      _adapted_measurements = &adapted_measurements_;
    }
    inline void bindClippedScene(PropertyContainerDynamic& clipped_scene_) {
      _clipped_scene = &clipped_scene_;
    }

    EstimateType _sensor_in_robot                   = EstimateType::Identity();
    EstimateType _robot_in_sensor                   = EstimateType::Identity();
    EstimateType _tracker_estimate                  = EstimateType::Identity();
    EstimateType _tracker_in_world                  = EstimateType::Identity();
    EstimateType _world_in_tracker                  = EstimateType::Identity();
    PropertyContainerDynamic* _adapted_measurements = nullptr;
    PropertyContainerDynamic* _clipped_scene        = nullptr;
  };

  //! @brief basic specialization of the tracker slice. standard tracking is performed
  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  class MultiTrackerSlice_ : public MultiTrackerSliceBase_<EstimateType_> {
  public:
    using EstimateType    = EstimateType_;
    using FixedType       = FixedMeasurementType_; // the measurements remapped
    using MovingSceneType = MovingSceneType_;      // the scene
    using BaseType        = MultiTrackerSliceBase_<EstimateType>;
    using ThisType        = MultiTrackerSlice_<EstimateType, FixedType, MovingSceneType>;

    using MeasurementAdaptorType = MeasurementAdaptor_<FixedType>;
    using MergerType             = Merger_<EstimateType, MovingSceneType, FixedType>;
    using MergerPtrType          = std::shared_ptr<MergerType>;
    using ClipperType            = SceneClipper_<EstimateType, MovingSceneType>;
    using ClipperPtrType         = std::shared_ptr<ClipperType>;

    // ds correspondence based merger (dynamically checked to provide it with correspondences)
    using MergerCorrespondenceType =
      MergerCorrespondence_<EstimateType, MovingSceneType, FixedType>;
    using MergerCorrespondencePtrType = std::shared_ptr<MergerCorrespondenceType>;

    // ds loop closure merging types TODO bawh
    using MergerClosureType    = MergerCorrespondenceHomo_<EstimateType, MovingSceneType>;
    using MergerClosurePtrType = std::shared_ptr<MergerClosureType>;

    template <typename T>
    friend class MultiTrackerBase_;

    PARAM(PropertyConfigurable_<MeasurementAdaptorType>,
          adaptor,
          "measurement adaptor used in the slice",
          nullptr,
          nullptr);

    PARAM(PropertyConfigurable_<MergerType>,
          merger,
          "merger used for aligment of a measurement to a local map in the slice",
          nullptr,
          nullptr);

    PARAM(PropertyConfigurable_<ClipperType>,
          clipper,
          "clipper used in the slice",
          nullptr,
          nullptr);

    PARAM(PropertyConfigurable_<MergerClosureType>,
          closure_merger,
          "merger used for aligment of local maps in the slice",
          nullptr,
          nullptr);

    void sanityCheck(const std::string& message = "MultiTrackerSlice_") const;
    MultiTrackerSlice_() {
    }

  protected:
    void merge() override;
    void adapt() override;
    void clip() override;
    bool isSceneSliceEmpty() const override;
    void setScene(PropertyContainerBase* scene_) override;
    void setMeasurement(BaseSensorMessagePtr meas) override;
    void populateScene(PropertyContainerDynamic& container) override;
    void addSceneProperty(PropertyContainerBase* scene_) override;
    void setClosure(const CorrespondenceVector& correspondences_,
                    const EstimateType& relative_transform_,
                    const EstimateType& tracker_estimate_) override;
    void addMeasurementProperty();

    // this is where the adaptors write the converted measurements
    FixedType _adapted_slice;

    // this is where the new scene gets mapped (extracting fields from local map)
    MovingSceneType* _scene_slice = nullptr;
    MovingSceneType _static_scene_slice;

    // this is where the local map gets clipped during the alignment
    MovingSceneType _clipped_scene_slice;

    //! temporary correspondence buffer - from registration or closure
    CorrespondenceVector _correspondences;
    bool _have_loop_closure_correspondences = false;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // srrg debugging
    MovingSceneType* scene() {
      return _scene_slice;
    }

    MovingSceneType* clippedScene() {
      return &_clipped_scene_slice;
    }

    FixedType* measurements() {
      return &_adapted_slice;
    }
  };
} // namespace srrg2_slam_interfaces
