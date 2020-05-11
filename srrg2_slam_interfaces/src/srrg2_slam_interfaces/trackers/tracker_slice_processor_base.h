#pragma once
#include "srrg2_slam_interfaces/raw_data_preprocessors/raw_data_preprocessor.h"
#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/correspondence.h>
#include <srrg_data_structures/platform.h>
#include <srrg_messages/messages/base_sensor_message.h>
#include <srrg_viewer/drawable_base.h>

namespace srrg2_slam_interfaces {

  template <typename T>
  class MultiTrackerBase_;

  //! @brief abstract class that handles a cue for the multi slice tracker
  template <typename EstimateType_>
  class TrackerSliceProcessorBase_ : public srrg2_core::Configurable,
                                     public srrg2_core::PlatformUser,
                                     public srrg2_core::DrawableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType          = EstimateType_;
    using SceneContainerType    = srrg2_core::PropertyContainerBase;
    using SceneContainerTypePtr = std::shared_ptr<SceneContainerType>;

    template <typename T>
    friend class MultiTrackerBase_;

    PARAM(srrg2_core::PropertyString,
          scene_slice_name,
          "name of the slice in the fixed scene",
          "",
          nullptr);

    PARAM(srrg2_core::PropertyString,
          measurement_slice_name,
          "name of the slice in the moving scene",
          "",
          nullptr);
    PARAM(srrg2_core::PropertyString,
          frame_id,
          "name of the sensor frame in the tf tree",
          "",
          nullptr);
    PARAM(srrg2_core::PropertyString,
          base_frame_id,
          "name of the base frame in the tf tree",
          "",
          nullptr);

  protected:
    /**
     * @brief set the robot pose wrt the current map (or local map) in which it works
     * and update it in every slice
     * if a platform is set it gets also the pose of the sensor in the robot
     * @param[in] robot_in_local_map_: robot pose wrt map origin
     */
    virtual void setRobotInLocalMap(const EstimateType& robot_in_local_map_);

    /**
     * @brief cache sensor pose wrt the robot
     * @param[in] sensor_in_robot_: sensor in robot pose
     */
    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _sensor_in_robot = sensor_in_robot_;
      _robot_in_sensor = _sensor_in_robot.inverse();
    }

    /**
     * @brief set current sensor data pack (will be processed here)
     * @param[in] msg_: message or message pack for the tracker
     */
    virtual void setRawData(srrg2_core::BaseSensorMessagePtr msg_) = 0;

    /**
     * @brief basic tracking functions - process current sensor data
     */
    virtual void adapt() = 0;

    /**
     * @brief clips the map around current robot pose, to perform registration
     */
    virtual void clip() = 0;

    /**
     * @brief basic tracking functions - merges new guys in the current map, if any
     */
    virtual void merge() = 0;

    /**
     * @brief the scene is a dynamic container that contains everything (populated here)
     */
    virtual void setScene(SceneContainerType* scene_) = 0;

    /**
     * @brief checks whether the current scene is empty
     */
    virtual bool isSceneSliceEmpty() const = 0;
    /**
     * @brief gathers the casted scene slice and pushes the computed clipped scene in the
     * appropriate property container
     * @param[in/out] scene: scene property container
     */
    virtual void addSceneProperty(SceneContainerType* scene_) = 0;
    /**
     * @brief add a new slice for a particular tracker slice preprocessor to the scene container
     * @
     */
    virtual void populateScene(srrg2_core::PropertyContainerDynamic& scene_container_) = 0;

    virtual void addMeasurementProperty() = 0;

    /**
     * @brief sets the relative transform and correspondences for relocalization TODO burn
     * @param[in] correspondences_: precomputed correspondences from fixed to moving local map
     * @param[in] fixed_in_moving_: relative transformation between the origins of the fixed map
     *                              (other map) wrt the moving map (current)
     * @param[in] robot_in_moving_local_map_: pose of the robot in the current local map
     */
    virtual void setClosure(const srrg2_core::CorrespondenceVector& correspondences_,
                            const EstimateType& fixed_in_moving_,
                            const EstimateType& robot_in_moving_local_map_) {
    }

    /**
     * @brief set the global robot pose (for landmark estimation)
     * @param[in] robot_in_world_: pose of the robot wrt the global map
     */
    void setRobotInWorld(const EstimateType& robot_in_world_) {
      _robot_in_world = robot_in_world_;
      _world_in_robot = _robot_in_world.inverse();
    }

    /**
     * @brief bind the post processed raw data (aka measurement) to a slice
     * @param[in] measurement scene
     */
    inline void bindMeasurements(srrg2_core::PropertyContainerDynamic& measurements_) {
      _measurements_container = &measurements_;
    }
    /**
     * @brief bind clipped scene slice for later reuse
     * @param[in] clipped scene
     */
    inline void bindClippedScene(srrg2_core::PropertyContainerDynamic& clipped_scene_container_) {
      _clipped_scene_container = &clipped_scene_container_;
    }

    EstimateType _sensor_in_robot = EstimateType::Identity(); /**< cached sensor pose wrt robot*/
    EstimateType _robot_in_sensor = EstimateType::Identity(); /**< cached robot pose wrt sensor */
    EstimateType _robot_in_local_map =
      EstimateType::Identity(); /**< cached robot pose wrt the current local map*/
    EstimateType _robot_in_world =
      EstimateType::Identity(); /**< cached robot pose in global coordinates*/
    EstimateType _world_in_robot =
      EstimateType::Identity(); /**< cached pose of the origin frame in robot frame*/

    srrg2_core::PropertyContainerDynamic* _measurements_container =
      nullptr; /**< pointer to the measurement container */
    srrg2_core::PropertyContainerDynamic* _clipped_scene_container =
      nullptr; /**< pointer to the clipped scene container */
  };

  /**
   * @brief abstarct class of Tracker slice processor specialized with the measurement and scene
   */
  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  class TrackerSliceProcessorStandard_ : public TrackerSliceProcessorBase_<EstimateType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using EstimateType         = EstimateType_;
    using FixedMeasurementType = FixedMeasurementType_; // the measurements remapped
    using MovingSceneType      = MovingSceneType_;      // the scene
    using BaseType             = TrackerSliceProcessorBase_<EstimateType>;
    using ThisType =
      TrackerSliceProcessorStandard_<EstimateType, FixedMeasurementType, MovingSceneType>;
    using RawDataPreprocessorType = RawDataPreprocessor_<FixedMeasurementType>;

    template <typename T>
    friend class MultiTrackerBase_;

    // ds this slice is also (ab)used for pumping in poses for the motion model aligner slice
    PARAM(srrg2_core::PropertyConfigurable_<RawDataPreprocessorType>,
          adaptor,
          "measurement adaptor used in the slice",
          nullptr,
          nullptr);

  protected:
    /**
     * @brief performs basic checks for data integrity
     * @param[in] message: name of the class
     */
    virtual void sanityCheck(const std::string& message = "MultiTrackerSlicePrior_") const;
    void adapt() override;
    void setRawData(srrg2_core::BaseSensorMessagePtr msg_) override;
    void addMeasurementProperty() override;

    void setScene(srrg2_core::PropertyContainerBase* scene_) override;
    void populateScene(srrg2_core::PropertyContainerDynamic& container) override;
    void addSceneProperty(srrg2_core::PropertyContainerBase* scene_) override;
    virtual void enhanceSceneProperty(srrg2_core::Property_<MovingSceneType*>* p_) = 0;

    inline bool isSceneSliceEmpty() const override {
      sanityCheck("TrackerSliceProcessorPrior_::isSceneEmpty()");
      return (param_adaptor->status() == RawDataPreprocessorType::Ready);
    }

    FixedMeasurementType _measurement_slice; /**< adaptor writes here the converted raw data*/

    MovingSceneType* _scene_slice = nullptr; /**< pointer to the whole scene property container*/
    MovingSceneType _static_scene_slice;     /**< property added to the scene */
    MovingSceneType _clipped_scene_slice;    /**< computed clipped scene from the local map*/
  };
} // namespace srrg2_slam_interfaces
