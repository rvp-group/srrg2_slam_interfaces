#pragma once
#include "srrg2_slam_interfaces/registration/aligners/aligner.h"
#include "tracker.h"
#include "tracker_slice_processor_base.h"
#include <srrg_config/property_configurable_vector.h>
#include <srrg_solver/solver_core/solver_stats.h>
#include <srrg_system_utils/profiler.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief multi-cue base tracker class
   * as the parent class, but works with the different data slices
   */
  template <typename EstimateType_>
  class MultiTrackerBase_ : public Tracker_<EstimateType_, srrg2_core::PropertyContainerBase>,
                            public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType              = EstimateType_;
    using BaseType                  = Tracker_<EstimateType, srrg2_core::PropertyContainerBase>;
    using TrackerSliceProcessorType = TrackerSliceProcessorBase_<EstimateType>;
    using SliceProcessorTypePtr     = std::shared_ptr<TrackerSliceProcessorType>;

    using AlignerType =
      Aligner_<EstimateType, srrg2_core::PropertyContainerBase, srrg2_core::PropertyContainerBase>;
    using AlignerTypePtr           = std::shared_ptr<AlignerType>;
    using MeasurementContainerType = typename BaseType::MeasurementContainerType;
    using SceneContainerType       = srrg2_core::PropertyContainerBase;
    using SceneContainerTypePtr    = std::shared_ptr<SceneContainerType>;

    PARAM_VECTOR(srrg2_core::PropertyConfigurableVector_<TrackerSliceProcessorType>,
                 slice_processors,
                 "slices",
                 &_slice_processors_changed_flag);

    PARAM(srrg2_core::PropertyConfigurable_<AlignerType>,
          aligner,
          "computes relative transform between fixed and moving slices",
          nullptr,
          nullptr);

    virtual ~MultiTrackerBase_() {
    }

    /**
     * @brief provide the current message to the tracker
     * propagates the fresh sensor message to all tr-slices
     * @param[in] msg_: message or message pack for the tracker
     */
    void setRawData(srrg2_core::BaseSensorMessagePtr message_) override;

    /**
     * @brief the scene is a dynamic container used to perform tracking. each slice has its
     * dedicated "space" in the scene, accessible through the Property substratum (requires to know
     * the slice name)
     * @param[in] scene: container of slices that holds the scene for each slice (local map)
     */
    void setScene(SceneContainerType* scene_);

    /**
     * @brief create measurements from raw data using raw data preprocessors
     * it recursively calls the sensor adaptation of all tr-slices
     */
    void preprocessRawData() override;

    /**
     * @brief recursively calls all the tr-slices to populate their own slice with their freshly
     * computed data
     * @param[out] container: dynamic container
     */
    void populateScene(srrg2_core::PropertyContainerDynamic& container_);

    /**
     * @brief registration phase - call recursively all the al-slices performing registration
     */
    void align() override;

    /**
     *@brief merging phase - map management, populating and fusing new entries from fresh sensor
     *       data. calls recursively the tr-slices
     */
    void merge() override;

    /**
     * @brief set the robot pose wrt the current map (or local map) in which it moves
     * and update it in every slice
     * @param[in] robot_in_local_map_: robot pose wrt map origin
     */
    void setRobotInLocalMap(const EstimateType& robot_in_local_map_) override;

    const EstimateType& robotInLocalMap() const override {
      return _robot_in_local_map;
    }

    /**
     * @brief sets the relative transform and correspondences for relocalization TODO burn
     * @param[in] correspondences_: precomputed correspondences from fixed to moving local map
     * @param[in] fixed_in_moving_: relative transformation between the origins of the fixed map
     *                              (other map) wrt the moving map (current)
     * @param[in] robot_in_moving_local_map_: pose of the robot in the current local map
     */
    void setClosure(const srrg2_core::CorrespondenceVector& correspondences_,
                    const EstimateType& fixed_in_moving_,
                    const EstimateType& robot_in_moving_local_map_);

    /**
     * @brief gets statistics of the aligner
     * @return aligner's statistics iteration-wise
     */
    const srrg2_solver::IterationStatsVector& iterationStats() const {
      assert(param_aligner.value());
      return param_aligner->iterationStats();
    }

    MeasurementContainerType& measurementContainer() override {
      return _measurement_container;
    }

    /**
     * @brief external setter for aligner initial guess (can be overwritten by slices)
     * @param[in] moving_in_fixed_guess_: initial guess for the aligner
     */
    void setInitialGuessAligner(const EstimateType& moving_in_fixed_guess_) {
      _moving_in_fixed_guess = moving_in_fixed_guess_;
    }

    /**
     * @brief sets global pose of current local map (allows to recover tracker w.r.t world) global
     * pose information is required by the landmark estimator in merge phase
     * @param[in] local_map_in_world_: pose of the current local map in the global map
     */
    void setLocalMapInWorld(const EstimateType& local_map_in_world_) {
      _local_map_in_world = local_map_in_world_;
    }

    /**
     * @brief propagate platform to all internal modules
     * @param[in] platform_: robot platform
     */
    void setPlatform(srrg2_core::PlatformPtr platform_) override;

    /**
     * @brief visualize current pose-to-measurements correspondences in perspective recursively
     * calls al-slice draw AND tr-slice draw
     */
    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override;

  protected:
    /**
     * @brief updates the tracker estimate with the result of the aligner
     * @param[in] moving_in_fixed_: aligner estimate
     */
    virtual void _updateRobotInLocalMap(const EstimateType& moving_in_fixed_) {
      _robot_in_local_map = _robot_in_local_map * moving_in_fixed_.inverse();
      setRobotInLocalMap(_robot_in_local_map);
    }

    srrg2_core::PropertyContainerDynamic _clipped_scene_container; /**< cached clipped scene*/
    srrg2_core::PropertyContainerDynamic _measurement_container;   /**< measurement container*/

    EstimateType _robot_in_local_map =
      EstimateType::Identity(); /**< pose of the robot wrt the current local map*/
    EstimateType _moving_in_fixed_guess =
      EstimateType::Identity(); /**< inital guess for the aligner */
    EstimateType _local_map_in_world =
      EstimateType::Identity(); /**< pose of the current local map wrt the global one*/
    bool _slice_processors_changed_flag = true; /**< slice change control flag */
  };

  using MultiTracker2D    = MultiTrackerBase_<srrg2_core::Isometry2f>;
  using MultiTracker3D    = MultiTrackerBase_<srrg2_core::Isometry3f>;
  using MultiTracker2DPtr = std::shared_ptr<MultiTracker2D>;
  using MultiTracker3DPtr = std::shared_ptr<MultiTracker3D>;

} // namespace srrg2_slam_interfaces
