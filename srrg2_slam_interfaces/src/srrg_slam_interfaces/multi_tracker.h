#pragma once
#include <srrg_config/property_configurable_vector.h>

#include "multi_tracker_slice.h"
#include "srrg_system_utils/profiler.h"
#include "tracker.h"

namespace srrg2_slam_interfaces {

  template <typename EstimateType_>
  class MultiTrackerBase_ : public Tracker_<EstimateType_, srrg2_core::PropertyContainerBase>,
                            public srrg2_core::Profiler {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType     = Tracker_<EstimateType_, srrg2_core::PropertyContainerBase>;
    using EstimateType = EstimateType_;
    using MultiTrackerSliceProcessorType = MultiTrackerSliceBase_<EstimateType>;
    using SliceProcessorPtrType          = std::shared_ptr<MultiTrackerSliceProcessorType>;

    using AlignerType =
      Aligner_<EstimateType, srrg2_core::PropertyContainerBase, srrg2_core::PropertyContainerBase>;
    using AlignerPtrType       = std::shared_ptr<AlignerType>;
    using SceneType            = srrg2_core::PropertyContainerBase;
    using MeasurementSceneType = typename BaseType::MeasurementSceneType;
    using ScenePtrType         = srrg2_core::PropertyContainerBasePtr;

    PARAM_VECTOR(PropertyConfigurableVector_<MultiTrackerSliceBase_<EstimateType>>,
                 slice_processors,
                 "slices",
                 &_slice_processors_changed_flag);

    PARAM(PropertyConfigurable_<AlignerType>,
          aligner,
          "computes relative transform between fixed and moving slices",
          nullptr,
          nullptr);

    virtual ~MultiTrackerBase_() {
    }

    //! @brief propagates the fresh sensor message to all tr-slices
    void setMeasurement(srrg2_core::BaseSensorMessagePtr measurement_) override;

    //! @brief the scene is a dynamic container used to perform tracking. each slice has its
    //!        dedicated "space" in the scene, accessible through the Property substratum (requires
    //!        to know the slice name)
    void setScene(SceneType* scene_);

    //! @brief given sensor message, it recursively calls the sensor adaptation of all tr-slices
    void adaptMeasurements() override;

    //! @brief recursively calls all the tr-slices to populate their own slice with their
    //!        freshly computed data
    void populateScene(srrg2_core::PropertyContainerDynamic& container);

    //! @brief registration phase - call recursively all the al-slices performing registration
    void align() override;

    //! @brief merging phase - map management, populating and fusing new entries from fresh sensor
    //!        data. calls recursively the tr-slices
    void merge() override;

    //! @brief overrides the current tracker estimate and update it in every slice
    void setEstimate(const EstimateType& estimate_) override;

    //! @brief retrieve the current tracker estimate
    const EstimateType& estimate() const override {
      return _tracker_estimate;
    }

    //! sets the relative transform and correspondences for relocalization TODO burn
    void setClosure(const srrg2_core::CorrespondenceVector& correspondences_,
                    const EstimateType& relative_transform_,
                    const EstimateType& tracker_estimate_relocalized_);

    const srrg2_solver::IterationStatsVector& iterationStats() const {
      assert(param_aligner.value());
      return param_aligner->iterationStats();
    }

    MeasurementSceneType& measurementScene() override {
      return _adapted_measurements;
    }

    //! external setter for aligner initial guess (can be overwritten by slices)
    void setInitialGuessAligner(const EstimateType& initial_guess_) {
      _initial_guess_aligner = initial_guess_;
    }

    //! @brief sets global pose of current local map (allows to recover tracker w.r.t world)
    //!        global pose information is required by the landmark estimator in merge phase
    void setSceneInWorld(const EstimateType& scene_in_world_) {
      _scene_in_world = scene_in_world_;
    }

    // ds propagate platform to all internal users as well
    void setPlatform(srrg2_core::PlatformPtr platform_) override;

    //! @brief visualize current pose-to-measurements correspondences in perspective
    //!        recursively calls al-slice draw AND tr-slice draw
    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override;

  protected:
    srrg2_core::PropertyContainerDynamic _clipped_scene;
    srrg2_core::PropertyContainerDynamic _adapted_measurements;

    EstimateType _tracker_estimate      = EstimateType::Identity();
    EstimateType _initial_guess_aligner = EstimateType::Identity();
    EstimateType _scene_in_world        = EstimateType::Identity();
    bool _slice_processors_changed_flag = true;

    //! @brief updates the tracker estimate with the result of the aligner
    virtual void _updateTrackerEstimate(const EstimateType& aligner_estimate_) {
      _tracker_estimate = _tracker_estimate * aligner_estimate_.inverse();
      setEstimate(_tracker_estimate);
    }
  };

  using MultiTracker2D    = MultiTrackerBase_<srrg2_core::Isometry2f>;
  using MultiTracker3D    = MultiTrackerBase_<srrg2_core::Isometry3f>;
  using MultiTracker2DPtr = std::shared_ptr<MultiTracker2D>;
  using MultiTracker3DPtr = std::shared_ptr<MultiTracker3D>;

} // namespace srrg2_slam_interfaces
