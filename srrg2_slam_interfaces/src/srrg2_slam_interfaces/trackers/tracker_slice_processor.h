#pragma once
#include "srrg2_slam_interfaces/mapping/merger_correspondence_homo.h"
#include "srrg2_slam_interfaces/raw_data_preprocessors/raw_data_preprocessor.h"
#include "tracker_slice_processor_base.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename EsimateType_, typename MovingSceneType_>
  class SceneClipper_;

  template <typename EsimateType_, typename MovingSceneType_, typename FixedMeasurementType_>
  class MergerCorrespondence_;

  //! @brief basic specialization of the tracker slice. standard tracking is performed
  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  class TrackerSliceProcessor_ : public TrackerSliceProcessorStandard_<EstimateType_,
                                                                       FixedMeasurementType_,
                                                                       MovingSceneType_> {
  public:
    using EstimateType         = EstimateType_;
    using FixedMeasurementType = FixedMeasurementType_; // the measurements remapped
    using MovingSceneType      = MovingSceneType_;      // the scene
    using BaseType =
      TrackerSliceProcessorStandard_<EstimateType, FixedMeasurementType, MovingSceneType>;
    using ThisType = TrackerSliceProcessor_<EstimateType, FixedMeasurementType, MovingSceneType>;

    using RawDataPreprocessorType = RawDataPreprocessor_<FixedMeasurementType>;
    using MergerType              = Merger_<EstimateType, MovingSceneType, FixedMeasurementType>;
    using MergerTypePtr           = std::shared_ptr<MergerType>;
    using ClipperType             = SceneClipper_<EstimateType, MovingSceneType>;
    using ClipperTypePtr          = std::shared_ptr<ClipperType>;

    // ds correspondence based merger (dynamically checked to provide it with correspondences)
    using MergerCorrespondenceType =
      MergerCorrespondence_<EstimateType, MovingSceneType, FixedMeasurementType>;
    using MergerCorrespondenceTypePtr = std::shared_ptr<MergerCorrespondenceType>;

    // ds loop closure merging types TODO bawh
    using MergerClosureType    = MergerCorrespondenceHomo_<EstimateType, MovingSceneType>;
    using MergerClosureTypePtr = std::shared_ptr<MergerClosureType>;

    template <typename T>
    friend class MultiTrackerBase_;

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
    TrackerSliceProcessor_() {
    }

  protected:
    void merge() override;
    void clip() override;
    bool isSceneSliceEmpty() const override {
      sanityCheck("TrackerSliceProcessor_::isSceneEmpty()");
      return ThisType::_scene_slice->empty() ||
             (ThisType::param_adaptor->status() != RawDataPreprocessorType::Ready);
    }
    void setClosure(const CorrespondenceVector& correspondences_,
                    const EstimateType& fixed_in_moving_,
                    const EstimateType& robot_in_moving_local_map_) override;

    void enhanceSceneProperty(Property_<MovingSceneType*>* p_) override;

    CorrespondenceVector
      _correspondences; /**< temporary correspondence buffer - from registration or closure*/
    bool _have_loop_closure_correspondences =
      false; /**< loop closure correspondences control flag*/

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // srrg debugging
    MovingSceneType* scene() {
      return ThisType::_scene_slice;
    }

    MovingSceneType* clippedScene() {
      return &(ThisType::_clipped_scene_slice);
    }

    FixedMeasurementType* measurements() {
      return &(ThisType::_measurement_slice);
    }
  };

} // namespace srrg2_slam_interfaces
