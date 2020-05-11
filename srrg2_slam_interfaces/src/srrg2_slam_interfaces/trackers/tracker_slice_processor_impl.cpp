#include "srrg2_slam_interfaces/mapping/scene_clipper.h"
#include "tracker_slice_processor.h"
#include "tracker_slice_processor_base_impl.cpp"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessor_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::sanityCheck(
    const std::string& message) const {
    BaseType::sanityCheck(message);
    std::string msg = message;
    if (!param_clipper.value()) {
      msg += "| no clipper";
      throw std::runtime_error(msg.c_str());
    }
    if (!param_merger.value()) {
      msg += "| no merger";
      throw std::runtime_error(msg.c_str());
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  inline void TrackerSliceProcessor_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::
    enhanceSceneProperty(Property_<MovingSceneType*>* p_) {
    // ds check if the merger supports correspondences (defined by its dynamic type)
    MergerCorrespondenceTypePtr merger_correspondence =
      std::dynamic_pointer_cast<MergerCorrespondenceType>(param_merger.value());

    // ds and whether correspondences are available or not (in the clipped scene)
    if (merger_correspondence && ThisType::_clipped_scene_container) {
      // ds grab correspondences used for fixed measurement alignment from scene object
      using PropertyCorrespondenceVector = Property_<CorrespondenceVector>;
      PropertyCorrespondenceVector* auxiliary_data =
        ThisType::_clipped_scene_container->template property<PropertyCorrespondenceVector>(
          ThisType::param_measurement_slice_name.value() +
          CorrespondenceFinderBase::auxiliary_data_suffix_correspondences);
      if (auxiliary_data) {
        // ds get correspondences from previous aligment
        const CorrespondenceVector& local_correspondences = auxiliary_data->value();
        MovingSceneType& moving_cloud                     = *p_->value();
        moving_cloud.reserve(local_correspondences.size());
        _correspondences.clear();
        _correspondences.reserve(local_correspondences.size());
        // srrg take the correspondence.moving_idx and put it in the current local map
        for (size_t i = 0; i < local_correspondences.size(); ++i) {
          const Correspondence& correspondence = local_correspondences[i];

          // ds populate current scene with corresponding point from previous scene
          // ds note that points from the previous scene without correspondences are not imported!
          assert(static_cast<size_t>(correspondence.moving_idx) <
                 ThisType::_clipped_scene_slice.size());
          moving_cloud.emplace_back(ThisType::_clipped_scene_slice[correspondence.moving_idx]);
          _correspondences.emplace_back(
            Correspondence(i, correspondence.fixed_idx, correspondence.response));
        }

        // ds provide merger with correspondences
        // ds we need to do it already because the clipped scene will be swapped out after this
        // operation by the graph slam module - TODO change this confusing mechanic
        merger_correspondence->setCorrespondences(&_correspondences);
      }
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessor_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::setClosure(
    const CorrespondenceVector& correspondences_,
    const EstimateType& reference_in_query_,
    const EstimateType& robot_in_moving_local_map_) {
    _correspondences.clear();
    if (!param_closure_merger.value()) {
      return;
    }

    // ds the correspondences are already global - will be freed after merging TODO smart pointer
    _correspondences.reserve(correspondences_.size());

    // ds we need to carry along the global points with correspondences from the current scene
    // ds the current map points are temporarily preserver in the scene slice
    // ds the scene will be swapped out by the graph slam system once we relocalized
    ThisType::_clipped_scene_slice.clear();
    ThisType::_clipped_scene_slice.reserve(correspondences_.size());

    std::cerr << "TrackerSliceProcessor_::setClosure|following indices are wrong" << std::endl;
    std::cerr << correspondences_.size() << std::endl;
    for (size_t i = 0; i < correspondences_.size(); ++i) {
      const Correspondence& correspondence = correspondences_[i];

      // ds moving is the index of the target local map (in which we relocalize -> fixed)
      _correspondences.emplace_back(
        Correspondence(correspondence.moving_idx, i, correspondence.response));

      // ds grab the global point from the scene and it it to the current
      assert(static_cast<size_t>(correspondence.fixed_idx) < ThisType::_scene_slice->size());
      ThisType::_clipped_scene_slice.emplace_back(
        (*ThisType::_scene_slice)[correspondence.fixed_idx]);
    }

    // ds for merging the closure correspondences we need a high level merger (2D-2D or 3D-3D)
    // ds this is because also the local maps store their data in high level format
    // ds the actual merge will be automatically performed in MultiTrackerSlice::merge
    std::cerr << "TrackerSliceProcessor::setClosure|clipped scene size"
              << ThisType::_clipped_scene_slice.size() << std::endl;
    param_closure_merger->setMeasurement(&(ThisType::_clipped_scene_slice) /*instead of adapted*/);
    param_closure_merger->setMeasurementInScene(reference_in_query_ /*closure correction*/);
    param_closure_merger->setCorrespondences(&_correspondences);
    _have_loop_closure_correspondences = true;
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessor_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::merge() {
    sanityCheck("TrackerSliceProcessor_::merge()");
    assert(param_merger.value());

    // ds prepare merger
    param_merger->setScene(ThisType::_scene_slice);
    param_merger->setMeasurement(&(ThisType::_measurement_slice));
    param_merger->setMeasurementInScene(ThisType::_robot_in_local_map * ThisType::_sensor_in_robot);

    // ds check if the merger supports correspondences (defined by its dynamic type)
    MergerCorrespondenceTypePtr merger_correspondence =
      std::dynamic_pointer_cast<MergerCorrespondenceType>(param_merger.value());
    if (!merger_correspondence) {
      // ds perform regular merge if no correspondences are required
      param_merger->compute();
      return;
    }

    // ds set global tracker pose estimate (global landmark refinement)
    merger_correspondence->setMeasurementInWorld(ThisType::_robot_in_world *
                                                 ThisType::_sensor_in_robot);

    // ds if loop closure data is available for correspondence based merging
    if (_have_loop_closure_correspondences) {
      // ds correspondence-based closure merger must be available
      if (!param_closure_merger.value()) {
        throw std::runtime_error("MultiTrackerSlice::merge|ERROR: correspondence-based merger for "
                                 "closure aligment not available");
      }

      // ds prepare merger and compute - moving, correspondences and transform have been set
      param_closure_merger->setScene(ThisType::_scene_slice);
      std::cerr << "closure merging" << std::endl;
      param_closure_merger->compute();
      std::cerr << "done" << std::endl;
      // ds free correspondences set in closure update
      _correspondences.clear();
      _have_loop_closure_correspondences = false;
      return;
    }

    // ds clipped scene (which carries correspondences) must be available for correspondence merger
    if (!ThisType::_clipped_scene_container) {
      throw std::runtime_error(
        "MultiTrackerSlice::merge|ERROR: auxiliary data not available for correspondence import");
    }

    // ds grab correspondences used for fixed measurement alignment from scene object
    CorrespondenceVector global_correspondences;
    using PropertyCorrespondenceVector = Property_<CorrespondenceVector>;
    PropertyCorrespondenceVector* correspondence_property =
      ThisType::_clipped_scene_container->template property<PropertyCorrespondenceVector>(
        ThisType::param_measurement_slice_name.value() +
        CorrespondenceFinderBase::auxiliary_data_suffix_correspondences);
    if (correspondence_property) {
      const CorrespondenceVector& local_correspondences = correspondence_property->value();

      // ds set correspondences between fixed and scene as determined by aligner
      // ds NOTE that the correspondences have been computed between fixed and local scene
      // ds for the merger we have to flip the correspondences and map them to the global scene
      // ds TODO figure out a workaround for this overhead here
      global_correspondences.reserve(local_correspondences.size());
      const std::vector<int>& local_to_global(param_clipper->globalIndices());
      assert(local_to_global.size() <= ThisType::_scene_slice->size());
      assert(local_to_global.size() >= local_correspondences.size());
      for (const Correspondence& correspondence : local_correspondences) {
        assert(static_cast<size_t>(correspondence.moving_idx) < local_to_global.size());
        global_correspondences.emplace_back(
          Correspondence(local_to_global[correspondence.moving_idx] /* map to global scene */,
                         correspondence.fixed_idx,
                         correspondence.response));
      }

      // ds set correspondences and merge
      merger_correspondence->setCorrespondences(&global_correspondences);
      merger_correspondence->compute();
    } else {
      // ds perform regular merge (we are in a new local map)
      merger_correspondence->compute();
    }
  }

  template <typename EstimateType_, typename FixedMeasurementType_, typename MovingSceneType_>
  void TrackerSliceProcessor_<EstimateType_, FixedMeasurementType_, MovingSceneType_>::clip() {
    sanityCheck("TrackerSliceProcessor_::clip()");
    param_clipper->setFullScene(ThisType::_scene_slice);
    param_clipper->setClippedSceneInRobot(&(ThisType::_clipped_scene_slice));
    param_clipper->setAuxiliaryData(ThisType::_clipped_scene_container,
                                    ThisType::param_scene_slice_name.value());
    param_clipper->setRobotInLocalMap(ThisType::_robot_in_local_map);
    param_clipper->setSensorInRobot(ThisType::_sensor_in_robot);

    param_clipper->compute();
  }

} // namespace srrg2_slam_interfaces
