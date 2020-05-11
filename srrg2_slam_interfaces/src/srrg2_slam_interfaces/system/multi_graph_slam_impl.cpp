#include "multi_graph_slam.h"
#include "srrg2_slam_interfaces/registration/loop_detector/multi_loop_detector_brute_force.h"
#include "srrg2_slam_interfaces/trackers/multi_tracker.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename LoopClosureType_>
  MultiGraphSLAM_<LoopClosureType_>::MultiGraphSLAM_() {
    // srrg global solver
    {
      param_global_solver.setValue(SolverPtr(new Solver));
      _default_info.setIdentity();
    }

    // srrg closure validator
    {
      param_closure_validator.setValue(
        FactorGraphClosureValidatorPtr(new FactorGraphClosureValidator));
    }

    // srrg loop detector
    {
      using BFDetectorType     = MultiLoopDetectorBruteForce_<ThisType, AlignerType>;
      BFDetectorType* detector = new BFDetectorType;
      detector->param_relocalize_aligner.setValue(std::shared_ptr<AlignerType>(new AlignerType));
      param_loop_detector.setValue(std::shared_ptr<BFDetectorType>(detector));
    }

    // srrg tracker
    {
      param_tracker.setValue(std::shared_ptr<TrackerType>(new TrackerType));
      param_tracker->param_aligner.setValue(std::shared_ptr<AlignerType>(new AlignerType));
    }

    // srrg splitting criterion
    {
      using SplittingDistanceBased = LocalMapSplittingCriterionDistance_<ThisType>;
      param_splitting_criterion.setValue(
        std::shared_ptr<LocalMapSplittingCriterionDistance_<ThisType>>(new SplittingDistanceBased));
    }

    // srrg relocalizer
    { param_relocalizer.setValue(std::shared_ptr<RelocalizerType>(new RelocalizerType)); }

    // srrg initializer (ds TODO optional instead of dummies)
    //    { param_initializer.setValue(InitializerPtr(new DummyInitializer)); }
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::makeNewMap(float info_scale_) {
    PROFILE_TIME("MultiGraphSLAM::makeNewMap");
    std::cerr << FG_GREEN("MultiGraphSLAM::makeNewMap|opening new local map: ")
              << ThisType::_graph->variables().size() << FG_GREEN(" (graph ID: ")
              << ThisType::_graph->lastGraphId() << FG_GREEN(" # processed measurements: ")
              << _number_of_processed_measurements << FG_GREEN(")") << std::endl;
    // cache the value of the robot pose  as it is destroyed when creating a new local map
    const EstimateType prev_robot_pose     = robotInWorld();
    const LocalMapType* previous_local_map = _current_local_map;

    // ds create a new, empty local map
    _current_local_map = new LocalMapType();
    _current_local_map->setEstimate(prev_robot_pose);

    // tracker populates the map
    // ds this will also migrate the correspondences from the last aligment to the new map
    param_tracker->populateScene(_current_local_map->dynamic_properties);

    /// ds populate pose graph with local map state estimates
    _graph->addVariable(VariableBasePtr(_current_local_map));
    if (previous_local_map) {
      FactorBaseTypePtr factor(new FactorBaseType);
      factor->setVariableId(0, previous_local_map->graphId());
      factor->setVariableId(1, _current_local_map->graphId());
      factor->setMeasurement(_robot_in_local_map);
      typename FactorBaseType::InformationMatrixType info;
      factor->setInformationMatrix(_default_info * info_scale_);
      _graph->addFactor(factor);

      // ds train the previous local map in the detector for place recognition
      if (param_loop_detector.value()) {
        param_loop_detector->addPreviousQuery();
      }
    } else {
      _current_local_map->setStatus(VariableBase::Fixed);
    }

    _robot_in_local_map.setIdentity();
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::compute() {
    PROFILE_TIME("MultiGraphSLAM::compute");
    // ds optional initialization TODO rework this
    auto initializer = param_initializer.value();
    if (initializer && !initializer->isInitialized()) {
      std::cerr << "MultiGraphSLAM::compute|initializing" << std::endl;

      // ds set shared platform if needed TODO rework this
      if (!_platform) {
        ThisType::setPlatform(PlatformPtr(new Platform()));
      }

      // ds run initializer
      initializer->setRawData(_message);
      initializer->initialize();

      // ds skip processing if insufficient data is available for initialization
      if (!initializer->isInitialized()) {
        return;
      }
    }

    if (!_graph) {
      std::cerr << "MultiGraphSLAM_::compute|graph not set, creating instance" << std::endl;
      _graph = FactorGraphPtr(new FactorGraph);
    }

    auto tracker = param_tracker.value();
    if (!tracker) {
      throw std::runtime_error("MultiGraphSLAM::compute()|tracker not set");
    }

    auto splitting_criterion = param_splitting_criterion.value();
    if (!splitting_criterion) {
      throw std::runtime_error("MultiGraphSLAM::compute()|splitting criterion not set");
    }
    splitting_criterion->setSLAMAlgorithm(this);
    tracker->setPlatform(this->_platform);

    if (!_current_local_map) {
      std::cerr << "MultiGraphSLAM::compute|creating first map!" << std::endl;
      makeNewMap();
      tracker->setScene(&_current_local_map->dynamic_properties);
      tracker->setRobotInLocalMap(_robot_in_local_map);
    }

    // global pose
    tracker->setRawData(_message);
    tracker->preprocessRawData();
    tracker->align(); // ia I really do not like this name
    switch (tracker->status()) {
      case TrackerBase::Initializing:
        std::cerr << FG_YELLOW("MultiGraphSLAM::compute|initializing!") << std::endl;
        break;
      case TrackerBase::Initialized:
        std::cerr << FG_YELLOW("MultiGraphSLAM::compute|ready to go!") << std::endl;
        break;
      case TrackerBase::Tracking: {
        _robot_in_local_map = tracker->robotInLocalMap();
        splitting_criterion->compute();
        if (!splitting_criterion->hasToSplit()) {
          break;
        }
        GraphItemPtrSet_<LoopClosureTypePtr> relocalize_closures;
        loopDetect();
        loopValidate(relocalize_closures);
        optimize();
        relocalize(relocalize_closures);
        if (!_relocalized_closure) {
          // ds a new sub-scene (e.g. local map will be populated)
          makeNewMap(1);
        } else {
          // ds we do not have to create a new local scene but instead reload an old one
          std::cerr << FG_GREEN("MultiGraphSLAM::compute|reloading old local map (graph ID: ")
                    << _current_local_map->graphId() << FG_GREEN(")") << std::endl;

          // ds the correspondences from the closures can be re-used by the tracker for merging
          // ds it is fine if the correspondences are destroyed by scope after the merge
          const EstimateType& reference_in_query = _relocalized_closure->measurement();

          tracker->setClosure(
            _relocalized_closure->correspondences, reference_in_query, _robot_in_local_map);

          // ds reset closure buffer for next iteration (potentially without relocalization)
          _relocalized_closure = nullptr;
        }
        tracker->setScene(&_current_local_map->dynamic_properties);
        tracker->setRobotInLocalMap(
          _robot_in_local_map); // ds estimate is set to I after makeNewMap
        break;
      }
      case TrackerBase::Lost:
        std::flush(std::cerr);
        std::cerr << FG_YELLOW("MultiGraphSLAM::compute|local map lost") << std::endl;
        std::flush(std::cerr);
        makeNewMap(0.1);
        tracker->setScene(&_current_local_map->dynamic_properties);
        // tracker->setEstimate(_robot_in_local_map);
        break;
      case TrackerBase::Error:
        std::flush(std::cerr);
        std::cerr << FG_YELLOW("MultiGraphSLAM::compute|tracker status ERROR") << std::endl;
        std::flush(std::cerr);
        break;
      default:
        throw std::runtime_error("MultiGraphSLAM::compute|unknown tracker status");
    }

    // ds set global pose of current tracker scene to the tracker
    // ds this information will be used for global landmark updates in the merge phase
    tracker->setLocalMapInWorld(_current_local_map->estimate());
    tracker->merge();
    ++_number_of_processed_measurements;
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::loopDetect() {
    PROFILE_TIME("MultiGraphSLAM::loopDetect");
    if (param_loop_detector.value()) {
      param_loop_detector->setSLAMAlgorithm(this);
      param_loop_detector->compute();
    }
  }

  inline void printClosures(const FactorGraphClosureValidator::ClosureStatsPtrMap& closures) {
    for (auto c_it = closures.begin(); c_it != closures.end(); ++c_it) {
      auto& stats = *(c_it->second);
      std::cerr << "f: " << c_it->first->graphId() << " n_rounds: " << stats.num_rounds_checked
                << " n_checks: " << stats.num_times_checked << " n_ok: " << stats.num_times_good
                << " status: " << stats.status << std::endl;
    }
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::loopValidate(
    GraphItemPtrSet_<LoopClosureTypePtr>& reloc_closures_) {
    PROFILE_TIME("MultiGraphSLAM::loopValidate");
    num_valid_closures = 0;

    if (!param_loop_detector.value()) {
      return;
    }
    auto& detected_closures = param_loop_detector->detectedClosures();

    if (!detected_closures.size()) {
      return;
    }

    for (auto it : detected_closures) {
      it->setEnabled(false);
      _graph->addFactor(it);
    }
    _graph->bindFactors();

    // if not validator set, we say that all closures are good
    std::shared_ptr<FactorGraphClosureValidator> validator = param_closure_validator.value();
    if (!validator) {
      reloc_closures_.insert(detected_closures.begin(), detected_closures.end());
      num_valid_closures = reloc_closures_.size();
      for (auto c : reloc_closures_) {
        c->setEnabled(true);
      }
      return;
    }

    // feed the validator
    validator->setGraph(_graph);
    for (auto it = detected_closures.begin(); it != detected_closures.end(); ++it) {
      validator->addClosure(*it);
    }

    // voting_scheme
    validator->compute(_current_local_map);

    //    std::cerr << "loopValidate()| num partitions" << validator->partitionsNum() <<
    //    std::endl; std::cerr << "loopValidate()| factor stats" << std::endl;
    //    printClosures(validator->updatedClosures());

    // choose what to do with each closure
    for (auto it = validator->updatedClosures().begin(); it != validator->updatedClosures().end();
         ++it) {
      auto closure = std::dynamic_pointer_cast<LoopClosureType>(it->first);
      if (!closure) {
        continue;
      }
      const FactorGraphClosureValidator::ClosureStats* stats = it->second;
      switch (stats->status) {
          // remove rejected (also from graph)
        case FactorGraphClosureValidator::ClosureStats::Status::Rejected:
          validator->removeClosure(closure);
          _graph->removeFactor(closure);
          break;
        case FactorGraphClosureValidator::ClosureStats::Status::Accepted:
          // promote accepted and leave them in pending list
          validator->removeClosure(closure);
          ++num_valid_closures;
          closure->setEnabled(true);
          if (std::binary_search(detected_closures.begin(), detected_closures.end(), closure)) {
            reloc_closures_.insert(closure);
          }
          break;
        default:;
      }
    }
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::optimize() {
    PROFILE_TIME("MultiGraphSLAM::optimize");
    if (!num_valid_closures) {
      return;
    }

    SolverPtr global_solver = this->param_global_solver.value();
    if (!global_solver) {
      throw std::runtime_error("MultiGraphSLAM::optimize|global_solver == NULL");
    }
    if (!_graph) {
      throw std::runtime_error("MultiGraphSLAM::optimize|graph == NULL");
    }

    _graph->bindFactors();
    global_solver->setGraph(_graph);
    global_solver->compute();
  }

  template <typename LoopClosureType_>
  bool MultiGraphSLAM_<LoopClosureType_>::putMessage(BaseSensorMessagePtr message_) {
    setRawData(message_);
    compute();
    return true;
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::relocalize(
    const GraphItemPtrSet_<LoopClosureTypePtr>& reloc_closures_) {
    PROFILE_TIME("MultiGraphSLAM::relocalize");
    _relocalized_closure = nullptr;
    if (!reloc_closures_.size()) {
      return;
    }

    if (!param_relocalizer.value()) {
      return;
    }

    param_relocalizer->setSLAMAlgorithm(this);
    param_relocalizer->setClosureCandidates(reloc_closures_);
    param_relocalizer->compute();
    if (param_relocalizer->relocalizationMap()) {
      _current_local_map   = param_relocalizer->relocalizationMap();
      _robot_in_local_map  = param_relocalizer->robotInLocalMap();
      _relocalized_closure = param_relocalizer->relocalizedClosure();
    }
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::adjustDrawingAttributes() const {
    for (const auto& it : _graph->variables()) {
      LocalMapType* m = const_cast<LocalMapType*>(dynamic_cast<const LocalMapType*>(it.second));
      m->map_status   = LocalMapType::Idle;
      if (m == _current_local_map) {
        m->map_status = LocalMapType::Current;
        continue;
      }
      if (param_loop_detector.value() && param_loop_detector->attemptedClosures().count(m)) {
        m->map_status = LocalMapType::LoopChecked;
        continue;
      }
    }
  }

  template <typename LoopClosureType_>
  void MultiGraphSLAM_<LoopClosureType_>::draw(ViewerCanvasPtr canvas) const {
    if (!canvas || !_graph || !_current_local_map) {
      return;
    }
    _graph->bindFactors();

    // ds draw current robot pose (inside local map) in RED TODO move to local maps?
    canvas->pushMatrix();
    canvas->pushColor();
    canvas->setColor(srrg2_core::ColorPalette::color3fRed());

    // ds draw robot pose
    _current_local_map->setDrawingReferenceFrame(canvas, robotInWorld());
    canvas->putSphere(_current_local_map->size_local_map_sphere);

    // ds draw current correspondences at robot pose
    param_tracker->draw(canvas);
    canvas->popAttribute();
    canvas->popMatrix();

    // ds draw pose graph
    adjustDrawingAttributes();
    for (const auto& it : _graph->variables()) {
      it.second->draw(canvas);
    }
    for (const auto& it : _graph->factors()) {
      it.second->draw(canvas);
    }

    canvas->flush();
  }

} // namespace srrg2_slam_interfaces
