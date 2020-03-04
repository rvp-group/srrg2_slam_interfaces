#pragma once
#include "aligner.h"
#include "srrg_data_structures/iterator_interface.h"
#include "srrg_property/property_container.h"
#include "srrg_solver/solver_core/factor_correspondence_driven.h"
#include "srrg_solver/solver_core/solver.h"

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename VariableType_>
  class MultiAlignerBase_;

  template <typename VariableType_>
  class MultiAlignerSliceBase_ : public Configurable,
                                 public srrg2_core::PlatformUser,
                                 public srrg2_core::DrawableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ThisType     = MultiAlignerSliceBase_<VariableType_>;
    using VariableType = VariableType_;
    using EstimateType = typename VariableType::EstimateType;

    template <typename T>
    friend class MultiAlignerBase_;

    using AlignerType = Aligner_<EstimateType, PropertyContainerBase, PropertyContainerBase>;
    PARAM(PropertyConfigurable_<RobustifierBase>,
          robustifier,
          "robustifier used on this slice",
          0,
          0);

    PARAM(PropertyString,
          fixed_slice_name,
          "name of the slice in the fixed scene",
          "",
          &_fixed_slice_changed_flag);

    PARAM(PropertyString,
          moving_slice_name,
          "name of the slice in the moving scene",
          "",
          &_moving_slice_changed_flag);

    PARAM(PropertyString, frame_id, "name of the sensor's frame in the tf tree", "", nullptr);
    PARAM(PropertyString, base_frame_id, "name of the base frame in the tf tree", "", nullptr);

    virtual const CorrespondenceVector& correspondences() const {
      throw std::runtime_error("dummy function, shouldn't be called");
      return _dummy_corr;
    }

    virtual CorrespondenceVector& correspondences() {
      throw std::runtime_error("dummy function, shouldn't be called");
      return _dummy_corr;
    }

  protected:
    virtual void init(AlignerType* aligner_) {
      _aligner = aligner_;
    }

    // sets the fixed scene, and isolates the channels
    virtual void setFixed(PropertyContainerBase* fixed_scene_) {
      _fixed_scene = fixed_scene_;
      bindFixed();
    }

    // sets the moving scene, isolating the channels
    virtual void setMoving(PropertyContainerBase* moving_scene_) {
      _moving_scene = moving_scene_;
      bindMoving();
    }

    // set estimate for the correspondence finder
    virtual void setEstimate(const EstimateType& est_) = 0;

    virtual void initializeFactor() = 0;

    // tg generate a factor out of correspondences
    virtual FactorBasePtr factor() = 0;

    // binds the robustifier if any
    virtual void bindRobustifier() = 0;

    //! computes associations for current moving scene and fixed measurements
    //! returns a reference to computed associations for external use (e.g. pruning)
    virtual CorrespondenceVector* computeAssociation() = 0;

    //! stores associations in an auxiliary data buffer that can be re-used during merge
    virtual void storeCorrespondences() = 0;

    virtual bool associationGood() const = 0;

    // binds the slice of the fixed scene to the fixed slice pointer in class
    virtual void bindFixed() = 0;
    // binds the slice of the moving scene to the moving slice pointer in class
    virtual void bindMoving() = 0;

    virtual int numCorrespondences() const {
      return 0;
    }

    //! initializes the factor based on inherent values of the slice (e.g. camera matrix, prior)
    //! this method is called for all factors for each iteration (TODO intended?)
    virtual void setupFactor() = 0;

    PropertyContainerBase* _fixed_scene;
    PropertyContainerBase* _moving_scene;

    bool _robustifier_changed_flag  = true;
    bool _fixed_slice_changed_flag  = true;
    bool _moving_slice_changed_flag = true;

    AlignerType* _aligner = nullptr;

  private:
    CorrespondenceVector _dummy_corr;
  };

  template <typename VariableType_>
  using MultiAlignerSliceBasePtr_ = std::shared_ptr<MultiAlignerSliceBase_<VariableType_>>;

  // concrete implementation of the slice
  // - it has a correspondence finder, and stores a correspondence vector
  // - it has a factor
  // - it implements a factor iterator interface, that
  //      can iterate in the correspondence vector, and instantiate
  //      the factor accordingly
  // a multi cue aligner has many of those, all contributing to the same
  // default solver (stored, once in the aligner)
  // the scene is presented as a property container
  // the slice of the fixed scene is captured by its property name
  // the slice in the moving scene is captured by its property name too
  // the assigment to the slices to fixed and moving
  // is done in the methods setFixed and setMoving, to be overridden

  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class MultiAlignerSlice_ : public MultiAlignerSliceBase_<
                               typename VariableTypeAt_<typename FactorType_::VariableTupleType,
                                                        0>::VariableType::BaseVariableType> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using FactorBaseType  = FactorType_;
    using FixedType       = FixedType_;
    using FixedPointType  = typename FixedType::value_type;
    using MovingType      = MovingType_;
    using MovingPointType = typename MovingType::value_type;
    using ThisType        = MultiAlignerSlice_<FactorBaseType, FixedType, MovingType>;
    using VariableType =
      typename VariableTypeAt_<typename FactorType_::VariableTupleType, 0>::VariableType;
    using BaseVariableType = typename VariableType::BaseVariableType;
    using EstimateType     = typename VariableType::EstimateType;
    using FactorType       = FactorCorrespondenceDriven_<FactorBaseType, FixedType, MovingType>;
    using FactorTypePtr    = std::shared_ptr<FactorType>;
    using CorrespondenceFinderType = CorrespondenceFinder_<EstimateType, FixedType, MovingType>;
    using InformationMatrixVector  = typename FactorType::InformationMatrixVector;
    template <typename T>
    friend class MultiAlignerBase_;

    friend class FactorBase;
    friend class FactorCorrespondenceDriven_<FactorBaseType, FixedType, MovingType>;

    PARAM(PropertyConfigurable_<CorrespondenceFinderType>,
          finder,
          "correspondence finder used in this cue",
          0,
          &_finder_changed_flag);

    PARAM(PropertyInt,
          min_num_correspondences,
          "minimum number of correspondences in this slice",
          0,
          nullptr);

    MultiAlignerSlice_() : _factor(new FactorType) {
    }

    virtual ~MultiAlignerSlice_() {
    }
    FixedType* fixed() {
      return _fixed_slice;
    }
    MovingType* moving() {
      return _moving_slice;
    }

    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override {
      // ds do not force subclasses to override (YET)
    }

    // posesistema
    const CorrespondenceVector& correspondences() const override {
      return _correspondences;
    }

    CorrespondenceVector& correspondences() override {
      return _correspondences;
    }

  protected:
    // tg instanciate the factor, the moving and fixed scenes are propagated to the new factor,
    // if present
    void initializeFactor() override {
      _factor.reset(new FactorType);
      if (_fixed_slice) {
        bindFixed();
      }
      if (_moving_slice) {
        bindMoving();
      }
    }

    // propagates the transform to the inner modules
    void setEstimate(const EstimateType& est_) override;

    FactorBasePtr factor() override;

    virtual CorrespondenceVector* computeAssociation() override;

    virtual void storeCorrespondences() override;

    virtual bool associationGood() const override;

    void sanityCheck(const std::string& message = "MultiAlignerSlice_");

    int numCorrespondences() const override {
      return _correspondences.size();
    }

    FixedType* _fixed_slice   = nullptr;
    MovingType* _moving_slice = nullptr;

    FactorTypePtr _factor = nullptr;
    virtual void setupFactor() {
      // ds has to be overridden if factors require specific setup
      // tg for example set the information matrix for each correspondence
    }

    void setupFactorWithSensor(const std::string& class_name_) {
      if (!this->param_frame_id.value().length() || !this->_platform) {
        std::cerr << "frame id: " << this->param_frame_id.value() << std::endl;
        std::cerr << "platform:\n" << this->_platform << std::endl;
        std::cerr << FG_YELLOW(class_name_ << "::setupFactor|no frame id or not platform set")
                  << std::endl;
      }
      ThisType::_factor->setSensorInRobot(ThisType::_sensor_in_robot);
    }
    CorrespondenceVector _correspondences;
    InformationMatrixVector _fixed_information_matrix_vector;
    bool _finder_changed_flag = true;
    virtual void bindRobustifier() {
      _factor->setRobustifier(this->param_robustifier.value().get());
    }
    virtual void bindFixed() override;
    virtual void bindMoving() override;

    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _sensor_in_robot = sensor_in_robot_;
      _robot_in_sensor = _sensor_in_robot.inverse();
    }

    // ds visualization only TODO burst and retrieve from somewhere (now set by projective subs)
    EstimateType _camera_matrix;
    // gg: hack
    EstimateType _sensor_in_robot = EstimateType::Identity();
    EstimateType _robot_in_sensor = EstimateType::Identity();
  };

} // namespace srrg2_slam_interfaces
