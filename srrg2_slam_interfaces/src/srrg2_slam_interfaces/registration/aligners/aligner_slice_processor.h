#pragma once
#include "aligner.h"
#include "aligner_slice_processor_prior.h"
#include "srrg2_slam_interfaces/registration/correspondence_finder.h"
#include <srrg_config/property_configurable.h>
#include <srrg_data_structures/iterator_interface.h>
#include <srrg_solver/solver_core/factor_correspondence_driven.h>
#include <srrg_solver/solver_core/solver.h>

namespace srrg2_slam_interfaces {
  /**
   * @brief concrete implementation of an aligner slice processor
   * - it has a correspondence finder, and stores a correspondence vector
   * - it has a factor
   * - it implements a factor iterator interface, that
   *      can iterate in the correspondence vector, and instantiate
   *      the factor accordingly
   * a multi cue aligner has many of those, all contributing to the same
   * default solver (stored, once in the aligner)
   * the scene is presented as a property container
   * the slice of the fixed scene is captured by its property name
   * the slice in the moving scene is captured by its property name too
   * the assigment to the slices to fixed and moving
   * is done in the methods setFixed and setMoving, to be overridden
   */
  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class AlignerSliceProcessor_
    : public AlignerSliceProcessorPrior_<
        FactorCorrespondenceDriven_<FactorType_, FixedType_, MovingType_>,
        FixedType_,
        MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using FactorBaseType  = FactorType_;
    using FixedType       = FixedType_;
    using FixedPointType  = typename FixedType::value_type;
    using MovingType      = MovingType_;
    using MovingPointType = typename MovingType::value_type;
    using ThisType        = AlignerSliceProcessor_<FactorBaseType, FixedType, MovingType>;
    using VariableType =
      typename VariableTypeAt_<typename FactorType_::VariableTupleType, 0>::VariableType;
    using BaseVariableType = typename VariableType::BaseVariableType;
    using FactorType       = FactorCorrespondenceDriven_<FactorBaseType, FixedType, MovingType>;
    using BaseType         = AlignerSliceProcessorPrior_<FactorType, FixedType, MovingType>;
    using EstimateType     = typename VariableType::EstimateType;
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

    AlignerSliceProcessor_() : BaseType() {
    }

    virtual ~AlignerSliceProcessor_() {
    }
    /**
     * @brief retrieve fixed slice
     * @return pointer to casted fixed data slice
     */
    FixedType* fixed() {
      return BaseType::_fixed_slice;
    }
    /**
     * @brief retrieve moving slice
     * @return pointer to casted moving data slice
     */
    MovingType* moving() {
      return BaseType::_moving_slice;
    }

    void draw(srrg2_core::ViewerCanvasPtr canvas_) const override {
      // ds do not force subclasses to override (YET)
    }

    const CorrespondenceVector& correspondences() const override {
      return _correspondences;
    }

    CorrespondenceVector& correspondences() override {
      return _correspondences;
    }

  protected:
    void setMovingInFixed(const EstimateType& moving_in_fixed_) override;

    virtual CorrespondenceVector* computeCorrespondences() override;

    virtual void storeCorrespondences() override;

    virtual bool correspondencesGood() const override;

    /**
     * @brief checks for inconsistences
     * @param[in] message: prefix log message
     */
    void sanityCheck(const std::string& message = "AlignerSliceProcessor_");

    int numCorrespondences() const override {
      return _correspondences.size();
    }

    virtual void setupFactor() override {
    }

    virtual void bindRobustifier() override {
      BaseType::_factor->setRobustifier(BaseType::param_robustifier.value().get());
    }

    virtual void bindFixed() override;
    virtual void bindMoving() override;

    /**
     * @brief cache the sensor pose wrt the robot
     * @param[in] sensor_in_robot_: calibrated pose of the sensor in robot frame
     */
    inline void setSensorInRobot(const EstimateType& sensor_in_robot_) {
      _sensor_in_robot = sensor_in_robot_;
      _robot_in_sensor = _sensor_in_robot.inverse();
    }

    /**
     * @brief set the sensor pose wrt the robot in the factor
     * @param[in] class name: prefix for logging
     */
    void setupFactorWithSensor(const std::string& class_name_) {
      if (!ThisType::param_frame_id.value().length() || !ThisType::_platform) {
        std::cerr << "frame id: " << ThisType::param_frame_id.value() << std::endl;
        std::cerr << "platform:\n" << ThisType::_platform << std::endl;
        std::cerr << FG_YELLOW(class_name_ << "::setupFactor|no frame id or not platform set")
                  << std::endl;
      }
      ThisType::_factor->setSensorInRobot(ThisType::_sensor_in_robot);
    }

    virtual bool isPrior() override {
      return false;
    }

    CorrespondenceVector _correspondences; /**< computed correspondences*/
    InformationMatrixVector
      _fixed_information_matrix_vector; /**< diagonal value of the information matrix */
    bool _finder_changed_flag = true;   /**< correspondence finder control flag  */

    typename EstimateType::LinearMatrixType _camera_matrix =
      EstimateType::LinearMatrixType::Identity(); /**< visualization only TODO burst and retrieve
                                 from somewhere (now set by projective subs) */
    EstimateType _sensor_in_robot =
      EstimateType::Identity(); /**< cached pose of the sensor wrt the robot */
    EstimateType _robot_in_sensor =
      EstimateType::Identity(); /**< cached pose of the robot wrt the sensor */
  };

} // namespace srrg2_slam_interfaces
