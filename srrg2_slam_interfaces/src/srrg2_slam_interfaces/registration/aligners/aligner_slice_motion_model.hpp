#pragma once
#include "aligner_slice_processor_prior.h"
#include "srrg2_slam_interfaces/motion_models/motion_model_base.hpp"
#include <srrg_config/property_configurable.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_prior_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_prior_error_factor_ad.h>

namespace srrg2_slam_interfaces {

  /**
   * @brief slice that can process data from a TrackerSliceEstimationBuffer_ (trajectory chunk)
   */
  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class AlignerSliceMotionModel_
    : public AlignerSliceProcessorPrior_<FactorType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType = FactorType_;
    using FixedType  = FixedType_;
    using MovingType = MovingType_;
    using ThisType   = AlignerSliceMotionModel_<FactorType, FixedType, MovingType>;
    using BaseType   = AlignerSliceProcessorPrior_<FactorType, FixedType, MovingType>;
    using VariableType =
      typename VariableTypeAt_<typename FactorType::VariableTupleType, 0>::VariableType;
    using EstimateType    = typename VariableType::EstimateType;
    using MotionModelType = MotionModelBase_<EstimateType>;
    using AlignerType     = Aligner_<EstimateType, PropertyContainerBase, PropertyContainerBase>;
    // ds access to protected (?) setters
    template <typename T>
    friend class MultiAlignerBase_;

    PARAM(srrg2_core::PropertyConfigurable_<MotionModelType>,
          motion_model,
          "model used to estimate inter-frame motion",
          nullptr,
          nullptr);

    virtual ~AlignerSliceMotionModel_() {
    }

    /**
     *  @brief computes and sets the current motion model estimate to the aligner
     */
    void init(AlignerType* aligner_) override {
      if (!param_motion_model.value()) {
        throw std::runtime_error("AlignerSliceMotionModel_::init|ERROR: no motion model is set");
      }
      if (!ThisType::_fixed_slice) {
        throw std::runtime_error("AlignerSliceMotionModel_::init|ERROR: no fixed pose set");
      }
      assert(aligner_);
      BaseType::init(aligner_);

      // ds propagate motion model for given tracker pose estimate
      if (!ThisType::_fixed_slice->empty()) {
        // ds TODO generify this
        param_motion_model->setRobotInLocalMap(ThisType::_fixed_slice->back());
        if (ThisType::_fixed_slice->size() > 1) {
          param_motion_model->setRobotInLocalMapPrevious(*(ThisType::_fixed_slice->end() - 2));
        }
        param_motion_model->compute();
      }

      // ds set aligner estimate with motion model guess
      // srrg most likely that we must put the inverse
      //      std::cerr << "AlignerSliceMotionModel_::init()|check if we need the inverse or not"
      //                << std::endl;

      _motion_inverse = param_motion_model->estimate().inverse();
      ThisType::_aligner->setMovingInFixed(_motion_inverse);
    }

  protected:
    //! @brief set current motion model estimate as measurement
    void setupFactor() override {
      //      std::cerr << "AlignerSliceMotionModel_::setupFactor()|check if we need the inverse or
      //      not" << std::endl;
      ThisType::_factor->setMeasurement(_motion_inverse);
    }

    EstimateType _motion_inverse = EstimateType::Identity();
  };

  // ds we should be able to pick arbitrary SE(d) factors
  // ds since ultimately we are only interested in VariableType::EstimateType
  using AlignerSliceMotionModel2D = AlignerSliceMotionModel_<srrg2_solver::SE2PriorErrorFactor,
                                                             srrg2_core::StdDequeEigenIsometry2f,
                                                             srrg2_core::StdDequeEigenIsometry2f>;
  using AlignerSliceMotionModel3D =
    AlignerSliceMotionModel_<srrg2_solver::SE3PriorErrorFactorAD, // ds quaternions
                             srrg2_core::StdDequeEigenIsometry3f,
                             srrg2_core::StdDequeEigenIsometry3f>;

  using AlignerSliceMotionModel2DPtr = std::shared_ptr<AlignerSliceMotionModel2D>;
  using AlignerSliceMotionModel3DPtr = std::shared_ptr<AlignerSliceMotionModel3D>;

} // namespace srrg2_slam_interfaces
