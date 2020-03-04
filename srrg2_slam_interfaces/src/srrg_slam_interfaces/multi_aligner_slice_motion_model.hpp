#pragma once
#include <srrg_solver/variables_and_factors/types_2d/se2_prior_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_prior_error_factor_ad.h>

#include "motion_models/motion_model_base.hpp"
#include "multi_aligner_slice_prior.h"

namespace srrg2_slam_interfaces {

  // ds slice that can process data from a MultiTrackerSliceEstimationBuffer_ (trajectory chunk)
  template <typename FactorType_, typename FixedType_, typename MovingType_>
  class MultiAlignerSliceMotionModel_
    : public MultiAlignerSlicePrior_<FactorType_, FixedType_, MovingType_> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using FactorType   = FactorType_;
    using FixedType    = FixedType_;
    using MovingType   = MovingType_;
    using ThisType     = MultiAlignerSliceMotionModel_<FactorType, FixedType, MovingType>;
    using BaseType     = MultiAlignerSlicePrior_<FactorType_, FixedType_, MovingType_>;
    using VariableType = typename VariableTypeAt_<typename FactorType_::VariableTupleType, 0>::VariableType;
    using EstimateType = typename VariableType::EstimateType;
    using MotionModelType = MotionModelBase_<EstimateType>;
    using AlignerType     = Aligner_<EstimateType, PropertyContainerBase, PropertyContainerBase>;
    // ds access to protected (?) setters
    template <typename T>
    friend class MultiAlignerBase_;

    PARAM(srrg2_core::PropertyConfigurable_<MotionModelType>,
          motion_model,
          "model used to estimate interframe motion",
          nullptr,
          nullptr);

    virtual ~MultiAlignerSliceMotionModel_() {
    }

    //! computes and sets the current motion model estimate to the aligner
    void init(AlignerType* aligner_) override {
      if (!param_motion_model.value()) {
        throw std::runtime_error(
          "MultiAlignerSliceMotionModel_::init|ERROR: no motion model is set");
      }
      if (!ThisType::_fixed_slice) {
        throw std::runtime_error("MultiAlignerSliceMotionModel_::init|ERROR: no fixed pose set");
      }
      assert(aligner_);
      ThisType::_aligner = aligner_;

      // ds propagate motion model for given tracker pose estimate
      if (!ThisType::_fixed_slice->empty()) {
        // ds TODO generify this
        param_motion_model->addTrackerEstimate(ThisType::_fixed_slice->back());
        if (ThisType::_fixed_slice->size() > 1) {
          param_motion_model->setTrackerEstimatePrevious(*(ThisType::_fixed_slice->end() - 2));
        }
        param_motion_model->compute();
      }

      // ds set aligner estimate with motion model guess
      ThisType::_aligner->setEstimate(param_motion_model->estimate());
    }

  protected:
    //! set current motion model estimate as measurement
    void setupFactor() override {
      ThisType::_factor->setMeasurement(param_motion_model->estimate());
    }
  };

  // ds we should be able to pick arbitrary SE(d) factors
  // ds since ultimately we are only interested in VariableType::EstimateType
  using MultiAlignerSliceMotionModel2D =
    MultiAlignerSliceMotionModel_<srrg2_solver::SE2PriorErrorFactor,
                                  srrg2_core::StdDequeEigenIsometry2f,
                                  srrg2_core::StdDequeEigenIsometry2f>;
  using MultiAlignerSliceMotionModel3D =
    MultiAlignerSliceMotionModel_<srrg2_solver::SE3PriorErrorFactorAD, // ds quaternions
                                  srrg2_core::StdDequeEigenIsometry3f,
                                  srrg2_core::StdDequeEigenIsometry3f>;

  using MultiAlignerSliceMotionModel2DPtr = std::shared_ptr<MultiAlignerSliceMotionModel2D>;
  using MultiAlignerSliceMotionModel3DPtr = std::shared_ptr<MultiAlignerSliceMotionModel3D>;

} // namespace srrg2_slam_interfaces
