#pragma once
#include "aligner_slice_processor_prior.h"
#include <srrg_solver/variables_and_factors/types_2d/se2_prior_error_factor.h>
#include <srrg_solver/variables_and_factors/types_3d/se3_prior_error_factor_ad.h>

namespace srrg2_slam_interfaces {
  // priors
  class AlignerSliceOdom2DPrior
    : public AlignerSliceProcessorPrior_<SE2PriorErrorFactor, Isometry2f, Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = AlignerSliceProcessorPrior_<SE2PriorErrorFactor, Isometry2f, Isometry2f>;

    static constexpr int ErrorDim = SE2PriorErrorFactor::ErrorDim;
    using DiagInfoVector          = srrg2_core::Vector_<float, ErrorDim>;

    PARAM(srrg2_core::PropertyEigen_<DiagInfoVector>,
          diagonal_info_matrix,
          "value of the information matrix's diagonal",
          DiagInfoVector::Ones() * 1e2,
          nullptr);

    virtual ~AlignerSliceOdom2DPrior() = default;
    void init(AlignerType* aligner_) override;

  protected:
    void setupFactor() override;
  };

  using AlignerSliceOdom2DPriorPtr = std::shared_ptr<AlignerSliceOdom2DPrior>;

  class AlignerSliceOdom3DPrior
    : public AlignerSliceProcessorPrior_<SE3PriorErrorFactorAD, Isometry3f, Isometry3f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using BaseType = AlignerSliceProcessorPrior_<SE3PriorErrorFactorAD, Isometry3f, Isometry3f>;

    static constexpr int ErrorDim = SE3PriorErrorFactorAD::ErrorDim;
    using DiagInfoVector          = srrg2_core::Vector_<float, ErrorDim>;

    PARAM(srrg2_core::PropertyEigen_<DiagInfoVector>,
          diagonal_info_matrix,
          "value of the information matrix's diagonal",
          DiagInfoVector::Ones(),
          nullptr);

    virtual ~AlignerSliceOdom3DPrior() = default;

    void init(AlignerType* aligner_) override;

  protected:
    void setupFactor() override;
  };

  using AlignerSliceOdom3DPriorPtr = std::shared_ptr<AlignerSliceOdom3DPrior>;

} // namespace srrg2_slam_interfaces
