#pragma once
#include "aligner.h"
#include <srrg_data_structures/iterator_interface.h>
#include <srrg_property/property_container.h>
#include <srrg_solver/solver_core/factor_correspondence_driven.h>
#include <srrg_solver/solver_core/solver.h>

namespace srrg2_slam_interfaces {
  using namespace srrg2_core;
  using namespace srrg2_solver;

  template <typename VariableType_>
  class MultiAlignerBase_;

  /**
   * @brief base class for aligner slice processor
   * provides the interfaces for a slice-wise registration
   */
  template <typename VariableType_>
  class AlignerSliceProcessorBase_ : public Configurable,
                                     public srrg2_core::PlatformUser,
                                     public srrg2_core::DrawableBase {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    using ThisType     = AlignerSliceProcessorBase_<VariableType_>;
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

    //! @brief default ctor
    AlignerSliceProcessorBase_() = default;
    //! @brief virtual dtor
    virtual ~AlignerSliceProcessorBase_() = default;

    /**
     * @brief const correspondence vector getter
     * The correspondences are computed by the correspondence finder
     * The correspondence finder is not mandatory, thus this function could be useless in some cases
     * @return the const computed correspondence vector
     */
    virtual const CorrespondenceVector& correspondences() const {
      throw std::runtime_error(
        "AlignerSliceBase_::correspondence()|can't use the base class function");
      return _dummy_corr;
    }

    /**
     * @brief correspondence vector getter
     * The correspondences are computed by the correspondence finder
     * The correspondence finder is not mandatory, thus this function could be useless in some cases
     * @return the computed correspondence vector
     */
    virtual CorrespondenceVector& correspondences() {
      throw std::runtime_error(
        "AlignerSliceBase_::correspondence()|can't use the base class function");
      return _dummy_corr;
    }

  protected:
    /**
     * @brief initialize the slice with the related aligner
     * @param[in] aligner_: the base aligner that uses this slice
     */
    virtual inline void init(AlignerType* aligner_) {
      _aligner = aligner_;
    }

    /**
     * @brief demux the scene to gather the correct fixed slice
     * @param[in] fixed_scene_: the whole fixed scene (in tracking case use the measurement scene)
     */
    virtual void setFixed(PropertyContainerBase* fixed_scene_) {
      _fixed_scene = fixed_scene_;
      bindFixed();
    }

    /**
     * @brief demux the scene to gather the correct moving slice
     * @param[in] moving_scene_: the whole fixed scene (in tracking case use the local map scene)
     */
    virtual void setMoving(PropertyContainerBase* moving_scene_) {
      _moving_scene = moving_scene_;
      bindMoving();
    }

    /**
     * @brief propagates the transform to the inner modules
     * @param[in] moving_scene_: the whole fixed scene (in tracking case use the local map scene)
     */
    virtual void setMovingInFixed(const EstimateType& moving_in_fixed_) = 0;

    /**
     * @brief instanciate the factor, the moving and fixed scenes are propagated to the new
     * factor, if present
     */
    virtual void initializeFactor() = 0;

    /**
     * @brief generate a factor out of correspondences
     * @return the generated factor smart ptr
     */
    virtual FactorBasePtr factor() = 0;

    /**
     * @brief binds the robustifier if any
     */
    virtual void bindRobustifier() = 0;

    /**
     * @brief computes associations for current moving scene and fixed measurements
     * @return reference to computed associations for external use (e.g. pruning)
     */
    virtual CorrespondenceVector* computeCorrespondences() = 0;

    /**
     * @brief stores associations in an auxiliary data buffer that can be re-used during merge
     */
    virtual void storeCorrespondences() = 0;

    /**
     * @brief checks if correspondences are good enough
     * @return true if correspondences are good
     */
    virtual bool correspondencesGood() const = 0;

    /**
     * @brief binds the slice of the fixed scene to the fixed slice pointer in class
     */
    virtual void bindFixed() = 0;
    /**
     * @brief binds the slice of the moving scene to the moving slice pointer in class
     */
    virtual void bindMoving() = 0;

    /**
     * @brief auxiliary function for slice binding
     * @param[out] slice_: slice to be bound
     * @param[in] scene_: scene where to look for the slice
     * @param[in] slice_name_: name of the slice
     */
    template <typename SliceType_>
    void
    bindSlice(SliceType_*& slice_, PropertyContainerBase* scene_, const std::string& slice_name_);

    /**
     * @brief number of correspondences computed getter
     * @return size of the correspondence vector
     */
    virtual int numCorrespondences() const {
      return 0;
    }

    /**
     * @brief initializes the factor based on inherent values of the slice (e.g. camera matrix,
     * prior)
     * This method is called for all factors for each iteration
     */
    virtual void setupFactor() = 0;

    /**
     * @brief check if this aligner uses a prior factor
     * @return true if has prior factor
     */
    virtual bool isPrior() = 0;

    PropertyContainerBase* _fixed_scene =
      nullptr; /**< whole fixed scene (e.g. measurements for tracker)  */
    PropertyContainerBase* _moving_scene =
      nullptr; /**< whole moving scene (e.g. local map for tracker)  */

    bool _robustifier_changed_flag  = true; /**< change control flag for robustifiers */
    bool _fixed_slice_changed_flag  = true; /**< change control flag for fixed data slice */
    bool _moving_slice_changed_flag = true; /**< change control flag for moving data slice */

    AlignerType* _aligner = nullptr; /**< pointer to the aligner*/

  private:
    CorrespondenceVector _dummy_corr; /**< useless coords, to be purged */
  };

  template <typename VariableType_>
  using AlignerSliceProcessorBasePtr_ = std::shared_ptr<AlignerSliceProcessorBase_<VariableType_>>;

} // namespace srrg2_slam_interfaces
