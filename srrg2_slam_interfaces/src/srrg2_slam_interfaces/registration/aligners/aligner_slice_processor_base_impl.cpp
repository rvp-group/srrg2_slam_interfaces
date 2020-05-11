#include "aligner_slice_processor_base.h"
#include "srrg_config/property_configurable.h"
#include "srrg_property/property_identifiable.h"

namespace srrg2_slam_interfaces {

  template <typename VariableType_>
  template <typename SliceType_>
  void AlignerSliceProcessorBase_<VariableType_>::bindSlice(SliceType_*& slice_,
                                                            PropertyContainerBase* scene_,
                                                            const std::string& slice_name_) {
    if (!scene_) {
      std::cerr << "AlignerSliceBase_::bindSlice()|"
                << FG_RED("error while binding " << slice_name_) << std::endl;
      throw std::runtime_error("AlignerSliceBase_::bindSlice()| null scene");
    }
    PropertyBase* p_base = scene_->property(slice_name_);
    if (!p_base) {
      std::cerr << "property name: " << slice_name_ << std::endl;
      std::cerr << "num properties in fixed: " << scene_->properties().size() << std::endl;
      std::cerr << "properties currently in fixed: " << std::endl;
      for (auto it : scene_->properties()) {
        std::cerr << it.first << std::endl;
      }
      throw std::runtime_error("AlignerSliceBase_::bindSlice()| no property scene");
    }
    // 1 the property might be a plain object, so we try to fetch it as such
    {
      Property_<SliceType_*>* p_fixed = dynamic_cast<Property_<SliceType_*>*>(p_base);
      if (p_fixed) {
        slice_ = p_fixed->value();
      }
    }

    // 2 the property might be stored brutally, we get the pointer
    if (!slice_) {
      Property_<SliceType_>* p_fixed = dynamic_cast<Property_<SliceType_>*>(p_base);
      if (p_fixed) {
        slice_ = &p_fixed->value();
      }
    }

    // 3 the property might be a shared pointer, we get the raw pointer
    if (!slice_) {
      Property_<std::shared_ptr<SliceType_>>* p_fixed =
        dynamic_cast<Property_<std::shared_ptr<SliceType_>>*>(p_base);
      if (p_fixed) {
        slice_ = p_fixed->value().get();
      }
    }
  }

} // namespace srrg2_slam_interfaces
