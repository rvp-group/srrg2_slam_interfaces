#include "local_map_selector_user_defined.h"

namespace srrg2_slam_interfaces {
  template <typename SLAMAlgorithmType_>
  void LocalMapSelectorUserDefined_<SLAMAlgorithmType_>::compute() {
    if (!ThisType::_slam) {
      throw std::runtime_error("LocalMapSelectorBreadthFirst_::compute| no slam selected");
    }
    ThisType::_hints = ThisType::_slam->closureCandidates();
  }
} // namespace srrg2_slam_interfaces
