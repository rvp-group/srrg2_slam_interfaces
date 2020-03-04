#include "local_map_selector_user_defined.h"

namespace srrg2_slam_interfaces {
  template <typename SLAMAlgorithmType_>
  void LocalMapSelectorUserDefined_<SLAMAlgorithmType_>::compute() {
    if (!this->_slam) {
      throw std::runtime_error("LocalMapSelectorBreadthFirst_::compute| no slam selected");
    }
    this->_hints = this->_slam->closureCandidates();
  }
} // namespace srrg2_slam_interfaces
