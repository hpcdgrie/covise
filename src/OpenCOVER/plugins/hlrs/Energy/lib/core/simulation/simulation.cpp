#include "simulation.h"

#include <algorithm>

namespace core::simulation {

void Simulation::computeMinMax(const std::string &key,
                               const std::vector<double> &values) {
  const auto &[min_elem, max_elem] =
      std::minmax_element(values.begin(), values.end());
  if (auto it = m_scalarProperties.find(key); it == m_scalarProperties.end()) {
    auto &property = m_scalarProperties[key];
    property.min = *min_elem;
    property.max = *max_elem;
  } else {
    auto &property = it->second;
    if (*min_elem < property.min) property.min = *min_elem;
    if (*max_elem > property.max) property.max = *max_elem;
  }
}

void Simulation::computeMaxTimestep(const std::string &key,
                                    const std::vector<double> &values) {
  m_scalarProperties[key].timesteps = values.size();
}

void Simulation::setUnit(const std::string &key) {
  m_scalarProperties[key].unit = UNIT_MAP[key];
}
}  // namespace core::simulation
