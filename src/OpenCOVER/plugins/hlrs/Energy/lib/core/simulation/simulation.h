#pragma once

#include <map>
#include <string>
#include <vector>

#include "object.h"

namespace core::simulation {

const std::map<std::vector<std::string>, std::string> UNIT_MAP = {
    {{"kWh", "leistung", "power"}, "kWh"},
    {{"kW"}, "kW"},
    {{"q_dem_w", "waermestromdichte"}, "W/m2"},
    {{"delta_q", "aenderung_stromdichte"}, "W/m2"},
    {{"mass_flow", "massenstrom"}, "kg/s"},
    {{"celcius", "temp", "inlet_temp", "outlet_temp"}, "°C"},
    {{"elctricity_selling_price"}, "Cent/kWh"},
    {{"heatingCost"}, "€"},
};

struct ScalarProperty {
  std::string unit;
  std::string species;
  double min;
  double max;
  size_t timesteps;
};

typedef std::map<std::string, ScalarProperty> ScalarProperties;

class Simulation {
 public:
  Simulation() = default;

  void addData(const std::string &key, const std::vector<double> &value) {
    m_data[key] = value;
  }

  void addData(const std::string &key, const double &value) {
    m_data[key].push_back(value);
  }

  auto &getData() { return m_data; }
  const auto &getMinMax(const std::string &key) {
    return m_scalarProperties.at(key).min;
  }
  const auto &getMax(const std::string &key) const {
    return m_scalarProperties.at(key).max;
  }
  const auto &getMin(const std::string &key) const {
    return m_scalarProperties.at(key).min;
  }
  auto getMinMax(const std::string &key) const {
    return std::make_pair(getMin(key), getMax(key));
  }
  const auto &getTimesteps(const std::string &key) const {
    return m_scalarProperties.at(key).timesteps;
  }
  const auto &getSpecies(const std::string &key) const {
    return m_scalarProperties.at(key).species;
  }
  const auto &getUnit(const std::string &key) const {
    return m_scalarProperties.at(key).unit;
  }
  const auto &getScalarProperties() const { return m_scalarProperties; }
  auto &getScalarProperties() { return m_scalarProperties; }

  virtual void computeParameters() {};

 protected:
  template <typename T>
  void computeParameter(const ObjectContainer<T> &baseMap) {
    static_assert(std::is_base_of_v<Object, T>,
                  "T must be derived from core::simulation::Object");
    for (const auto &[_, base] : baseMap.get()) {
      const auto &data = base.getData();
      for (const auto &[key, values] : data) {
        setUnit(key);
        computeMinMax(key, values);
        computeMaxTimestep(key, values);
        m_scalarProperties[key].species = key;
      }
    }
  }

  virtual void computeMinMax(const std::string &key,
                             const std::vector<double> &values);
  virtual void computeMaxTimestep(const std::string &key,
                                  const std::vector<double> &values);
  virtual void setUnit(const std::string &key);

  ScalarProperties m_scalarProperties;
  // general meta data for the simulation
  Data m_data;
};
}  // namespace core::simulation
