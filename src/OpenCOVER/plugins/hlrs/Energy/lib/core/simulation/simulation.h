#pragma once

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "object.h"

namespace core::simulation {
struct UnitPair {
  std::vector<std::string> names;
  std::string unit;
};

class UnitMap {
 public:
  UnitMap(std::vector<UnitPair> &&unitPairs) {
    for (const auto &pair : unitPairs) {
      for (const auto &name : pair.names) {
        unit_map[name] = pair.unit;
      }
    }
  }

  const std::string operator[](const std::string &key) const {
    auto it = unit_map.find(key);
    if (it != unit_map.end()) {
      return it->second;
    }
    return "unknown";
  }

  auto begin() { return unit_map.begin(); }
  auto end() { return unit_map.end(); }

 private:
  std::unordered_map<std::string, std::string> unit_map;
};

const UnitMap UNIT_MAP = UnitMap({
    {{"kWh", "leistung", "power"}, "kWh"},
    {{"kW"}, "kW"},
    {{"q_dem_w", "waermestromdichte"}, "W/m2"},
    {{"delta_q", "aenderung_stromdichte"}, "W/m2"},
    {{"mass_flow", "massenstrom"}, "kg/s"},
    {{"celcius", "temp", "inlet_temp", "outlet_temp"}, "°C"},
    {{"electricity_selling_price"}, "Cent/kWh"},
    {{"heating_cost"}, "€"},
});

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
  virtual const std::vector<double> *getTimedependentScalar(const std::string &species, const std::string& node) const = 0;
  
  protected:
  template <typename T>
  void computeParameter(const ObjectContainer<T> &baseMap) {
    static_assert(std::is_base_of_v<Object, T>,
                  "T must be derived from core::simulation::Object");
    for (const auto &[_, base] : baseMap) {
      const auto &data = base.getData();
      for (const auto &[key, values] : data) {
        setUnit(key);
        computeMinMax(key, values);
        computeMaxTimestep(key, values);
        m_scalarProperties[key].species = key;
      }
    }
  }

  template <typename T>
  const std::vector<double> *getTimedependentScalar(const ObjectContainer<T> &baseMap,
                                          const std::string &species,
                                          const std::string &node) const{
    static_assert(std::is_base_of_v<Object, T>,
                  "T must be derived from core::simulation::Object");
    std::vector<double> result;
    for (const auto &[name, base] : baseMap) {
      if (base.getName() == node) {
        const auto &data = base.getData();
        if (data.find(species) != data.end()) {
          return &data.at(species);
        }
      }
    }
    return nullptr;
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
