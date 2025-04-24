#pragma once

#include <lib/core/simulation/power.h>

#include "app/ui/simulation/BaseSimulationUI.h"
#include "app/presentation/EnergyGrid.h"

using namespace core::simulation::power;

template <typename T>
class PowerSimulationUI : public BaseSimulationUI<T> {
 public:
  PowerSimulationUI(std::shared_ptr<PowerSimulation> sim, std::shared_ptr<T> parent, const opencover::ColorMap &colorMap)
      : BaseSimulationUI<T>(sim, parent, colorMap) {}
  ~PowerSimulationUI() = default;
  PowerSimulationUI(const PowerSimulationUI &) = delete;
  PowerSimulationUI &operator=(const PowerSimulationUI &) = delete;

  void updateTime(int timestep) override {
    auto parent = this->m_parent.lock();
    if (!parent) return;
    // TODO: rethink this pls => maybe use a visitor pattern
    std::shared_ptr<EnergyGrid> energyGrid =
        std::dynamic_pointer_cast<EnergyGrid>(parent);
    if (energyGrid) {
      auto powerSim = this->powerSimulationPtr();
      if (!powerSim) return;
      auto updateEnergyGridColorsForContainer = [&](auto entities) {
        this->updateEnergyGridColors(timestep, energyGrid, entities);
      };
      updateEnergyGridColorsForContainer(powerSim->Buses());
      updateEnergyGridColorsForContainer(powerSim->Generators());
      updateEnergyGridColorsForContainer(powerSim->Transformators());
    }
  }

  opencover::ColorMap updateTimestepColors(const opencover::ColorMap& map, bool resetMinMax = false) override {

    auto m = map;
    if (m.min > m.max) m.min = m.max;

    if (resetMinMax) {
      auto &[res_min, res_max] = powerSimulationPtr()->getMinMax(m.species);
      m.max = res_max;
      m.min = res_min;
    }

    // compute colors
    auto powerSim = this->powerSimulationPtr();
    if (!powerSim) return m;
    auto computeColorsForContainer = [&](auto container) {
      this->computeColors(m, container);
    };

    computeColorsForContainer(powerSim->Buses().get());
    computeColorsForContainer(powerSim->Generators().get());
    computeColorsForContainer(powerSim->Transformators().get());
    return m;
  }

 private:
  std::shared_ptr<PowerSimulation> powerSimulationPtr() {
    return std::dynamic_pointer_cast<PowerSimulation>(this->m_simulation.lock());
  }
};
