#pragma once

#include <lib/core/simulation/heating.h>
#include <lib/core/utils/color.h>

#include <iostream>
#include <memory>
#include <osg/Vec4>

#include "app/presentation/EnergyGrid.h"
#include "app/ui/simulation/BaseSimulationUI.h"

using namespace core::simulation::heating;

template <typename T>
class HeatingSimulationUI : public BaseSimulationUI<T> {
 public:
  HeatingSimulationUI(std::shared_ptr<HeatingSimulation> sim, std::shared_ptr<T> parent)
      : BaseSimulationUI<T>(sim, parent) {}
  ~HeatingSimulationUI() = default;
  HeatingSimulationUI(const HeatingSimulationUI &) = delete;
  HeatingSimulationUI &operator=(const HeatingSimulationUI &) = delete;

  void updateTime(int timestep) override {
    auto parent = this->m_parent.lock();
    if (!parent) return;
    // TODO: rethink this pls => maybe use a visitor pattern
    std::shared_ptr<EnergyGrid> energyGrid =
        std::dynamic_pointer_cast<EnergyGrid>(parent);
    if (energyGrid) {
      auto heatingSim = this->heatingSimulationPtr();
      if (!heatingSim) return;
      auto updateEnergyGridColorsForContainer = [&](auto entities) {
        this->updateEnergyGridColors(timestep, energyGrid, entities);
      };
      updateEnergyGridColorsForContainer(heatingSim->Consumers());
      updateEnergyGridColorsForContainer(heatingSim->Producers());
    }
  }

  opencover::ColorMap updateTimestepColors(const opencover::ColorMap& map, bool resetMinMax = false) override {

    auto m = map;

    if (m.min > m.max) m.min = m.max;

    if (resetMinMax) {
      m.min = heatingSimulationPtr()->getMin(map.species);
      m.max = heatingSimulationPtr()->getMax(map.species);
    }

    // compute colors
    auto heatingSim = this->heatingSimulationPtr();
    if (!heatingSim) return m;
    auto computeColorsForContainer = [&](auto entities) {
      this->computeColors(m, entities);
    };

    computeColorsForContainer(heatingSim->Consumers().get());
    computeColorsForContainer(heatingSim->Producers().get());
    return m;
  }

 private:
  std::shared_ptr<HeatingSimulation> heatingSimulationPtr() {
    return std::dynamic_pointer_cast<HeatingSimulation>(this->m_simulation.lock());
  }
};
