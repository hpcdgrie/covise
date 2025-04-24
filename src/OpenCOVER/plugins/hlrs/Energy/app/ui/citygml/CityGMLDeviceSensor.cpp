#include "CityGMLDeviceSensor.h"

#include <PluginUtil/colors/coColorMap.h>

#include <PluginUtil/coSensor.h>
#include <PluginUtil/coShaderUtil.h>
#include <app/presentation/CityGMLBuilding.h>

#include <memory>
#include <osg/Geometry>

CityGMLDeviceSensor::CityGMLDeviceSensor(
    osg::ref_ptr<osg::Group> parent,
    std::unique_ptr<core::interface::IInfoboard<std::string>> &&infoBoard,
    std::unique_ptr<core::interface::IBuilding> &&drawableBuilding)
    : coPickSensor(parent),
      m_cityGMLBuilding(std::move(drawableBuilding)),
      m_infoBoard(std::move(infoBoard))
{
  m_cityGMLBuilding->initDrawables();

  // infoboard
  m_infoBoard->initInfoboard();
  m_infoBoard->initDrawable();
  parent->addChild(m_infoBoard->getDrawable());
}

CityGMLDeviceSensor::~CityGMLDeviceSensor() {
  if (m_active) disactivate();
  getParent()->removeChild(m_infoBoard->getDrawable());
}

void CityGMLDeviceSensor::update() {
  m_cityGMLBuilding->updateDrawables();
  m_infoBoard->updateDrawable();
  coPickSensor::update();
}

void CityGMLDeviceSensor::activate() {
  if (!m_active) {
    m_infoBoard->updateInfo("DAS IST EIN TEST");
    m_infoBoard->showInfo();
  }
  m_active = !m_active;
}

void CityGMLDeviceSensor::disactivate() {
  if (m_active) return;
  m_infoBoard->hideInfo();
}

void CityGMLDeviceSensor::updateTimestepColors(const std::vector<float> &values, const opencover::ColorMap &map) {
  m_colors.clear();
  m_colors.resize(values.size());
  for (auto i = 0; i < m_colors.size(); ++i)
    m_colors[i] = getColor(values[i], map);
}

void CityGMLDeviceSensor::updateTime(int timestep) {
  if (timestep >= m_colors.size()) return;
  m_cityGMLBuilding->updateColor(m_colors[timestep]);
  m_cityGMLBuilding->updateTime(timestep);
  m_infoBoard->updateTime(timestep);
}
