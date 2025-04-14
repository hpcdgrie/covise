#include "SolarPanel.h"

#include <lib/core/utils/color.h>
#include <lib/core/utils/osgUtils.h>

#include <osg/Geode>
#include <osg/ref_ptr>

using namespace core::utils::osgUtils;

void SolarPanel::init() { initDrawables(); }

void SolarPanel::initDrawables() { m_drawables.push_back(m_node); }

void SolarPanel::updateDrawables() {}

void SolarPanel::updateColor(const osg::Vec4 &colorT) {
  static uint r = 255;
  static uint g = 0;
  static uint b = 0;

  if (r == 255 && g < 255 && b == 0) {
    g++;
  } else if (r > 0 && g == 255 && b == 0) {
    r--;
  } else if (r == 0 && g == 255 && b < 255) {
    b++;
  } else if (r == 0 && g > 0 && b == 255) {
    g--;
  } else if (r < 255 && g == 0 && b == 255) {
    r++;
  } else if (r == 255 && g == 0 && b > 0) {
    b--;
  }
  auto color = osg::Vec4(r / 255.0, g / 255.0, b / 255.0, 1.0);
  for (auto &node : m_drawables) {
    auto geode = dynamic_cast<osg::Geode *>(node.get());
    if (geode) {
      core::utils::color::overrideGeodeColor(geode, color);
      continue;
    }

    auto group = dynamic_cast<osg::Group *>(node.get());
    if (group) {
      auto geodes = getGeodes(group);
      for (auto &geode : *geodes) {
        core::utils::color::overrideGeodeColor(geode, color);
      }
    }
  }
}
