#include "EnergyGrid.h"

#include <cover/coVRSelectionManager.h>
#include <lib/core/constants.h>
#include <lib/core/simulation/grid.h>
#include <lib/core/utils/color.h>
#include <lib/core/utils/osgUtils.h>

#include <cassert>
#include <memory>
#include <osg/BoundingBox>
#include <osg/MatrixTransform>
#include <osg/Shape>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgText/Text>
#include <sstream>
#include <utility>
#include <variant>

#include "cover/VRViewer.h"

namespace {

auto get_string = [](const auto &data) {
  std::stringstream ss;
  ss << data << "\n\n";
  return ss.str();
};

}  // namespace

InfoboardSensor::InfoboardSensor(
    osg::ref_ptr<osg::Group> parent,
    std::unique_ptr<interface::IInfoboard<std::string>> &&infoboard,
    const std::string &content)
    : coPickSensor(parent), m_enabled(false), m_infoBoard(std::move(infoboard)) {
  m_infoBoard->initInfoboard();
  m_infoBoard->initDrawable();
  m_infoBoard->updateInfo(content);

  parent->addChild(m_infoBoard->getDrawable());
}

void InfoboardSensor::activate() {
  auto selectionManager = opencover::coVRSelectionManager::instance();
  selectionManager->clearSelection();
  auto selectedNode = getNode();
  if (!selectedNode) {
    std::cerr << "InfoboardSensor: No node selected for activation." << std::endl;
    return;
  }
  auto parent = selectedNode->getParent(0);
  if (!parent) {
    std::cerr << "InfoboardSensor: No parent node found for selected node."
              << std::endl;
    return;
  }

  constexpr float R = 173.0f / 255.0f;
  constexpr float G = 216.0f / 255.0f;
  constexpr float B = 230.0f / 255.0f;
  selectionManager->setSelectionColor(R, G, B);

  if (!m_enabled) {
    m_infoBoard->showInfo();
    m_enabled = true;
    selectionManager->addSelection(parent, getNode());
  } else {
    m_infoBoard->hideInfo();
    m_enabled = false;
  }

  coPickSensor::activate();
}

void InfoboardSensor::update() {
  updateDrawable();
  coPickSensor::update();
}

// EnergyGrid::EnergyGrid(EnergyGridConfig &&data) : m_config(std::move(data)) {
EnergyGrid::EnergyGrid(const EnergyGridConfig &data, bool ignoreOverlap)
    : m_config(data), m_ignoreOverlap(ignoreOverlap) {
  if (!m_config.parent.valid()) {
    m_config.parent = new osg::MatrixTransform;
    m_config.parent->setName(m_config.name);
  }
  initConnections();
};

void EnergyGrid::initConnections() {
  assert(m_config.valid() && "EnergyGridConfig is not valid");
  if (m_config.connectionType == EnergyGridConnectionType::Index)
    initConnectionsByIndex(m_config.indices, m_config.connectionRadius,
                           m_config.additionalConnectionData);
}

void EnergyGrid::initConnectionsByIndex(
    const grid::Indices &indices, const float &radius,
    const grid::ConnectionDataList &additionalConnectionData) {
  bool hasAdditionalData = !additionalConnectionData.empty();

  const auto &points = m_config.points;
  for (auto i = 0; i < indices.size(); ++i) {
    auto from = points[i];
    for (auto j = 0; j < indices[i].size(); ++j) {

      if (i < 0 || i >= points.size()) {
        std::cerr << "Invalid Index for points: " << i << "\n";
        continue;
      }

      const auto indice = indices[i][j];

      if (indice >= points.size() || indice < 0) {
        std::cerr << "Invalid Index for points: " << indice << "\n";
        continue;
      }
      auto to = points[indice];

      std::string name(from->getName() + " " + UIConstants::RIGHT_ARROW_UNICODE_HEX +
                       " " + to->getName());

      grid::Data additionalData{};
      if (hasAdditionalData)
        if (additionalConnectionData.size() > i)
          if (additionalConnectionData[i].size() > j)
            additionalData = additionalConnectionData[i][j];
      grid::ConnectionData data{name, from, to, radius, nullptr,
                                                    additionalData};
      m_connections.push_back(new grid::DirectedConnection(data));
    }
  }
}

void EnergyGrid::findCorrectHeightForLine(float radius,
                                          osg::ref_ptr<grid::Line> line,
                                          grid::Lines &processedLines) {
  if (m_ignoreOverlap) return;
  int redundantCount = 1;
  bool overlap = true;

  std::set<osg::ref_ptr<grid::Line>> lastMatch;
  while (overlap) {
    overlap = false;  // Reset overlap flag for each iteration

    for (auto otherLine : processedLines) {
      if (line == otherLine) continue;  // Skip comparing the line with itself.
      if (lastMatch.find(otherLine) != lastMatch.end())
        continue;  // Skip already checked line
      if (!line.valid() || !otherLine.valid()) continue;
      if (line->overlap(*otherLine)) {
        line->move(osg::Vec3(0, 0, -2 * radius * redundantCount));
        ++redundantCount;
        overlap = true;  // Set overlap flag to repeat the loop
        lastMatch.insert(
            otherLine);  // Store the last line to avoid redundant checks
        break;           // No need to check other lines in this iteration
      }
    }
  }
}

void EnergyGrid::initDrawableLines() {
  using namespace core::simulation::grid;
  osg::ref_ptr<osg::Group> lines = new osg::Group;
  lines->setName("Lines");
  const auto &sphereRadius =
      m_config.lines[0]->getConnections().begin()->second->getStart()->getRadius();
  grid::Lines overlappingLines;

  for (auto line : m_config.lines) {
    // move redundant line below the first one
    findCorrectHeightForLine(sphereRadius, line, overlappingLines);
    overlappingLines.push_back(line);
    initDrawableGridObject(lines, line);
  }
  m_config.parent->addChild(lines);
}

std::string EnergyGrid::createDataString(const grid::Data &data) const {
  std::string result;
  for (const auto &[key, value] : data) {
    result += UIConstants::TAB_SPACES + key + ": " + std::visit(get_string, value);
  }
  return result;
}

void EnergyGrid::initDrawablePoints() {
  osg::ref_ptr<osg::Group> points = new osg::Group;
  points->setName("Points");
  if (!m_config.points.empty()) {
    for (auto &point : m_config.points) {
      initDrawableGridObject(points, point);
    }
  } else {
    for (auto &[id, point] : m_config.pointsMap) {
      if (point.valid()) {
        initDrawableGridObject(points, point);
      }
    }
  }

  for (auto &[id, point] : m_config.pointsMap) {
    if (point.valid()) {
      initDrawableGridObject(points, point);
    }
  }

  m_config.parent->addChild(points);
}

osg::ref_ptr<grid::DirectedConnection> EnergyGrid::getConnectionByName(
    const std::string &name) {
  for (auto &connection : m_connections)
    if (connection->getName() == name) return connection;
  return nullptr;
}

const osg::ref_ptr<grid::Point> EnergyGrid::getPointByName(
    const std::string &name) const {
  for (auto &point : m_config.points)
    if (point->getName() == name) return point;
  return nullptr;
}

void EnergyGrid::initDrawableConnections() {
  osg::ref_ptr<osg::Group> connections = new osg::Group;
  connections->setName("Connections");

  for (auto connection : m_connections) {
    initDrawableGridObject(connections, connection);
  }

  m_config.parent->addChild(connections);
}

void EnergyGrid::initDrawables() {
  initDrawablePoints();
  switch (m_config.connectionType) {
    case EnergyGridConnectionType::Index:
      initDrawableConnections();
      break;
    case EnergyGridConnectionType::Line:
      initDrawableLines();
      break;
    default:
      std::cerr << "Invalid connection type\n";
  }
}

void EnergyGrid::updateColor(const osg::Vec4 &color) {
  for (auto &connection : m_connections) {
    utils::color::overrideGeodeColor(connection->getGeode(), color);
  }
  for (auto &point : m_config.points) {
    utils::color::overrideGeodeColor(point->getGeode(), color);
  }
  for (auto &line : m_config.lines) {
    for (auto &[name, connection] : line->getConnections()) {
      utils::color::overrideGeodeColor(connection->getGeode(), color);
    }
    // utils::color::overrideGeodeColor(line->getGeode(), color);
  }
}

void EnergyGrid::updateDrawables() {
  for (auto &infoboard : m_infoboards) {
    infoboard->updateDrawable();
  }
}
//toDo: streamline update for m_connections, m_lines and m_config.lines
void EnergyGrid::updateTime(int timestep)
{
  for(auto &conn : m_connections)
    conn->updateTimestep(timestep);
  for(auto &line : m_lines)
    for(auto &[_, conn] : line->getConnections())
      conn->updateTimestep(timestep);
  for(auto &line : m_config.lines)
    for(auto &[_, conn] : line->getConnections())
      conn->updateTimestep(timestep);
}



void EnergyGrid::setColorMap(const opencover::ColorMap &colorMap)
{
  for(auto &conn : m_connections)
    conn->setColorMap(colorMap);
  for(auto &line : m_lines)
    for(auto &[_, conn] : line->getConnections())
      conn->setColorMap(colorMap);
  for(auto &line : m_config.lines)
    for(auto &[_, conn] : line->getConnections())
      conn->setColorMap(colorMap);
}

void EnergyGrid::setData(const core::simulation::Simulation& sim, const std::string & species) {
  for(auto &conn : m_connections) {
    
    auto fromData = sim.getTimedependentScalar(species, conn->getStart()->getName());
    auto toData = sim.getTimedependentScalar(species, conn->getEnd()->getName());
    if (fromData && toData) {
      conn->setData(*fromData, *toData);
    } else {
      std::cerr << "No data found for connection: " << conn->getName() << "\n";
    }
  }
  for(auto &line : m_lines) {
    for(auto &[_, conn] : line->getConnections()) {
      
      auto fromData = sim.getTimedependentScalar(species, conn->getStart()->getName());
      auto toData = sim.getTimedependentScalar(species, conn->getEnd()->getName());
      if (fromData && toData) {
        conn->setData(*fromData, *toData);
      } else {
        std::cerr << "No data found for connection: " << conn->getName() << "\n";
      }
    }
  }
  for(auto &line : m_config.lines) {
    for(auto &[_, conn] : line->getConnections()) {
      
      auto fromData = sim.getTimedependentScalar(species, conn->getStart()->getName());
      auto toData = sim.getTimedependentScalar(species, conn->getEnd()->getName());
      if (fromData && toData) {
        conn->setData(*fromData, *toData);
      } else {
        std::cerr << "No data found for connection: " << conn->getName() << "\n";
      }
    }
  }


}