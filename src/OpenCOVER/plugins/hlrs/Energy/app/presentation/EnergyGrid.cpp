#include "EnergyGrid.h"

#include <lib/core/constants.h>
#include <lib/core/simulation/grid.h>
#include <lib/core/utils/color.h>
#include <lib/core/utils/osgUtils.h>

#include <cassert>
#include <memory>
#include <osg/BoundingBox>
#include <osg/Shape>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgText/Text>
#include <sstream>
#include <utility>
#include <variant>

#include "TxtInfoboard.h"
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
  if (!m_enabled) {
    m_infoBoard->showInfo();
    m_enabled = true;
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
EnergyGrid::EnergyGrid(const EnergyGridConfig &data) : m_config(data) {
  if (!m_config.parent.valid()) {
    m_config.parent = new osg::Group;
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
      std::unique_ptr<grid::ConnectionData> data;

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
      data = std::make_unique<grid::ConnectionData>(name, from, to, radius, nullptr,
                                                    additionalData);
      m_connections.push_back(new grid::DirectedConnection(*data));
    }
  }
}

void EnergyGrid::findCorrectHeightForLine(float radius,
                                          osg::ref_ptr<grid::Line> line,
                                          grid::Lines &processedLines) {
  int redundantCount = 1;
  bool overlap = true;

  std::set<osg::ref_ptr<grid::Line>> lastMatch;
  while (overlap) {
    overlap = false;  // Reset overlap flag for each iteration

    for (auto otherLine : processedLines) {
      if (line == otherLine) continue;  // Skip comparing the line with itself.
      if (lastMatch.find(otherLine) != lastMatch.end()) continue;  // Skip already checked lines
      if (!line.valid() || !otherLine.valid()) continue;

        // const auto &lineBB(line->getBoundingBox());
        // const auto &otherLineBB(otherLine->getBoundingBox());

        // if (lineBB.intersects(otherLineBB)) {
        //   line->move(osg::Vec3(0, 0, -2 * radius * redundantCount));
        //   line->recomputeBoundingBox();
        //   std::cerr << "redundant line: " << line->getName() << " " <<
        //   redundantCount
        //             << "\n";
        //   ++redundantCount;
        //   overlap = true;  // Set overlap flag to repeat the loop
        // lastMatch.insert(otherLine);  // Store the last line to avoid redundant checks
        //   break;           // No need to check other lines in this iteration
        // }

      if (line->overlap(*otherLine)) {
        line->move(osg::Vec3(0, 0, -2 * radius * redundantCount));
        std::cerr << "redundant line: " << line->getName() << " " << redundantCount
                  << "\n";
        ++redundantCount;
        overlap = true;  // Set overlap flag to repeat the loop
        lastMatch.insert(otherLine);  // Store the last line to avoid redundant checks
        break;           // No need to check other lines in this iteration
      }
    }
  }
}

void EnergyGrid::initDrawableLines() {
  using namespace core::simulation::grid;
  osg::ref_ptr<osg::Group> lines = new osg::Group;
  lines->setName("Lines");
//   const auto &radius = m_config.connectionRadius;
  const auto &radius = m_config.lines[0]->getConnections().begin()->second->getStart()->getRadius();
  grid::Lines overlappingLines;

  for (auto line : m_config.lines) {
    // move redundant line below the first one
    findCorrectHeightForLine(radius, line, overlappingLines);
    overlappingLines.push_back(line);
    // auto bbVis = core::utils::osgUtils::createBoundingBoxVisualization(
    //     line->getBoundingBox());
    // bbVis->setName(line->getName() + "_bb");
    // line->addChild(bbVis);
    // m_config.parent->addChild(bbVis);
    m_drawables.push_back(line);
    lines->addChild(line);
    std::string toPrint = "";
    // for (const auto &[name, data] : line->getAdditionalData()) {
    //   toPrint +=
    //       UIConstants::TAB_SPACES + name + ": " + std::visit(get_string, data);
    // }
    auto center = line->getCenter();
    center.z() += 30;
    auto name = line->getName();

    m_config.infoboardAttributes.position = center;
    m_config.infoboardAttributes.title = name;
    TxtInfoboard infoboard(m_config.infoboardAttributes);
    m_infoboards.push_back(std::make_unique<InfoboardSensor>(
        line, std::make_unique<TxtInfoboard>(infoboard), toPrint));
  }
  m_config.parent->addChild(lines);
}

void EnergyGrid::initDrawablePoints() {
  osg::ref_ptr<osg::Group> points = new osg::Group;
  points->setName("Points");
  for (auto &point : m_config.points) {
    m_drawables.push_back(point);
    points->addChild(point);
    std::string toPrint = "";
    for (const auto &[name, data] : point->getAdditionalData()) {
      toPrint +=
          UIConstants::TAB_SPACES + name + ": " + std::visit(get_string, data);
    }
    auto center = point->getPosition();
    center.z() += 30;
    auto name = point->getName();

    m_config.infoboardAttributes.position = center;
    m_config.infoboardAttributes.title = name;
    TxtInfoboard infoboard(m_config.infoboardAttributes);
    m_infoboards.push_back(std::make_unique<InfoboardSensor>(
        point, std::make_unique<TxtInfoboard>(infoboard), toPrint));
  }
  m_config.parent->addChild(points);
}

osg::ref_ptr<grid::DirectedConnection> EnergyGrid::getConnectionByName(
    const std::string &name) {
  for (auto &connection : m_connections)
    if (connection->getName() == name) return connection;
  return nullptr;
}

osg::ref_ptr<grid::Point> EnergyGrid::getPointByName(const std::string &name) {
  for (auto &point : m_config.points)
    if (point->getName() == name) return point;
  return nullptr;
}

void EnergyGrid::initDrawableConnections() {
  osg::ref_ptr<osg::Group> connections = new osg::Group;
  connections->setName("Connections");

  for (auto connection : m_connections) {
    m_drawables.push_back(connection);
    connections->addChild(connection);

    std::string toPrint = "";
    for (const auto &[name, data] : connection->getAdditionalData()) {
      toPrint +=
          UIConstants::TAB_SPACES + name + ": " + std::visit(get_string, data);
    }
    auto center = connection->getCenter();
    center.z() += 30;
    auto name = connection->getName();

    m_config.infoboardAttributes.position = center;
    m_config.infoboardAttributes.title = name;
    TxtInfoboard infoboard(m_config.infoboardAttributes);
    m_infoboards.push_back(std::make_unique<InfoboardSensor>(
        connection, std::make_unique<TxtInfoboard>(infoboard), toPrint));
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
}

void EnergyGrid::updateDrawables() {
  for (auto &infoboard : m_infoboards) {
    infoboard->updateDrawable();
  }
}
