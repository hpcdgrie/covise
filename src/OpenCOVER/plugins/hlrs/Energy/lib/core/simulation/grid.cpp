#include "grid.h"

#include <utils/osgUtils.h>

#include <algorithm>
#include <osg/MatrixTransform>
#include <osg/Shape>

namespace core::simulation::grid {
Point::Point(const std::string &name, const float &x, const float &y, const float &z,
             const float &radius, const Data &additionalData)
    : osg::MatrixTransform(),
      m_point(new osg::Sphere(osg::Vec3(x, y, z), radius)),
      m_additionalData(additionalData) {
  init(name);
}

Point::Point(const Point &other) {
  m_point = new osg::Sphere(other.m_point->getCenter(), other.m_point->getRadius());
  m_shape = new osg::ShapeDrawable(m_point, other.m_shape->getTessellationHints());
  m_additionalData = other.m_additionalData;
  init(other.getName());
}

void Point::init(const std::string &name) {
  osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints;
  hints->setDetailRatio(1.5f);
  m_shape = new osg::ShapeDrawable(m_point, hints);
  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addChild(m_shape);
  addChild(geode);
  setName(name);
}

DirectedConnection::DirectedConnection(const std::string &name,
                                       const osg::Vec3 &start, const osg::Vec3 &end,
                                       const float &radius,
                                       osg::ref_ptr<osg::TessellationHints> hints,
                                       const Data &additionalData,
                                       ConnectionType type)
    : osg::MatrixTransform(),
      m_start(new osg::Vec3(start)),
      m_end(new osg::Vec3(end)),
      m_additionalData(additionalData) {
  switch (type) {
    case ConnectionType::Line:
      m_geode = utils::osgUtils::createCylinderBetweenPoints(
          start, end, radius, osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f), hints);
      break;
    case ConnectionType::Arc:
      m_geode = utils::osgUtils::createBezierTube(
          start, end, radius * 2.0f, radius, 50, osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f));
      break;
    case ConnectionType::Arrow:
      assert(false && "Arrow type not implemented");
  }
  addChild(m_geode);
  setName(name);
}

Line::Line(std::string name, const Connections &connections) : m_name(name) {
  init(connections);
}

void Line::init(const std::vector<osg::ref_ptr<DirectedConnection>> &connections) {
  if (connections.empty()) {
    std::cerr << "Line: No connections provided\n";
    return;
  }
  setName(m_name);
  // TODO: for now only the first connection is used to get the additional data
  m_additionalData = connections[0]->getAdditionalData();
  for (const auto &connection : connections) {
    const auto &name = connection->getName();
    m_connections[name] = connection;
    addChild(connection);
  }
}

bool Line::operator==(const Line &other) const {
  return m_name == other.m_name ||
         std::any_of(m_connections.begin(), m_connections.end(),
                     [&other](const auto &pair) {
                       return other.m_connections.find(pair.first) !=
                              other.m_connections.end();
                     });
}

}  // namespace core::simulation::grid
