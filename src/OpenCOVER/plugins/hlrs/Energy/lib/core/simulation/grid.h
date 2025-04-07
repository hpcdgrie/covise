#pragma once

#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Vec3>
#include <variant>

#include "../utils/color.h"

namespace core::simulation::grid {
// grid::Data is a map of std::string and variants that can hold int, float, or
// string
typedef std::map<std::string, std::variant<float, int, std::string>> Data;
class Point : public osg::MatrixTransform {
 public:
  Point(const std::string &name, const float &x, const float &y, const float &z,
        const float &radius, const Data &additionalData = Data());
  Point(const Point &p);

  const auto &getPosition() const { return m_point->getCenter(); }
  const auto &getAdditionalData() const { return m_additionalData; }
  osg::ref_ptr<osg::Geode> getGeode() {
    return dynamic_cast<osg::Geode *>(osg::Group::getChild(0));
  }

  void updateColor(const osg::Vec4 &color) {
    core::utils::color::overrideGeodeColor(getGeode(), color);
  }

 private:
  void init(const std::string &name);
  osg::ref_ptr<osg::ShapeDrawable> m_shape;
  osg::ref_ptr<osg::Sphere> m_point;
  Data m_additionalData;
};

template <typename CoordType>
struct ConnectionData {
  ConnectionData(const std::string &name, const CoordType &start,
                 const CoordType &end, const float &radius,
                 osg::ref_ptr<osg::TessellationHints> hints = nullptr,
                 const Data &additionalData = Data())
      : name(name),
        start(start),
        end(end),
        radius(radius),
        hints(hints),
        additionalData(additionalData) {};
  std::string name;
  CoordType start;
  CoordType end;
  float radius;
  osg::ref_ptr<osg::TessellationHints> hints;
  Data additionalData;
};

enum class ConnectionType { Line, Arc, Arrow };

class DirectedConnection : public osg::MatrixTransform {
  DirectedConnection(const std::string &name, const osg::Vec3 &start,
                     const osg::Vec3 &end, const float &radius,
                     osg::ref_ptr<osg::TessellationHints> hints,
                     const Data &additionalData = Data(),
                     ConnectionType type = ConnectionType::Line);

 public:
  DirectedConnection(const ConnectionData<osg::Vec3> &data,
                     ConnectionType type = ConnectionType::Line)
      : DirectedConnection(data.name, data.start, data.end, data.radius, data.hints,
                           data.additionalData) {};

  DirectedConnection(const ConnectionData<Point> &data,
                     ConnectionType type = ConnectionType::Line)
      : DirectedConnection(data.name, data.start.getPosition(),
                           data.end.getPosition(), data.radius, data.hints,
                           data.additionalData) {};
  ~DirectedConnection() {
    delete m_start;
    delete m_end;
  }

  osg::Vec3 getDirection() const { return *m_end - *m_start; }
  osg::Vec3 getCenter() const { return (*m_start + *m_end) / 2; }
  osg::Vec3 getStart() const { return *m_start; }
  osg::Vec3 getEnd() const { return *m_end; }
  osg::ref_ptr<osg::Geode> getGeode() const { return m_geode; }
  const auto &getAdditionalData() const { return m_additionalData; }
  void updateColor(const osg::Vec4 &color) {
    core::utils::color::overrideGeodeColor(m_geode, color);
  }

 private:
  osg::ref_ptr<osg::Geode> m_geode;
  osg::Vec3 *m_start;
  osg::Vec3 *m_end;
  Data m_additionalData;
  ConnectionType m_type;
};

// list of directed connections between points
typedef std::vector<osg::ref_ptr<DirectedConnection>> Connections;

class Line : public osg::MatrixTransform {
 public:
  Line(std::string name, const Connections &connections);

  const auto &getConnections() const { return m_connections; }
  const auto &getAdditionalData() const { return m_additionalData; }
  osg::Vec3 getCenter() const {
    osg::Vec3 center(0.0f, 0.0f, 0.0f);
    for (const auto &[_, connection] : m_connections)
      center += connection->getCenter();
    return center / m_connections.size();
  }
  bool operator==(const Line &other) const;

 private:
  void init(const Connections &connections);

  std::string m_name;
  std::map<std::string, osg::ref_ptr<DirectedConnection>> m_connections;
  Data m_additionalData;
};

// TODO: write a concept for PointType
// template <typename PointType>
// using ConnectivityList = std::vector<ConnectionData<PointType>>;
typedef std::vector<osg::ref_ptr<Point>> Points;
typedef std::vector<osg::ref_ptr<Line>> Lines;
typedef std::vector<std::vector<int>> Indices;
typedef std::vector<Data> PointDataList;
typedef std::vector<std::vector<Data>> ConnectionDataList;

}  // namespace core::simulation::grid
