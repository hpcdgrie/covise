#pragma once

#include <osg/BoundingBox>
#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Vec3>
#include <osg/ref_ptr>
#include <variant>
#include <PluginUtil/coShaderUtil.h>

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

  void move(const osg::Vec3 &offset) {
    setMatrix(osg::Matrix::translate(offset));
  }

  const auto &getRadius() const { return m_radius; }
  const auto &getCenter() const { return m_point->getCenter(); }
  const auto &getPosition() const { return m_point->getCenter(); }
  const auto &getAdditionalData() const { return m_additionalData; }
  osg::ref_ptr<osg::Geode> getGeode() {
    return dynamic_cast<osg::Geode *>(osg::Group::getChild(0));
  }

  void updateColor(const osg::Vec4 &color) {
    core::utils::color::overrideGeodeColor(getGeode(), color);
  }
  bool hasData() const {
    return m_additionalData.size() > 0; // name is always set
  }
 private:
  void init(const std::string &name);

  float m_radius;
  osg::ref_ptr<osg::ShapeDrawable> m_shape;
  osg::ref_ptr<osg::Sphere> m_point;
  Data m_additionalData;
};

struct ConnectionData {
  std::string name;
  std::vector<osg::ref_ptr<Point>> points; // first and last Point must have data
  float radius;
  osg::ref_ptr<osg::TessellationHints> hints;
  Data additionalData;
};

enum class ConnectionType { Line, LineWithColorInterpolation, LineWithShader, Arc, Arrow };

class DirectedConnection : public osg::MatrixTransform {
 public:
  DirectedConnection(const ConnectionData &data,
                     ConnectionType type = ConnectionType::LineWithShader);

  void move(const osg::Vec3 &offset); 
  osg::Vec3 getDirection() const; 
  osg::Vec3 getCenter() const; 

  osg::ref_ptr<Point> getStart() const { return m_connectionData.points[0]; }
  osg::ref_ptr<Point> getEnd() const { return m_connectionData.points.back(); }
  osg::ref_ptr<osg::Geode> getGeode() const { return m_geode; }
  const auto &getAdditionalData() const { return m_connectionData.additionalData; }
  void updateColor(const osg::Vec4 &color) {
    core::utils::color::overrideGeodeColor(m_geode, color);
  }
  void setData(const std::vector<double> &fromData, const std::vector<double> &toData);
  void setColorMap(const opencover::ColorMap&colorMap);
  void updateTimestep(int timestep);
 private:
  osg::ref_ptr<osg::Geode> m_geode;
  ConnectionData m_connectionData;
  ConnectionType m_type;
  //to idea who owns the shader
  opencover::coVRShader *m_shader = nullptr;
};

// list of directed connections between points
typedef std::vector<osg::ref_ptr<DirectedConnection>> Connections;

class Line : public osg::MatrixTransform {
 public:
  Line(std::string name, const Connections &connections);

  void move(const osg::Vec3 &offset) {
    for (const auto &[_, connection] : m_connections) connection->move(offset);
  }

  const auto &getConnections() const { return m_connections; }
  auto &getConnections() { return m_connections; }
  const auto &getAdditionalData() const { return m_additionalData; }
  const auto &getName() const { return m_name; }
  osg::Vec3 getCenter() const;
  const osg::BoundingBox &getBoundingBox() const { return m_boundingBox; }
  void recomputeBoundingBox() { computeBoundingBox(); }
  bool overlap(const Line &other) const;
  bool operator==(const Line &other) const;

 private:
  void init(const Connections &connections);
  void computeBoundingBox();
  std::string m_name;
  osg::BoundingBox m_boundingBox;
  std::map<std::string, osg::ref_ptr<DirectedConnection>> m_connections;
  Data m_additionalData;
};

// TODO: write a concept for PointType
// template <typename PointType>
// using ConnectivityList = std::vector<ConnectionData<PointType>>;
typedef std::vector<osg::ref_ptr<Point>> Points;
typedef std::map<int, osg::ref_ptr<Point>> PointsMap;
typedef std::vector<osg::ref_ptr<Line>> Lines;
typedef std::vector<std::vector<int>> Indices;
typedef std::map<int, Data> PointDataList;
typedef std::vector<std::vector<Data>> ConnectionDataList;

}  // namespace core::simulation::grid
