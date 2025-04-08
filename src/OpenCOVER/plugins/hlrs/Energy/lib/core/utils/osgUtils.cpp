#include "osgUtils.h"

#include <utils/color.h>

#include <memory>
#include <osg/Array>
#include <osg/BlendFunc>
#include <osg/BoundingBox>
#include <osg/BoundingSphere>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Matrixd>
#include <osg/PrimitiveSet>
#include <osg/Shape>
#include <osg/PolygonMode>
#include <osg/ShapeDrawable>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgText/Text>
#include <osgUtil/SmoothingVisitor>

namespace core::utils::osgUtils {

osg::ref_ptr<osgText::Text> createTextBox(const std::string &text,
                                          const osg::Vec3 &position, int charSize,
                                          const char *fontFile,
                                          const float &maxWidth,
                                          const float &margin) {
  osg::ref_ptr<osgText::Text> textBox = new osgText::Text();
  textBox->setAlignment(osgText::Text::LEFT_TOP);
  textBox->setAxisAlignment(osgText::Text::XZ_PLANE);
  textBox->setColor(osg::Vec4(1, 1, 1, 1));
  textBox->setText(text, osgText::String::ENCODING_UTF8);
  textBox->setCharacterSize(charSize);
  textBox->setFont(fontFile);
  textBox->setMaximumWidth(maxWidth);
  textBox->setPosition(position);
  //   textBox->setDrawMode(osgText::Text::FILLEDBOUNDINGBOX | osgText::Text::TEXT);
  textBox->setBoundingBoxMargin(margin);
  textBox->setDrawMode(osgText::Text::TEXT);
  return textBox;
}

void enableLighting(osg::ref_ptr<osg::Geode> geode, bool enable) {
  osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet();
  if (enable) {
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    return;
  }
  stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
}

void setTransparency(osg::ref_ptr<osg::Geode> geode, float alpha) {
  osg::ref_ptr<osg::StateSet> stateset = geode->getOrCreateStateSet();

  // Enable blending
  stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
  stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

  // Set the blend function
  osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
  blendFunc->setFunction(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  stateset->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);

  // Modify the color to include alpha
  for (int i = 0; i < geode->getNumDrawables(); ++i) {
    osg::ref_ptr<osg::Drawable> drawable = geode->getDrawable(i);
    osg::ref_ptr<osg::Geometry> geometry = drawable->asGeometry();
    osg::ref_ptr<osg::Vec4Array> colors =
        dynamic_cast<osg::Vec4Array *>(geometry->getColorArray());
    if (colors) {
      (*colors)[0].a() = alpha;  // Set the alpha value
      geometry->setColorArray(colors, osg::Array::BIND_OVERALL);
      geometry->setColorBinding(
          osg::Geometry::BIND_OVERALL);  // Ensure the color binding is correct.
    }
  }
}

osg::ref_ptr<osg::Geometry> createBackgroundQuadGeometry(const osg::Vec3 &center,
                                                         float width, float height,
                                                         const osg::Vec4 &color) {
  osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

  //
  //         +Z
  //          |  +-----+
  //          |  |     |
  //          |  +-----+
  //          O--------+X
  //         /
  //        /
  //       -Y
  //
  osg::Vec3 topRight(center.x() + width / 2, center.y(), center.z() + height / 2);
  osg::Vec3 bottomRight(center.x() + width / 2, center.y(), center.z() - height / 2);
  osg::Vec3 bottomLeft(center.x() - width / 2, center.y(), center.z() - height / 2);
  osg::Vec3 topLeft(center.x() - width / 2, center.y(), center.z() + height / 2);
  // Vertices of the quad
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
  vertices->push_back(topRight);
  vertices->push_back(bottomRight);
  vertices->push_back(bottomLeft);
  vertices->push_back(topLeft);
  geometry->setVertexArray(vertices);

  // Indices for the quad
  osg::ref_ptr<osg::DrawElementsUInt> quad =
      new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0, 4);
  for (auto i = 0; i < 4; ++i) quad->push_back(i);
  geometry->addPrimitiveSet(quad);

  // Set the color
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
  colors->push_back(color);
  geometry->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);
  return geometry;
}

osg::ref_ptr<osg::Geometry> createBackgroundGeometryForText(
    osg::ref_ptr<osgText::Text> text, float padding,
    const osg::Vec4 &backgroundColor, float depthOffset) {
  const auto &bb = text->getBoundingBox();
  float width = bb.xMax() - bb.xMin() + 2 * padding;
  float height = bb.zMax() - bb.zMin() + 2 * padding;
  osg::Vec3 center = bb.center();
  center.y() += depthOffset;

  return createBackgroundQuadGeometry(center, width, height, backgroundColor);
}

std::unique_ptr<Geodes> getGeodes(osg::Group *grp) {
  Geodes geodes{};
  for (unsigned int i = 0; i < grp->getNumChildren(); ++i) {
    auto child = grp->getChild(i);
    if (osg::ref_ptr<osg::Geode> child_geode = dynamic_cast<osg::Geode *>(child)) {
      geodes.push_back(child_geode);
      continue;
    }
    if (osg::ref_ptr<osg::Group> child_group = dynamic_cast<osg::Group *>(child)) {
      auto child_geodes = getGeodes(child_group);
      std::move(child_geodes->begin(), child_geodes->end(),
                std::back_inserter(geodes));
    }
  }
  return std::make_unique<Geodes>(geodes);
}

osg::BoundingBox getBoundingBox(
    const std::vector<osg::ref_ptr<osg::Geode>> &geodes) {
  osg::BoundingBox bb;
  for (auto geode : geodes) {
    bb.expandBy(geode->getBoundingBox());
  }
  return bb;
}

void deleteChildrenFromOtherGroup(osg::Group *grp, osg::Group *other) {
  if (!grp || !other) return;

  for (unsigned int i = 0; i < other->getNumChildren(); ++i) {
    auto child = other->getChild(i);
    if (grp->containsNode(child)) grp->removeChild(child);
  }
}

void deleteChildrenRecursive(osg::Group *grp) {
  if (!grp) return;

  for (unsigned int i = 0; i < grp->getNumChildren(); ++i) {
    auto child = grp->getChild(i);
    if (auto child_group = dynamic_cast<osg::Group *>(child))
      deleteChildrenRecursive(child_group);
    grp->removeChild(child);
  }
}

osg::ref_ptr<osg::Geode> createCylinderBetweenPoints(
    osg::Vec3 start, osg::Vec3 end, float radius, osg::Vec4 cylinderColor,
    osg::ref_ptr<osg::TessellationHints> hints) {
  osg::ref_ptr geode = new osg::Geode;
  osg::Vec3 center;
  float height;
  osg::ref_ptr<osg::Cylinder> cylinder;
  osg::ref_ptr<osg::ShapeDrawable> cylinderDrawable;
  osg::ref_ptr<osg::Material> pMaterial;

  height = (start - end).length();
  center = osg::Vec3((start.x() + end.x()) / 2, (start.y() + end.y()) / 2,
                     (start.z() + end.z()) / 2);

  // This is the default direction for the cylinders to face in OpenGL
  osg::Vec3 z = osg::Vec3(0, 0, 1);

  // Get diff between two points you want cylinder along
  osg::Vec3 p = start - end;

  // Get CROSS product (the axis of rotation)
  osg::Vec3 t = z ^ p;

  // Get angle. length is magnitude of the vector
  double angle = acos((z * p) / p.length());

  // Create a cylinder between the two points with the given radius
  cylinder = new osg::Cylinder(center, radius, height);
  cylinder->setRotation(osg::Quat(angle, osg::Vec3(t.x(), t.y(), t.z())));

  cylinderDrawable = new osg::ShapeDrawable(cylinder, hints);
  geode->addDrawable(cylinderDrawable);

  // Set the color of the cylinder that extends between the two points.
  color::overrideGeodeColor(geode, cylinderColor);

  return geode;
}

osg::Vec3 cubicBezier(float t, const osg::Vec3 &p0, const osg::Vec3 &p1,
                      const osg::Vec3 &p2, const osg::Vec3 &p3) {
  float u = 1 - t;
  float tt = t * t;
  float uu = u * u;
  float uuu = uu * u;
  float ttt = tt * t;

  osg::Vec3 p = p0 * uuu;
  p += p1 * (3 * uu * t);
  p += p2 * (3 * u * tt);
  p += p3 * ttt;

  return p;
}

osg::ref_ptr<osg::Geode> createBezierTube(const osg::Vec3 &p1, const osg::Vec3 &p2,
                                          float midPointOffset, float tubeRadius,
                                          int numSegments, const osg::Vec4 &color) {
  osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;

  osg::Vec3 midPoint = (p1 + p2) * 0.5f;
  osg::Vec3 direction = (p2 - p1);
  direction.normalize();

  // Find a vector perpendicular to the direction.
  // osg::Vec3 up(0.0f, 1.0f, 0.0f); // Default up vector.
  osg::Vec3 up(0.0f, 0.0f, 1.0f);  // Default up vector.
  osg::Vec3 right = direction ^ up;
  if (right.length2() < 1e-6) {
    // If direction is parallel to up, use another up.
    up.set(0.0f, 1.0f, 0.0f);
    right = direction ^ up;
  }
  right.normalize();
  osg::Vec3 newUp = direction ^ right;
  newUp.normalize();

  // Calculate the offset point.
  osg::Vec3 offsetPoint = midPoint + newUp * midPointOffset;

  // Create control points for the cubic Bezier curve.
  osg::Vec3 controlPoint1 = p1 + (offsetPoint - p1) * 0.33f;
  osg::Vec3 controlPoint2 = p2 + (offsetPoint - p2) * 0.33f;

  // Generate vertices along the Bezier curve.
  std::vector<osg::Vec3> bezierPoints;
  for (int i = 0; i <= numSegments; ++i) {
    float t = static_cast<float>(i) / numSegments;
    bezierPoints.push_back(cubicBezier(t, p1, controlPoint1, controlPoint2, p2));
  }

  // Generate tube vertices and normals.
  for (size_t i = 0; i < bezierPoints.size(); ++i) {
    osg::Vec3 currentPoint = bezierPoints[i];

    // Calculate tangent, normal, and binormal.
    osg::Vec3 tangent;
    if (i == 0) {
      tangent = bezierPoints[1] - bezierPoints[0];
      tangent.normalize();
    } else if (i == bezierPoints.size() - 1) {
      tangent = bezierPoints[i] - bezierPoints[i - 1];
      tangent.normalize();
    } else {
      osg::Vec3 tangent1 = bezierPoints[i + 1] - bezierPoints[i];
      tangent1.normalize();
      osg::Vec3 tangent2 = bezierPoints[i] - bezierPoints[i - 1];
      tangent2.normalize();
      tangent = (tangent1 + tangent2);
      tangent.normalize();
    }

    osg::Vec3 normal = right ^ tangent;
    normal.normalize();
    osg::Vec3 binormal = tangent ^ normal;
    binormal.normalize();

    // Generate vertices around the tube.
    for (int j = 0; j <= numSegments; ++j) {
      float angle = 2.0f * osg::PI * static_cast<float>(j) / numSegments;
      osg::Vec3 vertex =
          currentPoint + (normal * cos(angle) + binormal * sin(angle)) * tubeRadius;
      vertices->push_back(vertex);

      osg::Vec3 outwardNormal = (normal * cos(angle) + binormal * sin(angle));
      outwardNormal.normalize();
      normals->push_back(outwardNormal);
    }
  }

  // Generate indices.
  osg::ref_ptr<osg::DrawElementsUInt> indices =
      new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLE_STRIP);
  for (int i = 0; i < numSegments; ++i) {
    for (int j = 0; j <= numSegments; ++j) {
      indices->push_back((i + 0) * (numSegments + 1) + j);
      indices->push_back((i + 1) * (numSegments + 1) + j);
    }
  }

  geometry->setVertexArray(vertices.get());
  geometry->setNormalArray(normals.get(), osg::Array::BIND_PER_VERTEX);
  geometry->addPrimitiveSet(indices.get());

  osgUtil::SmoothingVisitor sv;
  geometry->accept(sv);

  osg::ref_ptr<osg::Geode> geode = new osg::Geode;
  geode->addDrawable(geometry.get());

  // Add Material
  osg::ref_ptr<osg::Material> material = new osg::Material;
  material->setAmbient(osg::Material::FRONT_AND_BACK,
                       osg::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
  material->setDiffuse(osg::Material::FRONT_AND_BACK, color);
  geode->getOrCreateStateSet()->setAttribute(material.get());

  return geode;
}

osg::ref_ptr<osg::Geode> createBoundingBoxVisualization(const osg::BoundingBox &bb) {
  osg::ref_ptr<osg::Geode> geode = new osg::Geode();
  osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry();

  // Vertices of the bounding box
  osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
  vertices->push_back(osg::Vec3(bb.xMin(), bb.yMin(), bb.zMin()));  // 0
  vertices->push_back(osg::Vec3(bb.xMax(), bb.yMin(), bb.zMin()));  // 1
  vertices->push_back(osg::Vec3(bb.xMax(), bb.yMax(), bb.zMin()));  // 2
  vertices->push_back(osg::Vec3(bb.xMin(), bb.yMax(), bb.zMin()));  // 3
  vertices->push_back(osg::Vec3(bb.xMin(), bb.yMin(), bb.zMax()));  // 4
  vertices->push_back(osg::Vec3(bb.xMax(), bb.yMin(), bb.zMax()));  // 5
  vertices->push_back(osg::Vec3(bb.xMax(), bb.yMax(), bb.zMax()));  // 6
  vertices->push_back(osg::Vec3(bb.xMin(), bb.yMax(), bb.zMax()));  // 7

  geometry->setVertexArray(vertices);

  // Indices for the edges of the bounding box
  osg::ref_ptr<osg::DrawElementsUInt> lines =
      new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
  lines->push_back(0);
  lines->push_back(1);
  lines->push_back(1);
  lines->push_back(2);
  lines->push_back(2);
  lines->push_back(3);
  lines->push_back(3);
  lines->push_back(0);
  lines->push_back(4);
  lines->push_back(5);
  lines->push_back(5);
  lines->push_back(6);
  lines->push_back(6);
  lines->push_back(7);
  lines->push_back(7);
  lines->push_back(4);
  lines->push_back(0);
  lines->push_back(4);
  lines->push_back(1);
  lines->push_back(5);
  lines->push_back(2);
  lines->push_back(6);
  lines->push_back(3);
  lines->push_back(7);

  geometry->addPrimitiveSet(lines);

  // Set the color of the lines (e.g., red)
  osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
  colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));  // Red
  geometry->setColorArray(colors);
  geometry->setColorBinding(osg::Geometry::BIND_OVERALL);

  // Make the lines thicker
  osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth(2.0f);
  geometry->getOrCreateStateSet()->setAttributeAndModes(lineWidth,
                                                        osg::StateAttribute::ON);

  geode->addDrawable(geometry);

  return geode;
}

osg::ref_ptr<osg::Geode> createBoundingSphereVisualization(const osg::BoundingSphere& bs) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();

    // Create a wireframe sphere using osg::ShapeDrawable
    osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(bs.center(), bs.radius());
    osg::ref_ptr<osg::ShapeDrawable> shapeDrawable = new osg::ShapeDrawable(sphere);

    // Set wireframe mode
    osg::ref_ptr<osg::PolygonMode> polygonMode = new osg::PolygonMode;
    polygonMode->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    shapeDrawable->getOrCreateStateSet()->setAttributeAndModes(polygonMode, osg::StateAttribute::ON);

    // Set color (e.g., green)
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)); // Green
    shapeDrawable->setColorArray(colors);
    shapeDrawable->setColorBinding(osg::Geometry::BIND_OVERALL);

    // Make the lines thicker
    osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth(2.0f);
    shapeDrawable->getOrCreateStateSet()->setAttributeAndModes(lineWidth, osg::StateAttribute::ON);

    geode->addDrawable(shapeDrawable);

    return geode;
  }

}  // namespace core::utils::osgUtils
