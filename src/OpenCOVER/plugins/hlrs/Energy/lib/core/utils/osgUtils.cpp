#include "osgUtils.h"

#include <utils/color.h>

#include <memory>
#include <osg/BlendFunc>
#include <osg/BoundingBox>
#include <osg/BoundingSphere>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/Matrixd>
#include <osg/Shape>
#include <osg/ShapeDrawable>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgText/Text>

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
}  // namespace core::utils::osgUtils
