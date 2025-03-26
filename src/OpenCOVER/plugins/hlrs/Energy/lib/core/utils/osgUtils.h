#ifndef _CORE_UTILS_OSGUTILS_H
#define _CORE_UTILS_OSGUTILS_H

#include <memory>
#include <osg/BoundingBox>
#include <osg/Geode>
#include <osg/Geometry>
#include <osgText/Text>
#include <vector>

namespace core::utils::osgUtils {

typedef std::vector<osg::ref_ptr<osg::Geode>> Geodes;

std::unique_ptr<Geodes> getGeodes(osg::Group *grp);
osg::BoundingBox getBoundingBox(const Geodes &geodes);
void deleteChildrenFromOtherGroup(osg::Group *grp, osg::Group *anotherGrp);
void deleteChildrenRecursive(osg::Group *grp);
/**
 * @brief Adds a cylinder between two points.
 * Source: http://www.thjsmith.com/40/cylinder-between-two-points-opengl-c
 *
 * @param start The starting point of the cylinder.
 * @param end The ending point of the cylinder.
 * @param radius The radius of the cylinder.
 * @param cylinderColor The color of the cylinder.
 * @param group The group to which the cylinder will be added.
 * @param hints The tessellation hints for the cylinder.
 */
osg::ref_ptr<osg::Geode> createCylinderBetweenPoints(
    osg::Vec3 start, osg::Vec3 end, float radius, osg::Vec4 cylinderColor,
    osg::ref_ptr<osg::TessellationHints> hints = new osg::TessellationHints());

osg::ref_ptr<osgText::Text> createTextBox(const std::string &text,
                                          const osg::Vec3 &position, int charSize,
                                          const char *fontFile,
                                          const float &maxWidth,
                                          const float &margin);
void setTransparency(osg::ref_ptr<osg::Geode> geode, float alpha);
osg::ref_ptr<osg::Geometry> createBackgroundGeometryForText(
    osg::ref_ptr<osgText::Text> text, float padding,
    const osg::Vec4 &backgroundColor, float depthOffset = 0.1f);
osg::ref_ptr<osg::Geometry> createBackgroundQuadGeometry(const osg::Vec3 &center,
                                                         float width, float height,
                                                         const osg::Vec4 &color);
void enableLighting(osg::ref_ptr<osg::Geode> geode, bool enable = true);
}  // namespace core::utils::osgUtils
#endif
