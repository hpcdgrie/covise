#ifndef COVER_PLUGINUTIL_SHADERUTIL_H
#define COVER_PLUGINUTIL_SHADERUTIL_H
#include <cover/coVRShader.h>
#include "colors/coColorMap.h"
#include <string>
#include <osg/Drawable>
#include <util/coExport.h>
namespace opencover
{
constexpr int DataAttrib = 10; //use this when setting the attribute array
PLUGIN_UTILEXPORT coVRShader *applyShader(osg::Drawable *drawable, const ColorMap &colorMap, float min, float max, const std::string& shaderFile);

}

#endif // COVER_PLUGINUTIL_SHADERUTIL_H