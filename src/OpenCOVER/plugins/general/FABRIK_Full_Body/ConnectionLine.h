#ifndef COVER_PLUGIN_CONNECTIONLINE_H
#define COVER_PLUGIN_CONNECTIONLINE_H

#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osg/Geode>

class ConnectionLine
{
public:
ConnectionLine(osg::MatrixTransform *p1, osg::MatrixTransform *p2, float radius = 0.01f);    
void update();

private:
osg::ref_ptr<osg::MatrixTransform> m_line;
osg::MatrixTransform *m_point1;
osg::MatrixTransform *m_point2;

};


#endif // COVER_PLUGIN_CONNECTIONLINE_H