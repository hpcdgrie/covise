#include "ConnectionLine.h"


ConnectionLine::ConnectionLine(osg::MatrixTransform *p1, osg::MatrixTransform *p2, float radius)
: m_point1(p1), m_point2(p2)
{
    m_line = new osg::MatrixTransform;
    auto geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(), radius, 1.0f)));
    geode->setName("ConnectionLine");
    m_line->addChild(geode);
    m_point1->addChild(m_line.get());
    update();
}

void ConnectionLine::update()
{
    osg::Vec3 p1 = m_point1->getMatrix().getTrans();
    osg::Vec3 p2 = m_point2->getMatrix().getTrans();
    osg::Vec3 connectionVec = (p2 - p1);
    float distance = (connectionVec).length();
    osg::Matrix mat;
    mat = osg::Matrix::scale(osg::Vec3(1.0f, 1.0f, distance)); 
    mat *= osg::Matrix::rotate(osg::Vec3(0.0f, 0.0f, 1.0f), connectionVec);
    mat *= osg::Matrix::translate(connectionVec / 2.0f);

    m_line->setMatrix(mat); // Apply the transformation to the line
}