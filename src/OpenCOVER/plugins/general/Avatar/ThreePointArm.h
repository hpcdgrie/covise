#ifndef _THREE_POINT_ARM_H
#define _THREE_POINT_ARM_H

#include <osg/ref_ptr>
#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/Bone>
#include <osg/Matrix>
#include <string>
struct Joint
{
    osg::ref_ptr<osgAnimation::StackedQuaternionElement> rot; //rotation to manipulate joint
    const osgAnimation::StackedTranslateElement* basePos; //initial position of the joint
    osgAnimation::Bone *bone = nullptr;
    std::string name;
};

class ThreePointArm
{
private:
Joint m_base, m_mid, m_end;
float m_armLength;
osg::Vec3 m_controlPoint{0, 0, 1}; //the arm should bend towards this position
osg::Quat m_randomRot{0.5, 0.5, 0.5, 0.5};
public:
    ThreePointArm(osg::Node &model, const std::string &base, const std::string& mid, const std::string &end);
    void update(const osg::Matrix &target, float angle = 0);
    void setControlPoint(const osg::Vec3 &controlPoint){m_controlPoint = controlPoint;}

};






#endif // _THREE_POINT_ARM_H
