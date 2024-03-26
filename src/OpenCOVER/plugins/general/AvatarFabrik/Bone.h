#ifndef COVER_PLUGIN_FBXAVATAR_Bone_H
#define COVER_PLUGIN_FBXAVATAR_Bone_H

#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>
#include <osgAnimation/Bone>

#include <osg/NodeVisitor>
#include <fabrik.h>
#include <array>
#include <map>


class BoneParser : public osg::NodeVisitor {


public:
    BoneParser();
    void apply(osg::Node& node) override;
    void update();

    Skeleton m_skeleton;
    osgAnimation::Bone* m_rootBone;
private:
    std::array<osgAnimation::Bone*, numBones> m_bones;
    std::array<osg::ref_ptr<osgAnimation::StackedQuaternionElement>, numBones> m_joints;
    std::array<osg::ref_ptr<osg::MatrixTransform>, numBones> m_boneDummies;
};

constexpr std::array<const char*, numBones> boneNames{ //order see body_number.jpg
    "mixamorig:Hips",               // 1  Hips
    "mixamorig:LeftShoulder",       // 2  LeftShoulder
    "mixamorig:LeftArm",            // 3  LeftArm
    "mixamorig:LeftForeArm",        // 4  LeftForeArm
    "mixamorig:LeftHand",           // 5  LeftHand
    "mixamorig:RightShoulder",      // 6  RightShoulder
    "mixamorig:RightArm",           // 7  RightArm
    "mixamorig:RightForeArm",       // 8  RightForeArm
    "mixamorig:RightHand",          // 9  RightHand
    "mixamorig:LeftUpLeg",          // 10 LeftUpLeg
    "mixamorig:RightUpLeg",         // 11 RightUpLeg
    "mixamorig:LeftLeg",            // 12 LeftLeg
    "mixamorig:LeftFoot",           // 13 LeftFoot
    "mixamorig:LeftToeBase",        // 14 LeftToeBase
    "mixamorig:RightLeg",           // 15 RightLeg
    "mixamorig:RightFoot",          // 16 RightFoot
    "mixamorig:RightToeBase",       // 17 RightToeBase
    "mixamorig:Neck",               // 18 Neck
    "mixamorig:Head",               // 19 Head

};

#endif // COVER_PLUGIN_FBXAVATAR_Bone_H