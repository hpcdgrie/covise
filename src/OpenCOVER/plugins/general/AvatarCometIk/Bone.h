#ifndef COVER_PLUGIN_FBXAVATAR_Bone_H
#define COVER_PLUGIN_FBXAVATAR_Bone_H

#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/StackedQuaternionElement>

#include <osg/NodeVisitor>

#include <ik/ik.h>

#include <array>
#include <map>

struct BoneParser : public osg::NodeVisitor {
struct Bone
{
    osg::ref_ptr<osgAnimation::StackedQuaternionElement> rot; //rotation to manipulate joint
    const osgAnimation::StackedTranslateElement* basePos; //initial position of the joint
    Bone *parent = nullptr; 
    ik_node_t *ikNode = nullptr;
    osg::Node *osgNode = nullptr;
    std::unique_ptr<osg::Vec3> controlPoint; //the bone should bend towards this position
};
struct Effector{
   Effector(const char* name, ik_node_t* ikNode, size_t chainLenght, osg::Node* origin, ik_solver_t *ikSolver) 
   : name(name)
   , effector(ikSolver->effector->create())
   , origin(origin)
   {
        ikSolver->effector->attach(effector, ikNode);
        effector->chain_length = chainLenght;
   }

const char* name = nullptr;
ik_effector_t *effector = nullptr;
osg::Node* origin = nullptr; //The effector position has to be set in the local coordinate system of the origin, which ist chainLenght + 1 bones away from the effector
};

public:

    typedef  std::map<const osg::Node*, Bone> NodeMap;
    BoneParser();
    void apply(osg::Node& node);
    NodeMap::iterator findNode(const std::string &name);
    osg::Vec3 claculateBoneDistance(const std::string &boneName1, const std::string &boneName2);
    NodeMap nodeToIk;
    ik_solver_t *ikSolver; 
    uint32_t ikId = 0;
    ik_node_t *root;
    std::array<std::unique_ptr<Effector>, 5> effectors;
    const std::array<const char*, 5> effectorName{"mixamorig:Head", "mixamorig:LeftHand", "mixamorig:RightHand", "mixamorig:LeftFoot", "mixamorig:RightFoot"};
    // const std::array<const char*, 1> effectorName{"mixamorig:RightHand"};
    const std::array<size_t, 5> effectorChainLenghts{1, 2, 2, 2, 2};
};

#endif // COVER_PLUGIN_FBXAVATAR_Bone_H