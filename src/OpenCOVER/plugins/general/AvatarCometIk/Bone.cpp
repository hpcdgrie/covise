#include "Bone.h"
#include <osgAnimation/UpdateBone>
#include <osgAnimation/Bone>
#include <osgAnimation/StackedRotateAxisElement>
#include <array>

BoneParser::BoneParser() 
: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
, ikSolver(ik.solver.create(IK_FABRIK)) {
    /* We want to calculate rotations as well as positions */
    ikSolver->flags = 0; //only calc positions or we get a weird mix of positions and rotations
    ikSolver->flags |= IK_ENABLE_TARGET_ROTATIONS; 
    ikSolver->flags |= IK_ENABLE_CONSTRAINTS; 
}

void BoneParser::apply(osg::Node& node) {

    if(auto bone = dynamic_cast<osgAnimation::Bone*>(&node))
    {
        std::cerr << "bone: " << bone->getName() << std::endl;
        Bone *parent = nullptr;
        ik_node_t *ikNode;
        if(!bone->getBoneParent())
        {
            root = ikSolver->node->create(ikId++);
            ikNode = root;
        }
        else
        {
            parent = &nodeToIk[bone->getBoneParent()];
            ikNode = ikSolver->node->create_child(parent->ikNode, ikId++);
        }
        // auto constraint = ikSolver->constraint->create(IK_CUSTOM);
        // ikSolver->constraint->set_custom(constraint, [](ik_node_t* node){
        //     auto b = (Bone*) node->user_data;
        //     std::cerr << "constraint: " << b->osgNode->getName() << std::endl;
        //     return 1;
        // });
        // auto attachResult = ikSolver->constraint->attach(constraint, ikNode);

        auto &stacked = dynamic_cast<osgAnimation::UpdateBone*>(node.getUpdateCallback())->getStackedTransforms();
        osgAnimation::StackedTranslateElement *ste = nullptr;
        for (const auto &i : stacked)
        {
            if(auto translate = dynamic_cast<osgAnimation::StackedTranslateElement*>(i.get()))
            {
                auto t = translate->getTranslate();
                ikNode->position = ik.vec3.vec3(t.x(), t.y(), t.z());
                ste = translate;
            } 
        }
        
        auto sqe = new osgAnimation::StackedQuaternionElement;
        Bone &ikBone = nodeToIk.emplace(std::make_pair(&node, Bone{sqe, ste, parent, ikNode, &node})).first->second;
        stacked.push_back(sqe);
        ikNode->user_data = &ikBone;
        for (size_t i = 0; i < effectorName.size(); i++)
        {
            if(node.getName() == effectorName[i])
            {
                size_t chainLength = effectorChainLenghts[i];
                auto origin = bone;
                for (size_t i = 0; i < chainLength + 1; i++)
                {
                    auto &stacked =  dynamic_cast<osgAnimation::UpdateBone*>(origin->getUpdateCallback())->getStackedTransforms();
                    stacked.erase(std::remove_if(stacked.begin(), stacked.end(), [](const osgAnimation::StackedTransform::value_type& t){
                        return dynamic_cast<osgAnimation::StackedRotateAxisElement*>(t.get());
                    }), stacked.end());
                    origin = origin->getBoneParent();
                }
                effectors[i] = std::make_unique<Effector>(effectorName[i], ikNode, chainLength, origin, ikSolver);
            }
        }
    }
    traverse(node);
}

BoneParser::NodeMap::iterator BoneParser::findNode(const std::string &name)
{
    return std::find_if(nodeToIk.begin(), nodeToIk.end(), [&name](const NodeMap::value_type &p){
        return p.first->getName() == name;
    });
}

osg::Vec3 BoneParser::claculateBoneDistance(const std::string &boneName1, const std::string &boneName2)
{
    auto bone1it = findNode(boneName1);
    auto bone2it = findNode(boneName2);
    auto rootIt = findNode("mixamorig:Hips");
    if(bone1it == nodeToIk.end() || bone2it == nodeToIk.end() || rootIt == nodeToIk.end())
        return osg::Vec3(0,0,0);
    
    const osg::Node *bone1 = bone1it->first;
    const osg::Node *bone2 = bone2it->first;
    const osg::Node *root = rootIt->first;
    auto bone1Pos = bone1->getWorldMatrices(root)[0].getTrans();
    auto bone2Pos = bone2->getWorldMatrices(root)[0].getTrans();
    return bone2Pos - bone1Pos;
}