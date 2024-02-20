#include "Avatar.h"
#include "Bone.h"

#include <iostream>
#include <config/CoviseConfig.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Action.h>
#include <cover/ui/Button.h>
#include <cover/coVRCommunication.h>
#include <cover/coVRPartner.h>
#include <cover/VRAvatar.h>
#include <cover/coVRFileManager.h>
#include <boost/filesystem/path.hpp>
#include <osgDB/ReadFile>
#include <osgAnimation/UpdateBone>
#include <osg/ShapeDrawable>
#include <cover/ui/Slider.h>
#include <osg/Material>
#include <OpenVRUI/osg/mathUtils.h>
#include <ik/ik.h>
#include <osgAnimation/StackedScaleElement>
#include <cover/VRSceneGraph.h>
#include <algorithm>

#include <osgAnimation/Bone>
#include <osgUtil/UpdateVisitor>
//public

AnimationManagerFinder::AnimationManagerFinder() 
: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}

void AnimationManagerFinder::apply(osg::Node& node) {

    if (m_am.valid())
        return;

    if (node.getUpdateCallback()) {       
        m_am = dynamic_cast<osgAnimation::BasicAnimationManager*>(node.getUpdateCallback());
        return;
    }
    
    traverse(node);
}

bool LoadedAvatar::loadAvatar(const std::string &filename, VRAvatar *partnerAvatar, ui::Menu *menu)
{
    
    m_targetTransform = partnerAvatar->handTransform;
    m_headTransform = partnerAvatar->headTransform;
    model = osgDB::readNodeFile(filename);  
    if(!model)
        return false;
    positionAndScaleModel(model, partnerAvatar->feetTransform);
    std::cerr<< "loaded model " << filename << std::endl;
    buidCompleteIkModel();

    osg::Matrix m;
    // default size for all interactors
    float interSize = -1.f;
    // if defined, COVER.IconSize overrides the default
    interSize = coCoviseConfig::getFloat("COVER.IconSize", interSize);
    m_interactor.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "hand", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    m_interactor->enableIntersection();
    m_interactor->show();
    m_useInteractor = new ui::Button(menu, "useInteractor");
    m_effectorSelector =  new ui::SelectionList(menu, "effector");
    m_effectorSelector->setList(std::vector<std::string>{{"mixamorig:Head", "mixamorig:LeftHand", "mixamorig:RightHand", "mixamorig:LeftFoot", "mixamorig:RightFoot"}});
    m_modelHeight = new ui::Slider(menu, "modelHeight");
    m_modelHeight->setBounds(1.0, 2.5);
    m_modelHeight->setCallback([this](double val, bool x){
        
        if (val < 1)
        {
            return;
        }
        
        auto h = skeleton->claculateBoneDistance("mixamorig:HeadTop_End_end", "mixamorig:RightToe_End_end");
        std::cerr << "skeleton height: " << h.y() << std::endl;
        auto m = modelTrans->getMatrix();
        auto scale = std::abs(val * 1000/ h.y());
        auto sm = osg::Matrix::scale(osg::Vec3f(scale, scale, scale));
        m.makeScale(osg::Vec3f(scale, scale, scale));
        modelScale->setMatrix(m);
    });
    return true;
}

void LoadedAvatar::update(const osg::Vec3 &targetInWorldCoords)
{

}

static void applyIkToOsgNode2(ik_node_t*node, osg::Quat &appliedRotation)
{
    auto bone = (BoneParser::Bone*) node->user_data;
    if(!bone->parent)
        return;
    osg::Vec3d targetPosLocal{node->position.x, node->position.y, node->position.z};
    auto targetPositionOrigin = appliedRotation * targetPosLocal;
    osg::Vec3d sourcePos = bone->basePos->getTranslate();
    osg::Quat r;
    if((sourcePos ^ targetPositionOrigin).length2() > 0.00001)
        r.makeRotate(sourcePos, targetPositionOrigin);
    appliedRotation *= r.inverse();
    bone->parent->rot->setQuaternion(r);
}

osg::Matrix fromWorldToNode(const osg::Matrix &mInWorld, const osg::Node *node)
{
    auto nodeToWorld = node->getWorldMatrices(cover->getObjectsRoot())[0];
    auto worldToNode = osg::Matrix::inverse(nodeToWorld);
    return  mInWorld * worldToNode;
}

//real update function
void LoadedAvatar::update()
{
    static bool first = true;
    if(first)
    {
        auto h = skeleton->claculateBoneDistance("mixamorig:HeadTop_End_end", "mixamorig:RightToe_End_end");
        std::cerr << "skeleton height: " << h.length() << std::endl;
        first = false;
    }
    //transform the target to the origin of the ik effector chain 
    auto &effector = skeleton->effectors[m_effectorSelector->selectedIndex()];
    auto bone = (BoneParser::Bone*)effector->effector->node->user_data;
    auto origin  = ((osgAnimation::Bone*)effector->origin);
    auto originBone = &skeleton->nodeToIk[origin];
    auto targetInOriginCoords = fromWorldToNode(m_interactor->getMatrix(), origin);
    auto m = targetInOriginCoords;


    effector->effector->target_position = ik.vec3.vec3(m.getTrans().x(), m.getTrans().y(), m.getTrans().z());
    effector->effector->target_rotation = ik.quat.quat(m.getRotate().x(), m.getRotate().y(), m.getRotate().z(), m.getRotate().w());
    auto result = ik.solver.solve(skeleton->ikSolver);
    
    
    
    //update the osg counterparts of the ik model
    //ik positions are in the local coord system, so they have to be transformed to the origin system before transforming them to rotations
    std::vector<ik_node_t*> tree;
    
    auto node = effector->effector->node;

    while(node != originBone->ikNode)
    {
        tree.push_back(node);
        node = node->parent;
    }
    tree.push_back(node);
    osg::Quat rotationFromOrigin;
    for (auto it = tree.rbegin(); it != tree.rend(); ++it)
    {
        applyIkToOsgNode2(*it, rotationFromOrigin);
    }
    // auto finalBone = (BoneParser::Bone*)effector->effector->node->user_data; 
    // finalBone->rot->setQuaternion(rotationFromOrigin * targetInOriginCoords.getRotate()); 
}

void LoadedAvatar::positionAndScaleModel(osg::Node* model, osg::MatrixTransform *pos)
{
    modelTrans = new osg::MatrixTransform;
    modelTrans->setMatrix(osg::Matrix::translate(10, 10, 10));
    modelTrans->setName("AvatarTrans");
    modelScale = new osg::MatrixTransform;
    modelScale->setName("AvatarScale");
    modelScale->addChild(modelTrans);
    auto scale = osg::Matrix::scale(osg::Vec3f(10, 10, 10));
    modelScale->setMatrix(scale);
    cover->getObjectsRoot()->addChild(modelScale);
    auto rot1 = osg::Matrix::rotate(osg::PI / 2, 1,0,0);
    auto rot2 = osg::Matrix::rotate(0, 0,0,1);
    auto transM = osg::Matrix::translate(osg::Vec3f{0,0,0});
    modelTrans->setMatrix(rot1 * rot2 * transM);
    modelTrans->addChild(model);

}

void LoadedAvatar::createAnimationSliders(ui::Menu *menu)
{
    model->accept(m_animationFinder);
    m_animations = m_animationFinder.m_am->getAnimationList();
    if(m_animations.empty())
        return;
    for(const auto & anim : m_animations)
    {
        anim->setPlayMode(osgAnimation::Animation::PlayMode::LOOP);
        auto slider = new ui::Slider(menu, anim->getName());
        slider->setBounds(0,1);
        slider->setCallback([this, &anim](double val, bool x){
            m_animationFinder.m_am->playAnimation(anim, 1, val);
            if(val < 0.01)
                m_animationFinder.m_am->stopAnimation(anim);
        });
        m_animationFinder.m_am->stopAnimation(anim);
    }
    //play animation to reset orientation
    auto &anim = *m_animations.begin();
    m_animationFinder.m_am->playAnimation(anim, 1, 1);
}

void LoadedAvatar::buidCompleteIkModel()
{
    skeleton = new BoneParser; //never deleted :(
    model->accept(*skeleton);


    /* Assign our tree to the solver, rebuild data and calculate solution */

    ik.solver.set_tree(skeleton->ikSolver, skeleton->root);
    ik.solver.rebuild(skeleton->ikSolver);


    
}


