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
    osg::Matrix m;
    // default size for all interactors
    float interSize = -1.f;
    // if defined, COVER.IconSize overrides the default
    interSize = coCoviseConfig::getFloat("COVER.IconSize", interSize);
    interSize = 0.2;
    m.setTrans(0, 10, 0.2);
    m_interactorFloor.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "floor", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    // m.setTrans(0, 10, 3);
    m.setTrans(0, 0, 0);
    m_interactor.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "hand", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    m_interactor->enableIntersection();
    m_interactor->show();
    m_interactorFloor->enableIntersection();
    m_interactorFloor->show();
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
        
        auto m = modelTrans->getMatrix();
        modelScale->setMatrix(m);
    });
    m_leftLegDistance = new ui::Slider(menu, "leftLegDistance");
    m_rightLegDistance = new ui::Slider(menu, "rightLegDistance");
    m_leftLegDistance->setBounds(-50, 50);
    m_rightLegDistance->setBounds(-50, 50);
    m_leftLegDistance->setValue(15);
    m_rightLegDistance->setValue(-15);

    m_controllPoint = new ui::VectorEditField(menu, "controlPoint");
    m_controllPoint->setCallback([this](const osg::Vec3 &val){
    });

    return true;
}

void LoadedAvatar::update(const osg::Vec3 &targetInWorldCoords)
{

}

osg::Vec3 getClosestPointOnPlane(const osg::Plane &p, const osg::Vec3 &point)
{
    auto d = p.distance(point);
    return point - p.getNormal() * d;
}

osg::Vec3 rotateBoneToControlPoint(const osg::Vec3 &bone, const osg::Vec3 &controlPoint, const osg::Vec3 &originToTarget)
{
    osg::Matrix transformFromOrigin;
    auto targetPositionOrigin = transformFromOrigin * bone;
    osg::Plane p(originToTarget, osg::Vec3(0,0,0));

    auto boneDistanceFromOriginPlane = p.distance(targetPositionOrigin);
    auto boneOnOriginPlane = targetPositionOrigin - originToTarget * boneDistanceFromOriginPlane;
    auto controlOnOriginPlane = getClosestPointOnPlane(p, controlPoint);

    auto boneInControlDirectionPlane = controlOnOriginPlane /controlOnOriginPlane.length() * (boneOnOriginPlane).length();
    auto boneInControlDirection =  boneInControlDirectionPlane + originToTarget * boneDistanceFromOriginPlane; 
    auto boneInControlDirectionLocal = osg::Matrix::inverse(transformFromOrigin) * boneInControlDirection;
    // auto d = std::abs(bone.length2() - boneInControlDirectionLocal.length2());
    // assert(d < 0.01);
    return boneInControlDirectionLocal;
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
        first = false;
        buidCompleteIkModel();
    }
    
    auto targetGlobal = m_interactor->getMatrix();
    auto targetLocal = fromWorldToNode(targetGlobal, m_skeleton->m_rootBone);
    static const osg::Matrix swapYZMatrix{
        1, 0, 0, 0,
        0, 0, 1, 0,
        0, 1, 0, 0,
        0, 0, 0, 1
    };
    targetLocal *= swapYZMatrix;
    Fabrik fabrik(m_skeleton->m_skeleton, targetLocal.getTrans(), targetLocal.getRotate());
    std::cerr << "target: " << targetLocal.getTrans().x() << " " << targetLocal.getTrans().y() << " " << targetLocal.getTrans().z() << std::endl;
    std::cerr << "target: " << targetLocal.getRotate().x() << " " << targetLocal.getRotate().y() << " " << targetLocal.getRotate().z() << " " << targetLocal.getRotate().w() << std::endl;
    for(size_t i = 0; i < numBones; i++)
    {
        std::cerr << m_skeleton->m_skeleton[i].position.x() << " " << m_skeleton->m_skeleton[i].position.y() << " " << m_skeleton->m_skeleton[i].position.z() << std::endl;
    }
    std::cerr << std::endl;
    fabrik.solve();
    // for(size_t i = 0; i < numBones; i++)
    // {
    //     std::cerr << m_skeleton->m_skeleton[i].position.x() << " " << m_skeleton->m_skeleton[i].position.y() << " " << m_skeleton->m_skeleton[i].position.z() << std::endl;
    // }
    m_skeleton->update();

}

void LoadedAvatar::positionAndScaleModel(osg::Node* model, osg::MatrixTransform *pos)
{
    modelTrans = new osg::MatrixTransform;
    auto modelTransLocal = new osg::MatrixTransform;
    // modelTrans->setMatrix(osg::Matrix::translate(10, 10, 10));
    modelTrans->setName("AvatarTrans");
    modelScale = new osg::MatrixTransform;
    modelScale->setName("AvatarScale");
    modelScale->addChild(modelTransLocal);
    float s = 0.01;
    auto scale = osg::Matrix::scale(osg::Vec3f(s, s, s));
    modelScale->setMatrix(scale);
    cover->getObjectsRoot()->addChild(modelTrans);
    modelTrans->addChild(modelScale);
    auto rot1 = osg::Matrix::rotate(osg::PI / 2, 1,0,0);
    auto rot2 = osg::Matrix::rotate(0, 0,0,1);
    auto transM = osg::Matrix::translate(osg::Vec3f{0,0,0});
    modelTransLocal->setMatrix(rot1 * rot2 * transM);
    modelTransLocal->addChild(model);

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
    m_skeleton = std::make_unique<BoneParser>();
    model->accept(*m_skeleton);

}


