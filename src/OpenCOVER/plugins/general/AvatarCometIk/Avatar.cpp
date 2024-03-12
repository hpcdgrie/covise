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
    interSize = 0.2;
    m.setTrans(0, 10, 0.2);
    m_interactorFloor.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "floor", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    m.setTrans(0, 10, 3);
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
        
        auto h = skeleton->claculateBoneDistance("mixamorig:HeadTop_End_end", "mixamorig:RightToe_End_end");
        std::cerr << "skeleton height: " << h.y() << std::endl;
        auto m = modelTrans->getMatrix();
        auto scale = std::abs(val * 1000/ h.y());
        auto sm = osg::Matrix::scale(osg::Vec3f(scale, scale, scale));
        m.makeScale(osg::Vec3f(scale, scale, scale));
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
        auto &bone = skeleton->nodeToIk[skeleton->findNode("mixamorig:RightLeg")->first];
        bone.controlPoint = std::make_unique<osg::Vec3>(val);
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

static void applyIkToOsgNode2(ik_node_t*node, osg::Matrixd &transformFromOrigin, const osg::Vec3d &originToTarget, osg::Quat &correctionRotation)
{

    auto bone = (BoneParser::Bone*) node->user_data;
    if(!bone->parent)
        return;
    osg::Vec3d bonePosLocal{node->position.x, node->position.y, node->position.z};
    bonePosLocal = correctionRotation * bonePosLocal;
    if(bone->controlPoint)
    {
        auto transformToOrigin = osg::Matrix::inverse(transformFromOrigin);
        auto bonePositionOrigin = bonePosLocal * transformToOrigin;
        bonePositionOrigin = rotateBoneToControlPoint(bonePositionOrigin, *bone->controlPoint, originToTarget);
        auto bonePosLocalCorrected = bonePositionOrigin * transformFromOrigin;
        osg::Quat r;
        if((bonePosLocal ^ bonePosLocalCorrected).length2() > 0.00001)
        {
            r.makeRotate(bonePosLocal, bonePosLocalCorrected);
            correctionRotation *= r;
        }
    }
    auto targetPositionOrigin = transformFromOrigin.getRotate() * bonePosLocal;
    osg::Vec3d sourcePos = bone->basePos->getTranslate();
    osg::Quat r;
    if((sourcePos ^ targetPositionOrigin).length2() > 0.00001)
        r.makeRotate(sourcePos, targetPositionOrigin);
    osg::Matrixd m;
    m.setRotate(r);
    m.setTrans(bonePosLocal);
    transformFromOrigin *= osg::Matrixd::inverse(m);
    bone->parent->rot->setQuaternion(r);
}

osg::Matrix fromWorldToNode(const osg::Matrix &mInWorld, const osg::Node *node)
{
    auto nodeToWorld = node->getWorldMatrices(cover->getObjectsRoot())[0];
    auto worldToNode = osg::Matrix::inverse(nodeToWorld);
    return  mInWorld * worldToNode;
}

class IkUpdater{
public:
    IkUpdater(BoneParser *skeleton) : skeleton(skeleton) {}
    void updateEffectorChain(const BoneParser::Effector &effector, const osg::Matrix &gloabalTarget, const osg::Vec3 &localOffset = osg::Vec3(0,0,0))
    {
        //transform the target to the origin of the ik effector chain 
        auto bone = (BoneParser::Bone*)effector.effector->node->user_data;
        auto origin  = ((osgAnimation::Bone*)effector.origin);
        auto targetInOriginCoords = fromWorldToNode(gloabalTarget, origin);
        auto m = targetInOriginCoords;
        m.setTrans(m.getTrans() + localOffset);
        effector.effector->target_position = ik.vec3.vec3(m.getTrans().x(), m.getTrans().y(), m.getTrans().z());
        effector.effector->target_rotation = ik.quat.quat(m.getRotate().x(), m.getRotate().y(), m.getRotate().z(), m.getRotate().w());
        effectors.push_back(&effector);
    }
    ikret_t solveAndApply()
    {
        auto result = ik.solver.solve(skeleton->ikSolver);
        for (auto effector : effectors)
        {
            std::vector<ik_node_t*> tree;
            auto originBone = &skeleton->nodeToIk[((osgAnimation::Bone*)effector->origin)];

            auto node = effector->effector->node;

            while(node != originBone->ikNode)
            {
                tree.push_back(node);
                node = node->parent;
            }
            tree.push_back(node);
            auto tp = effector->effector->target_position;
            auto originToTarget =  osg::Vec3(tp.x, tp.y, tp.z);
            originToTarget.normalize();
            osg::Matrixd transformFromOrigin;
            osg::Quat correctionRotation;
            for (auto it = tree.rbegin(); it != tree.rend(); ++it)
            {
                applyIkToOsgNode2(*it, transformFromOrigin, originToTarget, correctionRotation);
            }
            // auto finalBone = (BoneParser::Bone*)effector->effector->node->user_data; 
            // finalBone->rot->setQuaternion(rotationFromOrigin * targetInOriginCoords.getRotate()); 
        }
        return result;
    }
    
private:
    BoneParser *skeleton;
    std::vector<const BoneParser::Effector*> effectors;
};


//real update function
void LoadedAvatar::update()
{
    static bool first = true;
    if(first)
    {
        m_modelHeightValue = skeleton->claculateBoneDistance("mixamorig:HeadTop_End", "mixamorig:RightToe_End").y();
        auto heightTorso = skeleton->claculateBoneDistance("mixamorig:HeadTop_End", "mixamorig:Hips").y();
        first = false;
    }

    auto floorGlobal = m_interactorFloor->getMatrix();
    auto targetGlobal = m_interactor->getMatrix();

    auto heightGlobal = targetGlobal.getTrans().y() - floorGlobal.getTrans().y();
    auto &leftFoot = skeleton->effectors[3];
    auto &rightFoot = skeleton->effectors[4];
    auto modelScale = model->getWorldMatrices(cover->getObjectsRoot())[0].getScale().x();
    auto heightModel = m_modelHeightValue * modelScale;

    auto targetGlobalPos = targetGlobal.getTrans();
    coCoord coord(targetGlobal);
    coord.hpr[1] = 0;
    coord.hpr[2] = 0;

    auto modelTransMatrix = targetGlobal;
    coord.makeMat(modelTransMatrix);
    auto s = targetGlobal.getScale();
    targetGlobalPos.z() -= heightModel *10;
    modelTransMatrix.setTrans(targetGlobalPos);
    modelTrans->setMatrix(modelTransMatrix);


    auto targetLeftFootGloabl = targetGlobal;
    auto targetKeftFootPos = targetLeftFootGloabl.getTrans();
    targetKeftFootPos.z() = floorGlobal.getTrans().z();
    // targetKeftFootPos.x() += legDistance;
    targetLeftFootGloabl.setTrans(targetKeftFootPos);

    auto targetRightFootGloabl = targetGlobal;  
    auto targetRightFootPos = targetRightFootGloabl.getTrans();
    targetRightFootPos.z() = floorGlobal.getTrans().z();
    // targetRightFootPos.x() -= legDistance;
    targetRightFootGloabl.setTrans(targetRightFootPos);

    float legDistance = 0;
    IkUpdater ikUpdater(skeleton);
    ikUpdater.updateEffectorChain(*leftFoot, targetLeftFootGloabl, osg::Vec3(m_leftLegDistance->value(),0,0));
    ikUpdater.updateEffectorChain(*rightFoot, targetRightFootGloabl, osg::Vec3(m_rightLegDistance->value(),0,0));
    auto result = ikUpdater.solveAndApply();


    //transform the target to the origin of the ik effector chain 
    // auto &effector = skeleton->effectors[m_effectorSelector->selectedIndex()];
    // updateEffectorChain(*effector, targetGlobal);

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
    skeleton = new BoneParser; //never deleted :(
    model->accept(*skeleton);
    skeleton->findNode("mixamorig:RightLeg")->second.controlPoint = std::make_unique<osg::Vec3>(osg::Vec3(0,1,0));
    // skeleton->findNode("mixamorig:LeftLeg")->second.controlPoint = std::make_unique<osg::Vec3>(osg::Vec3(0,1,0));

    /* Assign our tree to the solver, rebuild data and calculate solution */

    ik.solver.set_tree(skeleton->ikSolver, skeleton->root);
    ik.solver.rebuild(skeleton->ikSolver);


    
}


