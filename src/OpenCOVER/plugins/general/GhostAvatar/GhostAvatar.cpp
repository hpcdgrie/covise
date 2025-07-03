/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#include "Bone.h"
#include <cover/coVRPluginSupport.h>
#include <cover/ui/FileBrowser.h>
#include <cover/ui/Owner.h>
#include <osg/MatrixTransform>
#include <map>
#include <cover/ui/Menu.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Action.h>
#include <cover/ui/SelectionList.h>
#include <PluginUtil/coVR3DTransRotInteractor.h>
#include <osg/NodeVisitor>

#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/RigTransformHardware>
#include <osgAnimation/Animation>
#include <osgAnimation/Skeleton>
#include <osgGA/GUIEventHandler>
#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/UpdateBone>
#include <osgDB/ReadFile>

using namespace covise;
using namespace opencover;
using namespace ui;

void printMatrix(const osg::Matrix &m)
{
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            std::cerr << m(i, j) << " ";
        }
        std::cerr << std::endl;
    }
    std::cerr << "------------------------" << std::endl;
}

osg::Matrix fromWorldToNode(const osg::Matrix &sourceInWorld, const osg::Node *node)
{
    auto nodeToWorld = node->getWorldMatrices(cover->getObjectsRoot())[0];
    auto worldToNode = osg::Matrix::inverse(nodeToWorld);
    return sourceInWorld * worldToNode;
}

osg::Matrix fromWorldToNode(const osg::Matrix &sourceInWorld, const osg::Matrix &nodeToWorld)
{
    auto worldToNode = osg::Matrix::inverse(nodeToWorld);
    return sourceInWorld * worldToNode;
}
struct AnimationManagerFinder : public osg::NodeVisitor
{
    osg::ref_ptr<osgAnimation::BasicAnimationManager> m_am;
    AnimationManagerFinder() : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node &node) override
    {

        if (m_am.valid())
            return;

        if (node.getUpdateCallback())
        {
            m_am = dynamic_cast<osgAnimation::BasicAnimationManager *>(node.getUpdateCallback());
            return;
        }

        traverse(node);
    }
};

class GhostAvatar : public coVRPlugin, public ui::Owner
{
public:
    void loadAnimations()
    {
        m_avatarTrans->accept(m_amFinder);
        if (m_amFinder.m_am.valid())
        {
            std::cerr << "Found AnimationManager" << std::endl;
        }
        else
        {
            std::cerr << "No AnimationManager found" << std::endl;
            return;
        }
        for (const auto &anim : m_amFinder.m_am->getAnimationList())
        {
            auto slider = new ui::Slider(m_menu, anim->getName());
            slider->setBounds(0, 1);
            slider->setCallback([this, &anim](double val, bool x)
                                { m_amFinder.m_am->playAnimation(anim, 1, val); });
            m_sliders.push_back(slider);
        }
    }
    GhostAvatar()
        : coVRPlugin(COVER_PLUGIN_NAME), Owner(COVER_PLUGIN_NAME, cover->ui), m_menu(new ui::Menu("GhostAvatar", this))
    {
        auto reloadButton = new ui::Action(m_menu, "Reload Animations");
        reloadButton->setCallback([this]()
                                  {
            for (auto slider : m_sliders)
                delete slider;
            m_sliders.clear();
            loadAnimations(); });

        // Add selection list for default direction
        m_defaultDirList = new ui::SelectionList(m_menu, "Default Arm Direction");
        std::vector<std::string> defaultDirs = {"X+", "X-", "Y+", "Y-", "Z+", "Z-"};
        m_defaultDirList->setList(defaultDirs);
        m_defaultDirList->select(2); // Default to Y+ (confirmed in blender)
        m_defaultDirList->setCallback([this](int idx)
                                      { m_defaultDirIndex = idx; });
    }

    bool update() override
    {
        static bool first = true;
        if (first)
        {
            first = false;
            loadAvatar();
            m_avatarTrans->accept(m_parser);
            createInteractors();
            loadAnimations();
            auto rightArm = m_parser.findNode("RightArm");
            // dynamic_cast<osg::MatrixTransform*>(rightArm->second.osgNode)->addChild(m_bar);
        }
        m_bar->setMatrix(m_interactorFloor->getMatrix());

        // Get bar's current position (origin)
        osg::Vec3 barPos = m_bar->getMatrix().getTrans();
        // Get hand position
        osg::Vec3 handPos = m_interactorHand->getMatrix().getTrans();
        // Direction from bar to hand
        osg::Vec3 dir = handPos - barPos;
        float length = dir.length();
        if (length > 1e-6)
        {
            dir.normalize();
            // Default cylinder axis in OSG is +Z
            osg::Vec3 defaultAxis(0, 0, 1);
            osg::Quat rot;
            rot.makeRotate(defaultAxis, dir);

            // Set matrix: translate to barPos, rotate, scale to match length
            osg::Matrix mat;
            mat.makeRotate(rot);
            mat.setTrans(barPos);
            // Optionally scale the cylinder to reach the hand
            // mat.preMultScale(osg::Vec3(1, 1, length)); // if you want to stretch the bar

            m_bar->setMatrix(mat);
        }
        m_avatarTrans->setMatrix(m_interactorFloor->getMatrix());

        auto rightArm = m_parser.findNode("RightArm");

        const osgAnimation::Bone *osgBone = dynamic_cast<const osgAnimation::Bone *>(rightArm->first);
        if (rightArm != m_parser.nodeToIk.end())
        {
            auto &bone = rightArm->second;
            if (bone.rot && m_interactorHand)
            {
                auto parentWorld = rightArm->second.parent->osgNode->getWorldMatrices(cover->getObjectsRoot())[0];
                auto boneBindPos = rightArm->second.basePos;

                osg::Vec3 boneWorldPos = boneBindPos * parentWorld;
                auto boneWorldMat = osg::Matrix::translate(boneBindPos) * parentWorld;

                auto targetLocal = m_interactorHand->getMatrix() * osg::Matrix::inverse(boneWorldMat);
                osg::Vec3 defaultDir(0, 1, 0);

                osg::Vec3 targetDir = targetLocal.getTrans();
                targetDir.normalize();

                osg::Quat rot;
                rot.makeRotate(defaultDir, targetDir);

                bone.rot->setQuaternion(rot);

                // Set the sphere's position to the bone's world position
                osg::Matrix sphereMat;
                sphereMat.makeTranslate(boneWorldPos);
                m_handSphere->setMatrix(sphereMat);
            }
        }
        return true;
    }
    void loadAvatar()
    {
        auto model = osgDB::readNodeFile("C:/Users/Dennis/Data/Starts/ghost_noCloth.fbx");
        m_avatarTrans = new osg::MatrixTransform();
        m_avatarTrans->setName("AvatarTrans");
        m_avatarTrans->addChild(model);
        cover->getObjectsRoot()->addChild(m_avatarTrans);

        // Create the sphere
        m_handSphere = new osg::MatrixTransform();
        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->setName("HandSphereGeode");
        osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), 0.2));
        geode->addDrawable(sphere);
        m_handSphere->addChild(geode);

        cover->getObjectsRoot()->addChild(m_handSphere);

        m_bar = new osg::MatrixTransform();
        osg::ref_ptr<osg::Geode> barGeode = new osg::Geode();
        barGeode->setName("BarGeode");
        osg::ref_ptr<osg::ShapeDrawable> cylinder = new osg::ShapeDrawable(
            new osg::Cylinder(osg::Vec3(0, 0, 0), 1.0, 100.0)); // radius 0.1, height 1.0
        barGeode->addDrawable(cylinder);
        m_bar->addChild(barGeode);
    }

private:
    AnimationManagerFinder m_amFinder;
    osg::MatrixTransform *m_avatarTrans = nullptr;
    osg::ref_ptr<osg::MatrixTransform> m_handSphere, m_bar;
    BoneParser m_parser;
    std::vector<ui::Slider *> m_sliders;
    ui::Menu *m_menu = nullptr;
    std::unique_ptr<opencover::coVR3DTransRotInteractor> m_interactorHead, m_interactorFloor, m_interactorHand;
    ui::SelectionList *m_defaultDirList = nullptr;
    int m_defaultDirIndex = 0;
    void createInteractors()
    {
        osg::Matrix m;
        auto interSize = 10.2;
        m.setTrans(0, 10, 0.2);
        m_interactorFloor.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "floor", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
        m_interactorFloor->enableIntersection();
        m_interactorFloor->show();

        m.setTrans(0, 11, 1.5);
        m_interactorHand.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "hand", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
        m_interactorHand->enableIntersection();
        m_interactorHand->show();
    }
};

COVERPLUGIN(GhostAvatar)