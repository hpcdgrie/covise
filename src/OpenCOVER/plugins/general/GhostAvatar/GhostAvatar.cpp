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
#include <osg/LineWidth>
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

        // Add sliders for Euler angles
        const char *names[3] = {"Pitch", "Yaw", "Roll"};
        for (int i = 0; i < 3; ++i)
        {
            auto slider = new ui::Slider(m_menu, names[i]);
            slider->setBounds(-180, 180);
            slider->setValue(0);
            slider->setCallback([this, i](double val, bool)
                                { m_eulerAngles[i] = val; });
            m_eulerSliders.push_back(slider);
        }
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

        if (rightArm != m_parser.nodeToIk.end())
        {
            auto &bone = rightArm->second;
            if (bone.rot && m_interactorHand)
            {
                auto parentWorld = rightArm->second.parent->osgNode->getWorldMatrices(cover->getObjectsRoot())[0];
                auto boneBindPos = rightArm->second.basePos;

                osg::Vec3 boneWorldPos = boneBindPos * parentWorld;
                auto boneWorldMat = osg::Matrix::translate(boneBindPos) * parentWorld;

                auto targetWorldPos = m_interactorHand->getMatrix().getTrans();
                osg::Vec3 targetDir = targetWorldPos * osg::Matrix::inverse(boneWorldMat);
                targetDir.normalize();

                osg::Quat rot;
                // Extract rest direction from bone's local matrix (bind pose)
                const osgAnimation::Bone *osgBone = dynamic_cast<const osgAnimation::Bone *>(rightArm->first);
                osg::Matrix localMat = osgBone->getMatrixInBoneSpace();
                osg::Vec3 restDir = osg::Vec3(0, 1, 0) * localMat;
                restDir.normalize();
                rot.makeRotate(restDir, targetDir);
                bone.rot->setQuaternion(rot);

                // ----- DEBUGGING (delete methods if not needed anymore) -----
                printRotationEuler(rot);               
                drawDebugLine(boneWorldPos, targetWorldPos);
                //bone.rot->setQuaternion(getQuaternionFromEulerSliders()); 
                // ----- DEBUGGING -----

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
        auto model = osgDB::readNodeFile("/data/STARTS-ECHO/Avatars/ghost_noCloth.fbx");
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
    osg::ref_ptr<osg::MatrixTransform> m_handSphere, m_bar, m_debugLine;
    ;
    BoneParser m_parser;
    std::vector<ui::Slider *> m_sliders;
    ui::Menu *m_menu = nullptr;
    std::unique_ptr<opencover::coVR3DTransRotInteractor> m_interactorHead, m_interactorFloor, m_interactorHand;
    ui::SelectionList *m_defaultDirList = nullptr;
    int m_defaultDirIndex = 0;
    std::vector<ui::Slider *> m_eulerSliders;
    float m_eulerAngles[3] = {0, 0, 0}; // pitch, yaw, roll
    void createInteractors()
    {
        osg::Matrix m;
        auto interSize = 10.2;
        m.setTrans(0, 10, 0.2);
        m_interactorFloor.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "floor", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
        m_interactorFloor->enableIntersection();
        m_interactorFloor->show();

        m.setTrans(-45, 165, 265);
        m_interactorHand.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "hand", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
        m_interactorHand->enableIntersection();
        m_interactorHand->show();
    }

    void drawDebugLine(const osg::Vec3 &start, const osg::Vec3 &end)
    {
        if (m_debugLine.valid())
            m_avatarTrans->removeChild(m_debugLine);
        osg::ref_ptr<osg::Geode> lineGeode = new osg::Geode();
        osg::ref_ptr<osg::Geometry> lineGeom = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> verts = new osg::Vec3Array();
        verts->push_back(start);
        verts->push_back(end);
        lineGeom->setVertexArray(verts);
        lineGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
        colors->push_back(osg::Vec4(1, 0, 0, 1)); // Red
        lineGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
        lineGeode->addDrawable(lineGeom);
        // Set line width for better visibility
        osg::ref_ptr<osg::LineWidth> lineWidth = new osg::LineWidth(8.0f);
        lineGeode->getOrCreateStateSet()->setAttributeAndModes(lineWidth.get(), osg::StateAttribute::ON);
        m_debugLine = new osg::MatrixTransform();
        m_debugLine->addChild(lineGeode);
        m_avatarTrans->addChild(m_debugLine);
    }

    void printRotationEuler(const osg::Quat &rot)
    {
        double pitch, yaw, roll;
        pitch = std::atan2(2.0 * (rot.w() * rot.x() + rot.y() * rot.z()), 1.0 - 2.0 * (rot.x() * rot.x() + rot.y() * rot.y()));
        yaw = std::asin(2.0 * (rot.w() * rot.y() - rot.z() * rot.x()));
        roll = std::atan2(2.0 * (rot.w() * rot.z() + rot.x() * rot.y()), 1.0 - 2.0 * (rot.y() * rot.y() + rot.z() * rot.z()));
        pitch = osg::RadiansToDegrees(pitch);
        yaw = osg::RadiansToDegrees(yaw);
        roll = osg::RadiansToDegrees(roll);
        std::cerr << "rot Euler angles (deg): pitch=" << pitch << " yaw=" << yaw << " roll=" << roll << std::endl;
    }

    osg::Quat getQuaternionFromEulerSliders()
    {
        //  Convert Euler angles (degrees) to radians
        double pitchRad = osg::DegreesToRadians(m_eulerAngles[0]);
        double yawRad = osg::DegreesToRadians(m_eulerAngles[1]);
        double rollRad = osg::DegreesToRadians(m_eulerAngles[2]);
        // Create quaternion from Euler angles (ZYX order: roll, yaw, pitch)
        osg::Quat qPitch(pitchRad, osg::Vec3(1, 0, 0));
        osg::Quat qYaw(yawRad, osg::Vec3(0, 1, 0));
        osg::Quat qRoll(rollRad, osg::Vec3(0, 0, 1));
        return qRoll * qYaw * qPitch;
    }
};

COVERPLUGIN(GhostAvatar)