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

void drawFrame(const osg::Vec3 &origin, const osg::Matrix &orientation, float length, const std::string &name, osg::ref_ptr<osg::MatrixTransform> &framePtr)
{
    if (framePtr.valid())
    {
        cover->getObjectsRoot()->removeChild(framePtr);
        framePtr = nullptr;
    }

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();

    // Extract rotation part of orientation matrix
    osg::Matrix rotMat = orientation;
    rotMat.setTrans(0, 0, 0);

    // X axis (red)
    osg::Vec3 xAxis = osg::Vec3(1, 0, 0) * rotMat;
    vertices->push_back(origin);
    vertices->push_back(origin + xAxis * length);
    colors->push_back(osg::Vec4(1, 0, 0, 1));
    colors->push_back(osg::Vec4(1, 0, 0, 1));

    // Y axis (green)
    osg::Vec3 yAxis = osg::Vec3(0, 1, 0) * rotMat;
    vertices->push_back(origin);
    vertices->push_back(origin + yAxis * length);
    colors->push_back(osg::Vec4(0, 1, 0, 1));
    colors->push_back(osg::Vec4(0, 1, 0, 1));

    // Z axis (blue)
    osg::Vec3 zAxis = osg::Vec3(0, 0, 1) * rotMat;
    vertices->push_back(origin);
    vertices->push_back(origin + zAxis * length);
    colors->push_back(osg::Vec4(0, 0, 1, 1));
    colors->push_back(osg::Vec4(0, 0, 1, 1));

    geom->setVertexArray(vertices);
    geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    geom->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 6));
    osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(4.0f);
    geom->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
    geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(geom);

    framePtr = new osg::MatrixTransform();
    framePtr->setName(name);
    framePtr->addChild(geode);

    cover->getObjectsRoot()->addChild(framePtr);
}

class GhostAvatar : public coVRPlugin, public ui::Owner
{
public:
    GhostAvatar()
        : coVRPlugin(COVER_PLUGIN_NAME), Owner(COVER_PLUGIN_NAME, cover->ui), m_menu(new ui::Menu("GhostAvatar", this))
    {

        // add sliders to control arm manually with Euler angles (for debugging)
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
        }

        m_avatarTrans->setMatrix(m_interactorFloor->getMatrix());

        // Draw global frame at interactorFloor position
        osg::Vec3 globalOrigin = m_interactorFloor->getMatrix().getTrans();
        drawFrame(globalOrigin, osg::Matrix::identity(), 40.0f, "GlobalFrame", m_globalFrame);

        auto armNode = m_parser.findNode("LeftArm");
        if (armNode != m_parser.nodeToIk.end())
        {
            auto &armBoneParser = armNode->second;
            if (armBoneParser.rot && m_interactorHand)
            {
                // matrices to convert between local and world coordinates
                auto localToWorldMat = armNode->second.parent->osgNode->getWorldMatrices(cover->getObjectsRoot())[0];
                auto worldToLocalMat = osg::Matrix::inverse(localToWorldMat);

                auto localArmPos = armNode->second.basePos;
                auto worldArmPos = localArmPos * localToWorldMat;

                auto worldTargetPos = m_interactorHand->getMatrix().getTrans();
                auto localTargetPos = worldTargetPos * worldToLocalMat;

                // compute vector from the base of the arm to the target
                osg::Vec3 localTargetDir = localTargetPos - localArmPos;
                localTargetDir.normalize();

                // to make the frames of the right arm and COVER match, we must swap y and z and negate y:
                // --- RIGHT ARM ---
                /*
                osg::Matrix adjustMatrix(
                    1, 0, 0, 0,
                    0, 0, 1, 0,
                    0, -1, 0, 0,
                    0, 0, 0, 1);
                */
                // --- LEFT ARM ---
                osg::Matrix adjustMatrix(
                    1, 0, 0, 0,
                    0, 0, -1, 0,
                    0, 1, 0, 0,
                    0, 0, 0, 1);

                osg::Vec3 adjustedTargetDir = adjustMatrix * localTargetDir;

                // rotate the arm bone to point to the target
                osg::Vec3 localArmDir(0, 1, 0); // this could be different, too, depending on the bone!
                osg::Quat rotation;

                rotation.makeRotate(localArmDir, adjustedTargetDir);
                armBoneParser.rot->setQuaternion(rotation);

                // Draw local arm frame at worldArmPos, with orientation from localToWorldMat
                drawFrame(worldArmPos, localToWorldMat, 1.0f, "ArmLocalFrame", m_armLocalFrame);

                drawDebugLine(worldArmPos, worldTargetPos);
            }
        }
        return true;
    }

    void loadAvatar()
    {
        auto model = osgDB::readNodeFile("/home/hpcsmalh/STARTS/ECHO/CoviseAvatar/ghost_noCloth.fbx");
        m_avatarTrans = new osg::MatrixTransform();
        m_avatarTrans->setName("AvatarTrans");
        m_avatarTrans->addChild(model);
        cover->getObjectsRoot()->addChild(m_avatarTrans);
    }

    void drawDebugLine(const osg::Vec3 &armBase, const osg::Vec3 &targetPos)
    {
        // Remove previous line if exists
        if (m_debugLine.valid())
        {
            cover->getObjectsRoot()->removeChild(m_debugLine);
            m_debugLine = nullptr;
        }

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        vertices->push_back(armBase);
        vertices->push_back(targetPos);
        geom->setVertexArray(vertices);

        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));

        osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
        colors->push_back(osg::Vec4(1, 0, 0, 1)); // red
        geom->setColorArray(colors, osg::Array::BIND_OVERALL);

        osg::ref_ptr<osg::LineWidth> linewidth = new osg::LineWidth(3.0f);
        geom->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(geom);

        m_debugLine = new osg::MatrixTransform();
        m_debugLine->addChild(geode);

        cover->getObjectsRoot()->addChild(m_debugLine);
    }

private:
    osg::MatrixTransform *m_avatarTrans = nullptr;
    osg::ref_ptr<osg::MatrixTransform> m_debugLine;
    osg::ref_ptr<osg::MatrixTransform> m_globalFrame;
    osg::ref_ptr<osg::MatrixTransform> m_armLocalFrame;
    BoneParser m_parser;
    std::vector<ui::Slider *> m_animationSliders, m_eulerSliders;
    ui::Menu *m_menu = nullptr;
    std::unique_ptr<opencover::coVR3DTransRotInteractor> m_interactorHead, m_interactorFloor, m_interactorHand;
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
        return qPitch * qRoll * qYaw;
    }
};

COVERPLUGIN(GhostAvatar)