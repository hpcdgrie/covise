/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#include "Bone.h"
#include <cover/coVRPluginSupport.h>
#include <cover/ui/FileBrowser.h>
#include <cover/ui/Owner.h>
#include <cover/ui/Button.h>
#include <cover/ui/VectorEditField.h>
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
        m_tabletUINote = new ui::Action(m_menu, "Please make changes in TabletUI!");
        createSettingsMenu();
        createDebugMenu();
    }

    void cleanUpDebugLines()
    {
        if ((!m_showFrames || !m_showFrames->state()) && m_globalFrame.valid())
        {
            cover->getObjectsRoot()->removeChild(m_globalFrame);
            m_globalFrame = nullptr;
        }
        if ((!m_showFrames || !m_showFrames->state()) && m_armLocalFrame.valid())
        {
            cover->getObjectsRoot()->removeChild(m_armLocalFrame);
            m_armLocalFrame = nullptr;
        }

        if ((!m_showTargetLine || !m_showTargetLine->state()) && m_targetLine.valid())
        {
            cover->getObjectsRoot()->removeChild(m_targetLine);
            m_targetLine = nullptr;
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

            auto rightArm = m_parser.findNode(m_armNodeName);
        }
        m_avatarTrans->setMatrix(m_interactorFloor->getMatrix());

        auto armNode = m_parser.findNode(m_armNodeName);
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

                // the axis convention of the bone might note match with the one used in COVER
                osg::Vec3 adjustedTargetDir = m_adjustMatrix * localTargetDir;

                // rotate the arm bone to point to the target
                osg::Quat rotation;
                rotation.makeRotate(m_armBaseDir, adjustedTargetDir);
                armBoneParser.rot->setQuaternion(rotation);

                // UI elements for debugging
                if (m_showFrames && m_showFrames->state())
                {
                    drawFrame(m_interactorFloor->getMatrix().getTrans(), osg::Matrix::identity(), 40.0f, "GlobalFrame", m_globalFrame);
                    drawFrame(worldArmPos, localToWorldMat, 1.0f, "ArmLocalFrame", m_armLocalFrame);
                }

                if (m_showTargetLine && m_showTargetLine->state())
                {
                    drawLine(worldArmPos, worldTargetPos);
                }
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

    void drawLine(const osg::Vec3 &armBase, const osg::Vec3 &targetPos)
    {
        // Remove previous line if exists
        if (m_targetLine.valid())
        {
            cover->getObjectsRoot()->removeChild(m_targetLine);
            m_targetLine = nullptr;
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

        m_targetLine = new osg::MatrixTransform();
        m_targetLine->addChild(geode);

        cover->getObjectsRoot()->addChild(m_targetLine);
    }

private:
    osg::MatrixTransform *m_avatarTrans = nullptr;
    osg::ref_ptr<osg::MatrixTransform> m_targetLine;
    osg::ref_ptr<osg::MatrixTransform> m_globalFrame;
    osg::ref_ptr<osg::MatrixTransform> m_armLocalFrame;
    BoneParser m_parser;
    ui::Menu *m_menu = nullptr;
    std::string m_armNodeName = "LeftArm";
    ui::Menu *m_armBaseDirMenu = nullptr;
    osg::Vec3 m_armBaseDir = {0, 1, 0};
    ui::Menu *m_debugMenu = nullptr;
    ui::Button *m_showFrames = nullptr;
    ui::Button *m_showTargetLine = nullptr;
    ui::Action *m_axisNote = nullptr;
    ui::Action *m_tabletUINote = nullptr;
    std::unique_ptr<opencover::coVR3DTransRotInteractor> m_interactorHead, m_interactorFloor, m_interactorHand;

    // Replace 9 SelectionLists with 3 VectorEditFields for adjustMatrix
    std::array<ui::VectorEditField *, 3> m_adjustMatrixVecFields;
    osg::Matrix m_adjustMatrix = osg::Matrix::identity();
    ui::Menu *m_adjustMatrixMenu = nullptr;

    ui::Menu *m_settingsMenu = nullptr;

    ui::VectorEditField *m_armBaseDirField = nullptr;
    osg::Vec3 m_testVec = osg::Vec3(1.0f, 2.0f, 3.0f); // Example default value

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

    void createArmBaseDirectionMenu()
    {
        m_armBaseDirMenu = new ui::Menu(m_settingsMenu, "Change Arm Base Direction");
        m_armBaseDirField = new ui::VectorEditField(m_armBaseDirMenu, "Arm Base Direction");
        m_armBaseDirField->setValue(m_armBaseDir);
        m_armBaseDirField->setCallback([this](const osg::Vec3 &dir)
                                       { m_armBaseDir = dir; });
    }
    void createDebugMenu()
    {
        m_debugMenu = new ui::Menu(m_menu, "Debugging");

        m_showTargetLine = new ui::Button(m_debugMenu, "Show Target Line");
        m_showTargetLine->setState(false);
        m_showTargetLine->setCallback([this](bool state)
                                      { m_showTargetLine->setState(state); 
                                       cleanUpDebugLines(); });

        m_showFrames = new ui::Button(m_debugMenu, "Show Frames");
        m_showFrames->setState(false);
        m_showFrames->setCallback([this](bool state)
                                  { m_showFrames->setState(state);
                                    cleanUpDebugLines(); });

        m_axisNote = new ui::Action(m_debugMenu, "x - red, y - green, z - blue");
        m_axisNote->setEnabled(false);
    }
    void createAdjustMatrixMenu()
    {
        m_adjustMatrixMenu = new ui::Menu(m_settingsMenu, "Change Adjust Matrix");

        // set correct axis conventions for the GhostAvatar model
        if (m_armNodeName == "LeftArm")
        {
            m_adjustMatrix.set(
                1, 0, 0, 0,
                0, 0, -1, 0,
                0, 1, 0, 0,
                0, 0, 0, 1);
        }
        else if (m_armNodeName == "RightArm")
        {
            m_adjustMatrix.set(
                1, 0, 0, 0,
                0, 0, 1, 0,
                0, -1, 0, 0,
                0, 0, 0, 1);
        }

        for (int row = 0; row < 3; ++row)
        {
            osg::Vec3 rowVec(m_adjustMatrix(row, 0), m_adjustMatrix(row, 1), m_adjustMatrix(row, 2));
            std::string label = "Row " + std::to_string(row);
            m_adjustMatrixVecFields[row] = new ui::VectorEditField(m_adjustMatrixMenu, label);
            m_adjustMatrixVecFields[row]->setValue(rowVec);
            m_adjustMatrixVecFields[row]->setCallback([this, row](const osg::Vec3 &v)
                                                      {
                m_adjustMatrix(row, 0) = v.x();
                m_adjustMatrix(row, 1) = v.y();
                m_adjustMatrix(row, 2) = v.z(); });
        }
    }

    void createSettingsMenu()
    {
        m_settingsMenu = new ui::Menu(m_menu, "Settings");

        createArmBaseDirectionMenu();
        createAdjustMatrixMenu();
    }
};

COVERPLUGIN(GhostAvatar)