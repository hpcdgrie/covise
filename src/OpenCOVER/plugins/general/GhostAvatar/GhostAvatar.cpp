#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include "GhostAvatar.h"

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

GhostAvatar::GhostAvatar()
    : coVRPlugin(COVER_PLUGIN_NAME), Owner(COVER_PLUGIN_NAME, cover->ui), m_mainMenu(new ui::Menu("GhostAvatar", this))
{
    createSettingsMenu();
    createDebugMenu();
}

void GhostAvatar::loadAvatar()
{
    auto model = osgDB::readNodeFile(m_pathToFbx);
    m_avatarTrans = new osg::MatrixTransform();
    m_avatarTrans->setName("AvatarTrans");
    m_avatarTrans->addChild(model);
    cover->getObjectsRoot()->addChild(m_avatarTrans);
}

bool GhostAvatar::update()
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
            if (m_armBaseVec[0] != 0 || m_armBaseVec[1] != 0 || m_armBaseVec[2] != 0)
            {
                rotation.makeRotate(m_armBaseVec, adjustedTargetDir);
                armBoneParser.rot->setQuaternion(rotation);
            }

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

void GhostAvatar::drawLine(const osg::Vec3 &armBase, const osg::Vec3 &targetPos)
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

void GhostAvatar::cleanUpDebugLines()
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

void GhostAvatar::createInteractors()
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

void GhostAvatar::createSettingsMenu()
{
    m_settingsMenu = new ui::Menu(m_mainMenu, "Settings");
    m_tabletUINote = new ui::Action(m_settingsMenu, "Changes can only be made in the TabletUI!");

    createArmBaseVectorMenu();
    createAdjustMatrixMenu();
}

void GhostAvatar::createArmBaseVectorMenu()
{
    m_armBaseVecMenu = new ui::Menu(m_settingsMenu, "Arm Base Vector");
    m_armBaseVecField = new ui::VectorEditField(m_armBaseVecMenu, "Vector");
    m_armBaseVecField->setValue(m_armBaseVec);
    m_armBaseVecField->setCallback([this](const osg::Vec3 &dir)
                                   { m_armBaseVec = dir; });
}

void GhostAvatar::createAdjustMatrixMenu()
{
    m_adjustMatrixMenu = new ui::Menu(m_settingsMenu, "Adjust Matrix");

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

void GhostAvatar::createDebugMenu()
{
    m_debugMenu = new ui::Menu(m_mainMenu, "Debugging");

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
