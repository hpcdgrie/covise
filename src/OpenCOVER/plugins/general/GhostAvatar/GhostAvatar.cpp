#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include "GhostAvatar.h"
#include <cover/coVRPluginSupport.h>
#include <cover/coVRPartner.h>
#include <cover/VRAvatar.h>
using namespace covise;
using namespace opencover;
using namespace ui;

constexpr bool USE_INTERACTORS = false;

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

GhostAvatar::GhostAvatar(int id, osg::Matrix adjustMatrix, osg::Matrix permFix)
    : m_id(id), m_adjustMatrix(adjustMatrix), m_permFix(permFix)
{
    std::cerr << "Creating GhostAvatar for partner ID " << m_id << "\n";
}

GhostAvatar::~GhostAvatar()
{
    std::cerr << "Destroying GhostAvatar for partner ID " << m_id << "\n";
    cover->getObjectsRoot()->removeChild(m_avatarTrans);
    cover->getObjectsRoot()->removeChild(m_globalFrame);
    cover->getObjectsRoot()->removeChild(m_armLocalFrame);
}

void GhostAvatar::loadAvatar()
{
    auto model = osgDB::readNodeFile(m_pathToFbx);

    m_avatarTrans = new osg::MatrixTransform();
    m_avatarTrans->setName("AvatarTrans");

    // Fixed axis conversion: Model' = [Bz, Bx, By]
    m_axisFix = new osg::MatrixTransform();
    m_axisFix->setName("AxisFix");

    // Apply UI-configurable axis permutation (defaults to identity)
    m_axisFix->setMatrix(m_permFix);

    m_axisFix->addChild(model);
    m_avatarTrans->addChild(m_axisFix);
    cover->getObjectsRoot()->addChild(m_avatarTrans);
}

static void setBoneToWorldPosition(osgAnimation::Bone* bone,
                                   const osg::Vec3 &worldPos)
{
    if (!bone || !bone->getParent(0)) return;

    // stop UpdateBone (so stacked elements / callbacks won't overwrite us)
    bone->setUpdateCallback(nullptr);

    // compute world->skeleton (avatar root + axis fix define skeleton->world)
    auto reference = bone->getParent(0)->getWorldMatrices(cover->getObjectsRoot())[0];
    auto inv = osg::Matrix::inverse(reference);
    // transform world position into skeleton space (note Vec * Matrix)
    osg::Vec3 posInSkeleton = worldPos * inv;

    // build a matrix for the bone in skeleton space (only translation here)
    osg::Matrix boneMtxInSkeleton = osg::Matrix::translate(posInSkeleton);

    // apply directly to the bone
    bone->setMatrixInSkeletonSpace(boneMtxInSkeleton);
}

bool GhostAvatar::update()
{
    if (m_firstUpdate)
    {
        m_firstUpdate = false;
        loadAvatar();
        m_avatarTrans->accept(m_parser);
        if(USE_INTERACTORS)
            createInteractors();

        auto rightArm = m_parser.findNode(ARM_NODE_NAME);
        m_head = dynamic_cast<osgAnimation::Bone*>(m_parser.findNode("Bone.002")->second.osgNode);
        m_arm = dynamic_cast<osgAnimation::Bone*>(m_parser.findNode("Bone.003")->second.osgNode);
        m_feet = dynamic_cast<osgAnimation::Bone*>(m_parser.findNode("Bone")->second.osgNode);
        // stop any stacked-transform update callbacks on these bones so we can set their matrices directly
        // if (m_head)
        //     m_head->setUpdateCallback(nullptr);
        // if (m_arm)
        //     m_arm->setUpdateCallback(nullptr);
        // if (m_feet)
        //     m_feet->setUpdateCallback(nullptr);
    
    }
    auto floorPos = USE_INTERACTORS ? m_interactorFloor->getMatrix() : coVRPartnerList::instance()->get(m_id)->getAvatar()->feetTransform->getMatrix();
    m_avatarTrans->setMatrix(coVRPartnerList::instance()->get(m_id)->getAvatar()->feetTransform->getMatrix());

    if(m_head && m_arm)
    {
        auto armPos = USE_INTERACTORS ? m_interactorHand->getMatrix() : coVRPartnerList::instance()->get(m_id)->getAvatar()->handTransform->getMatrix();
        auto headPos = USE_INTERACTORS ? m_interactorHead->getMatrix() : coVRPartnerList::instance()->get(m_id)->getAvatar()->headTransform->getMatrix();

        setBoneToWorldPosition(m_arm, armPos.getTrans());
        setBoneToWorldPosition(m_head, headPos.getTrans());

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
    if (m_showFrames && m_globalFrame.valid())
    {
        cover->getObjectsRoot()->removeChild(m_globalFrame);
        m_globalFrame = nullptr;
    }
    if (!m_showFrames && m_armLocalFrame.valid())
    {
        cover->getObjectsRoot()->removeChild(m_armLocalFrame);
        m_armLocalFrame = nullptr;
    }

    if (!m_showTargetLine && m_targetLine.valid())
    {
        cover->getObjectsRoot()->removeChild(m_targetLine);
        m_targetLine = nullptr;
    }
}

void GhostAvatar::createInteractors()
{
    std::cerr << "Creating interactors for GhostAvatar " << m_id << "\n";
    osg::Matrix m;
    auto interSize = 10.2;
    m.setTrans(0, 0, 237);
    m_interactorFloor.reset(new coVR3DTransformInteractor(interSize, vrui::coInteraction::InteractionType::ButtonA, "floor", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    m_interactorFloor->updateTransform(m);
    m_interactorFloor->enableIntersection();
    m_interactorFloor->show();

    m.setTrans(130, 0, 53);
    m_interactorHand.reset(new coVR3DTransformInteractor(interSize, vrui::coInteraction::InteractionType::ButtonA, "hand", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    m_interactorHand->updateTransform(m);
    m_interactorHand->enableIntersection();
    m_interactorHand->show();

    m.setTrans(0, 0, 27);
    m_interactorHead.reset(new coVR3DTransformInteractor(interSize, vrui::coInteraction::InteractionType::ButtonA, "headBone", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    m_interactorHead->updateTransform(m);
    m_interactorHead->enableIntersection();
    m_interactorHead->show();
}
