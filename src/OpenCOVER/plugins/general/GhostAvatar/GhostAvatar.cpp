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

/*osg::Quat computeRotation(const osg::Vec3 &target, const osg::Vec3 &boneVector)
{
    // Current bone direction
    osg::Vec3 v = boneVector;
    v.normalize();

    // Dot and cross
    double c = v * target;       // dot product
    osg::Vec3 axis = v ^ target; // cross product
    double s = axis.length();

    if (s < 1e-8) // vectors nearly parallel
    {
        if (c > 0.0)
        {
            // Already aligned
            return osg::Quat(); // identity
        }
        else
        {
            // Opposite direction: rotate 180° about a perpendicular axis
            osg::Vec3 perp = osg::Vec3(1, 0, 0);
            if (fabs(v * perp) > 0.9) // too parallel, pick another
                perp = osg::Vec3(0, 1, 0);

            axis = v ^ perp;
            axis.normalize();
            return osg::Quat(osg::PI, axis);
        }
    }

    axis.normalize();
    double angle = atan2(s, c);

    return osg::Quat(angle, axis);
}*/

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

        auto rightArm = m_parser.findNode("RightArm");
        if (rightArm != m_parser.nodeToIk.end())
        {
            auto &bone = rightArm->second;
            if (bone.rot && m_interactorHand)
            {
                auto localToWorldMat = rightArm->second.parent->osgNode->getWorldMatrices(cover->getObjectsRoot())[0];
                auto worldToLocalMat = osg::Matrix::inverse(localToWorldMat);

                // get right arm position and target position in local frame coordinates
                auto rightArmPosLocal = rightArm->second.basePos;
                auto rightArmPosWorld = rightArmPosLocal * localToWorldMat;

                auto targetPosWorld = m_interactorHand->getMatrix().getTrans();
                auto targetPosLocal = targetPosWorld * worldToLocalMat;

                // get the directional vector between target position and right arm base
                // auto direction = targetPosLocal - rightArmPosLocal;
                // direction.normalize();

                auto boneQuat = getQuaternionFromEulerSliders();
                bone.rot->setQuaternion(boneQuat);

                // yaw and roll seem to be switched
                boneQuat = osg::Quat(
                    osg::DegreesToRadians(m_eulerAngles[0]), osg::Vec3(1, 0, 0), 
                    osg::DegreesToRadians(-m_eulerAngles[2]), osg::Vec3(0, 1, 0), 
                    osg::DegreesToRadians(m_eulerAngles[1]), osg::Vec3(0, 0, 1)  
                );
                auto rotatedDirection = boneQuat *  osg::Vec3(0, 0, 1); // rotate direction by quaternion

                float boneLength = 1.5f; // use actual bone length if available
                auto endPosLocal = rightArmPosLocal + rotatedDirection * boneLength;
                auto endPosWorld = endPosLocal * localToWorldMat;

                // auto quat = computeRotation(direction, osg::Vec3(1, 0, 0));
                // bone.rot->setQuaternion(quat);

                // drawArmTargetLine(rightArmPosWorld, targetPosWorld);
                drawArmTargetLine(rightArmPosWorld, endPosWorld);
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

    void drawArmTargetLine(const osg::Vec3 &armBase, const osg::Vec3 &targetPos)
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
        return qRoll * qYaw * qPitch;
    }
};

COVERPLUGIN(GhostAvatar)