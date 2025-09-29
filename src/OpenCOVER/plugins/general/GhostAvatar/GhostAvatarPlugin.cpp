#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/ShapeDrawable>
#include <osgDB/ReadFile>

#include "GhostAvatarPlugin.h"
#include "GhostAvatar.h"

#include <cover/coVRPluginSupport.h>
#include <cover/coVRPartner.h>
#include <cover/coVRCommunication.h>
using namespace covise;
using namespace opencover;
using namespace ui;



GhostAvatarPlugin::GhostAvatarPlugin()
    : coVRPlugin(COVER_PLUGIN_NAME), Owner(COVER_PLUGIN_NAME, cover->ui), m_mainMenu(new ui::Menu("GhostAvatar", this))
{
    createSettingsMenu();
    createDebugMenu();
    m_permFix.set(-1,0,0,0,
                0,1,0,0,
                0,0,-1,0,
                0,0,0,1); // invert X and Z axes
    coVRCommunication::instance()->subscribeNotification(coVRCommunication::Notification::PartnerJoined, [this]() {
        std::cerr << "Partner joined\n";
        // check for new partners and add avatars
        for (const auto &partner : *coVRPartnerList::instance())
        {
            if(partner->ID() == coVRPartnerList::instance()->me()->ID())
                continue;
            auto it = std::find_if(m_avatars.begin(), m_avatars.end(), [&partner](const std::unique_ptr<GhostAvatar> &avatar) {
                return avatar->ID() == partner->ID();
            });
            if (it == m_avatars.end())
                m_avatars.push_back(std::make_unique<GhostAvatar>(partner->ID(), m_adjustMatrix, m_permFix));
        }
    });

    coVRCommunication::instance()->subscribeNotification(coVRCommunication::Notification::PartnerLeft, [this]() {
        std::cerr << "Partner left\n";
        // check for removed partners and remove avatars
        auto it = m_avatars.begin();
        while (it != m_avatars.end())   
        {
            auto partner = coVRPartnerList::instance()->get((*it)->ID());
            if (!partner)
            {
                it = m_avatars.erase(it);
            }
            else
            {
                ++it;
            }
        }
    });

    coVRCommunication::instance()->subscribeNotification(coVRCommunication::Notification::SessionChanged, [this]() {
        std::cerr << "Session changed\n";
        // check for removed partners and remove avatars
        m_avatars.clear();  
        for(const auto &partner : *coVRPartnerList::instance())
        {
            if(partner->ID() == coVRPartnerList::instance()->me()->ID() || partner->sessionID() != coVRPartnerList::instance()->me()->sessionID())
                continue;
            m_avatars.push_back(std::make_unique<GhostAvatar>(partner->ID(), m_adjustMatrix, m_permFix));
        }
    });


}

bool GhostAvatarPlugin::update()
{
    for(auto &avatar : m_avatars)
        avatar->update();
    return true;
}

void GhostAvatarPlugin::createSettingsMenu()
{
    m_settingsMenu = new ui::Menu(m_mainMenu, "Settings");
    m_tabletUINote = new ui::Action(m_settingsMenu, "Changes can only be made in the TabletUI!");

    createArmBaseVectorMenu();
    createAdjustMatrixMenu();
    createPermFixMenu();
}

void GhostAvatarPlugin::createArmBaseVectorMenu()
{
    m_armBaseVecMenu = new ui::Menu(m_settingsMenu, "Arm Base Vector");
    m_armBaseVecField = new ui::VectorEditField(m_armBaseVecMenu, "Vector");
    m_armBaseVecField->setValue({0, 1, 0});
    m_armBaseVecField->setCallback([this](const osg::Vec3 &dir)
                                   {
                                    for(auto &avatar : m_avatars)
                                        avatar->setArmBaseVector(dir);
                                   });
}

void GhostAvatarPlugin::createAdjustMatrixMenu()
{
    m_adjustMatrixMenu = new ui::Menu(m_settingsMenu, "Adjust Matrix");

    // set correct axis conventions for the GhostAvatar model
    
    if (ARM_NODE_NAME == "LeftArm")
    {
        m_adjustMatrix.set(
            1, 0, 0, 0,
            0, 0, -1, 0,
            0, 1, 0, 0,
            0, 0, 0, 1);
    }
    else if (ARM_NODE_NAME == "RightArm")
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
                m_adjustMatrix(row, 2) = v.z();
                for(auto &avatar : m_avatars)
                {
                    avatar->setAdjustMatrix(m_adjustMatrix);
                }});
    }
    for(auto &avatar : m_avatars)
    {
        avatar->setAdjustMatrix(m_adjustMatrix);
    }
}

void GhostAvatarPlugin::createPermFixMenu()
{
    m_permFixMenu = new ui::Menu(m_settingsMenu, "Axis Permutation (permFix)");

    // Default to identity; users can change rows to select which source axis maps to target X/Y/Z
    // Provide a helpful initial mapping comment in UI names
    for (int row = 0; row < 3; ++row)
    {
        osg::Vec3 rowVec(m_permFix(row, 0), m_permFix(row, 1), m_permFix(row, 2));
        std::string label = std::string("Row ") + std::to_string(row) + " (target " + (row==0?"X":row==1?"Y":"Z") + ")";
        m_permFixVecFields[row] = new ui::VectorEditField(m_permFixMenu, label);
        m_permFixVecFields[row]->setValue(rowVec);
        m_permFixVecFields[row]->setCallback([this, row](const osg::Vec3 &v)
                                             {
            // Keep last column as 0 and bottom row untouched (Matrix is affine with last row 0,0,0,1)
            m_permFix(row, 0) = v.x();
            m_permFix(row, 1) = v.y();
            m_permFix(row, 2) = v.z();
            // Enforce last column row-wise and last row for safety
            m_permFix(0,3) = 0; m_permFix(1,3) = 0; m_permFix(2,3) = 0;
            m_permFix(3,0) = 0; m_permFix(3,1) = 0; m_permFix(3,2) = 0; m_permFix(3,3) = 1;
            // Also update axis-angle representation to match new matrix (extracting axis-angle can be ambiguous; we leave axis & angle untouched for now)
            for (auto &avatar : m_avatars)
                avatar->setPermFix(m_permFix);
        });
    }

    // Ensure bottom row is [0 0 0 1]
    m_permFix(3,0) = 0; m_permFix(3,1) = 0; m_permFix(3,2) = 0; m_permFix(3,3) = 1;
    for (auto &avatar : m_avatars)
        avatar->setPermFix(m_permFix);

    // Axis-angle controls
    m_permAxisField = new ui::VectorEditField(m_permFixMenu, "Rotation Axis (X,Y,Z)");
    m_permAxisField->setValue(m_permAxis);
    m_permAxisField->setCallback([this](const osg::Vec3 &axis){
        m_permAxis = axis;
        recomputePermFixFromAxisAngle();
    });

    m_permAngleSlider = new ui::Slider(m_permFixMenu, "Rotation Angle (deg)");
    m_permAngleSlider->setBounds(-360.0, 360.0);
    m_permAngleSlider->setValue(m_permAngleDeg);
    m_permAngleSlider->setCallback([this](double val, bool){
        m_permAngleDeg = val;
        recomputePermFixFromAxisAngle();
    });
}

void GhostAvatarPlugin::recomputePermFixFromAxisAngle()
{
    osg::Vec3 axis = m_permAxis;
    // If axis is zero, keep identity and avoid NaNs
    if (axis.length2() < 1e-12)
    {
        m_permFix.makeIdentity();
    }
    else
    {
        axis.normalize();
        double rad = m_permAngleDeg * M_PI / 180.0;
        osg::Quat q(rad, axis);
        osg::Matrix rot(q);
        // Ensure pure rotation in top-left 3x3 and affine bottom row/col
        m_permFix = rot;
        m_permFix(0,3) = 0; m_permFix(1,3) = 0; m_permFix(2,3) = 0;
        m_permFix(3,0) = 0; m_permFix(3,1) = 0; m_permFix(3,2) = 0; m_permFix(3,3) = 1;
    }

    // Update row UI fields to reflect the new matrix
    for (int row = 0; row < 3; ++row)
    {
        osg::Vec3 rowVec(m_permFix(row,0), m_permFix(row,1), m_permFix(row,2));
        if (m_permFixVecFields[row])
            m_permFixVecFields[row]->setValue(rowVec);
    }
    // Push to avatars
    for (auto &avatar : m_avatars)
        avatar->setPermFix(m_permFix);
}

void GhostAvatarPlugin::createDebugMenu()
{
    m_debugMenu = new ui::Menu(m_mainMenu, "Debugging");

    m_showTargetLine = new ui::Button(m_debugMenu, "Show Target Line");
    m_showTargetLine->setState(false);
    m_showTargetLine->setCallback([this](bool state)
                                  { for(auto &avatar : m_avatars) avatar->showTargetLine(state); });

    m_showFrames = new ui::Button(m_debugMenu, "Show Frames");
    m_showFrames->setState(false);
    m_showFrames->setCallback([this](bool state)
                              { for(auto &avatar : m_avatars) avatar->showFrames(state); });

    m_axisNote = new ui::Action(m_debugMenu, "x - red, y - green, z - blue");
    m_axisNote->setEnabled(false);
}
