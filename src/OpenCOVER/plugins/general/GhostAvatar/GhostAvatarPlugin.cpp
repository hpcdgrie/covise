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
                m_avatars.push_back(std::make_unique<GhostAvatar>(partner->ID(), m_adjustMatrix));
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
            if(partner->ID() == coVRPartnerList::instance()->me()->ID())
                continue;
            m_avatars.push_back(std::make_unique<GhostAvatar>(partner->ID(), m_adjustMatrix));
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
                m_adjustMatrix(row, 2) = v.z(); });
                for(auto &avatar : m_avatars)
                {
                    avatar->setAdjustMatrix(m_adjustMatrix);
                }
    }
    for(auto &avatar : m_avatars)
    {
        avatar->setAdjustMatrix(m_adjustMatrix);
    }
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
