#include "AnimatedAvatarPlugin.h"

#include <cover/coVRCommunication.h>
#include <cover/coVRPartner.h>
#include <cover/coVRPluginSupport.h>
#include <cover/ui/FileBrowser.h>
#include <cover/ui/Owner.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Menu.h>

#include <algorithm>
#include <map>
#include <vector>

#include <osg/MatrixTransform>
#include <osgCal/CoreModel>
#include <osgCal/Model>

using namespace covise;
using namespace opencover;
using namespace ui;



AnimatedAvatarPlugin::AnimatedAvatarPlugin()
: coVRPlugin(COVER_PLUGIN_NAME)
, Owner(COVER_PLUGIN_NAME, cover->ui)
{
   ui::Menu *menu = new ui::Menu("AnimatedAvatar", this);
   m_singleModeButton = new ui::Button(menu, "Single_mode");
}

bool AnimatedAvatarPlugin::update()
{
    addPartner();

    removeAvatar();

    // Avatar-Update durchführen
    for (auto& avatar : m_avatars)
    {
        avatar.second->update();
    }

    return true;
}

void AnimatedAvatarPlugin::removeAvatar()
{
    auto partnerList = coVRPartnerList::instance();
    for (auto avatarIt = m_avatars.begin(); avatarIt != m_avatars.end(); avatarIt++)
    {
        auto &avatar = *avatarIt;
        int partnerId = avatar.first;
        auto partner = std::find_if(partnerList->begin(), partnerList->end(), [partnerId](const std::unique_ptr<coVRPartner> &partner){
            return partner->ID() == partnerId;
        });
        if (partner == partnerList->end() || (partnerId == coVRCommunication::instance()->getID() && !m_singleModeButton->state()))
        {
            avatarIt = m_avatars.erase(avatarIt);
            return;
        }
    }
}

void AnimatedAvatarPlugin::addPartner()
{
    auto partnerList = coVRPartnerList::instance();
    for (const auto& partner : *partnerList)
    {
        int partnerId = partner->ID();

        if(partnerId == coVRCommunication::instance()->getID() && !m_singleModeButton->state())
            continue;
        // Avatar bereits vorhanden?
        if (m_avatars.find(partnerId) != m_avatars.end())
        {
            // Avatar bereits vorhanden, überspringen
            continue;
        }
        std::string modelFilename = partner->userInfo().avatar;
        // Avatar erstellen und in m_avatars einfügen
        m_avatars[partnerId] = std::make_unique<AnimatedAvatar>(modelFilename, partnerId);
    }
}

COVERPLUGIN(AnimatedAvatarPlugin)
