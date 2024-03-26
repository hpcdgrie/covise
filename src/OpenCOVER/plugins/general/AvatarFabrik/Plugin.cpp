
#include "Plugin.h"
#include "Avatar.h"
#include <cover/coVRFileManager.h>
#include <cover/coVRCommunication.h>
#include <cover/coVRPartner.h>
#include <cover/VRAvatar.h>
#include <osgAnimation/UpdateBone>
#include <osgUtil/UpdateVisitor>

using namespace opencover;

COVERPLUGIN(AvatarPlugin);

AvatarPlugin::AvatarPlugin()
: coVRPlugin(COVER_PLUGIN_NAME)
, ui::Owner("AvatarPlugin", cover->ui)//braucht man für gui
, m_transform(new osg::MatrixTransform)
, m_sphereTransform(new osg::MatrixTransform)
, m_config(config())
, m_menu(new ui::Menu("Avatar",this))
{
    // auto loadPartnerAvatars = [this](){
        
    //     const auto me = coVRPartnerList::instance()->me();
    //     for(const auto &partner :  *coVRPartnerList::instance())
    //     {
    //         if(partner->ID() == me->ID() || partner->sessionID() != me->sessionID())
    //             continue;
    //         auto av = m_avatars.find(partner->ID());
    //         if(av == m_avatars.end())
    //         {
    //             auto avatar = std::make_unique<LoadedAvatar>();
    //             auto partnerAvatar = partner->getAvatar();

    //             if(avatar->loadAvatar(partner->userInfo().avatar, partnerAvatar, m_menu))
    //                 m_avatars[partner->ID()] = std::move(avatar);
    //         }    
    //     }
    // };
    // coVRCommunication::instance()->subscribeNotification(coVRCommunication::Notification::PartnerJoined, loadPartnerAvatars);
    // coVRCommunication::instance()->subscribeNotification(coVRCommunication::Notification::SessionChanged, [this, loadPartnerAvatars](){
    //     m_avatars.clear();
    //     loadPartnerAvatars();
    // });
    // coVRCommunication::instance()->subscribeNotification(coVRCommunication::Notification::PartnerLeft, [this](){
        
    //     const auto me = coVRPartnerList::instance()->me();
    //     for (auto a = m_avatars.begin(); a != m_avatars.end();)
    //     {
    //         auto p = coVRPartnerList::instance()->get(a->first);
    //         if(!p || p->sessionID() != me->sessionID())
    //         {
    //             a = m_avatars.erase(a);
    //         } else
    //             ++a;
    //     }
    // });
    // coVRCommunication::instance()->subscribeNotification(coVRCommunication::Notification::Disconnected, [this](){
    //     m_avatars.clear();
    // });



    m_avatar = std::make_unique<LoadedAvatar>();
    VRAvatar *a = new VRAvatar;
    if(m_avatar->loadAvatar("C:/Users/Dennis/Data/Avatare/Final/SimulierteWeltenAvatar2.fbx", a, m_menu))
        m_config->save();
    else
        m_avatar = nullptr;
    m_avatars[1] = std::move(m_avatar);

}

// void AvatarPlugin::loadAvatar()
// {
//     m_avatar = std::make_unique<LoadedAvatar>();
//     auto path = coVRFileManager::instance()->findOrGetFile(m_avatarFile->getValue());
//     if(m_avatar->loadAvatar(path, m_menu))
//         m_config->save();
//     else
//         m_avatar = nullptr;
// }

// void updateMatrixPosition(osg::MatrixTransform *mt, const osg::Vec3f &pos)
// {
//     auto m = mt->getMatrix();
//     m.setTrans(m.getTrans() + pos);
//     mt->setMatrix(m);
// }

// void AvatarPlugin::key(int type, int keySym, int mod)
// {
//     std::string key = "unknown";
//     if (!(keySym & 0xff00))
//     {
//         char buf[2] = { static_cast<char>(keySym), '\0' };
//         key = buf;
//     }
//     constexpr float speed = 50;
//     osg::Vec3f position;
//     if(key == "w")
//     {
//         position = osg::Vec3f(speed,0,0);
//     } else if (key == "a")
//     {
//         position = osg::Vec3f(0, 0, speed);
//     }
//     else if (key == "s")
//     {
//         position = osg::Vec3f(-speed,0,0);
//     }
//     else if (key == "d")
//     {
//         position = osg::Vec3f(0, 0, -speed);
//     }
//     else if (key == "e")
//     {
//         position = osg::Vec3f(0, speed, 0);
//     }
//     else if (key == "r")
//     {
//         position = osg::Vec3f(0, -speed, 0);
//     }
//     updateMatrixPosition(m_sphereTransform, position);

// }

bool AvatarPlugin::update(){

    for(auto &avatar : m_avatars)
    {
        avatar.second->update();
    }
    // if(!m_avatar)
    //     return true;
    // osg::Matrix m = osg::Matrix::identity();
    // auto pos = m_interactor->getPos();
    // m.setTrans(pos);
    // m_sphereTransform->setMatrix(m);
    // m_avatar->update(pos);
    return true;
}

// void AvatarPlugin::preFrame()
// {
//     m_interactor->preFrame();
// }
