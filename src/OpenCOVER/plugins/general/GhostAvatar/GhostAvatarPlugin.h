#ifndef COVER_PLUGIN_GHOSTAVATAR_GhostAvatarPlugin_H
#define COVER_PLUGIN_GHOSTAVATAR_GhostAvatarPlugin_H

#include <array>
#include <memory>

#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/ref_ptr>

#include <cover/coVRPluginSupport.h>
#include <cover/ui/Action.h>
#include <cover/ui/Button.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Owner.h>
#include <cover/ui/VectorEditField.h>

#include "Bone.h"

class GhostAvatar;

class GhostAvatarPlugin : public opencover::coVRPlugin, public opencover::ui::Owner
{
public:
    GhostAvatarPlugin();

    bool update() override;


private:

    std::vector<std::unique_ptr<GhostAvatar>> m_avatars;
    // UI elements
    opencover::ui::Menu *m_mainMenu = nullptr;

    opencover::ui::Menu *m_settingsMenu = nullptr;
    opencover::ui::Action *m_tabletUINote = nullptr;
    opencover::ui::Menu *m_armBaseVecMenu = nullptr;
    opencover::ui::VectorEditField *m_armBaseVecField = nullptr;
    opencover::ui::Menu *m_adjustMatrixMenu = nullptr;
    std::array<opencover::ui::VectorEditField *, 3> m_adjustMatrixVecFields;

    opencover::ui::Menu *m_debugMenu = nullptr;
    opencover::ui::Button *m_showFrames = nullptr;
    opencover::ui::Button *m_showTargetLine = nullptr;
    opencover::ui::Action *m_axisNote = nullptr;
    osg::Matrix m_adjustMatrix;
    // methods to create UI elements
    void createSettingsMenu();
    void createArmBaseVectorMenu();
    void createAdjustMatrixMenu();
    void createDebugMenu();
};

COVERPLUGIN(GhostAvatarPlugin)

#endif // COVER_PLUGIN_GHOSTAVATAR_GhostAvatarPlugin_H