#ifndef COVER_PLUGIN_ANIMATION_H
#define COVER_PLUGIN_ANIMATION_H

#include <cover/coVRPluginSupport.h>
#include <cover/coVRFileManager.h>
#include <cover/ui/Owner.h>
#include <cover/ui/Action.h>
#include "../../general/ToolMachine/ToolChanger/Utility.h"

class AnimationPlugin : public opencover::coVRPlugin, opencover::ui::Owner
{
    public:
        AnimationPlugin();
    private:
        std::vector<opencover::ui::Action*> actions;
        opencover::ui::Menu *menu;
        void updateMenu();
        AnimationManagerFinder animFinder;

};

#endif // COVER_PLUGIN_ANIMATION_H