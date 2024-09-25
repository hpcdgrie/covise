#include "Animation.h"
#include <cover/ui/Menu.h>

COVERPLUGIN(AnimationPlugin)
using namespace opencover;

AnimationPlugin::AnimationPlugin()
: coVRPlugin(COVER_PLUGIN_NAME) 
, ui::Owner("AnimationPlugin", cover->ui)

{
    menu = new ui::Menu("Animation", this);
    auto update = new ui::Action(menu, "update");
    update->setCallback([this](){
        updateMenu();
    });

}

void AnimationPlugin::updateMenu()
{
    actions.clear();
    cover->getObjectsRoot()->accept(animFinder);
    for(auto &a : animFinder.m_am->getAnimationList())
    {
        auto action = new ui::Action(menu, a->getName());
        action->setCallback([a, this](){
            a->setPlayMode(osgAnimation::Animation::ONCE);
            animFinder.m_am->playAnimation(a, 1, 1);
        });
        actions.push_back(action);
    }
}