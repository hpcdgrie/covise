#ifndef COVER_PLUGIN_FBXAVATAR_PLUGIN_H
#define COVER_PLUGIN_FBXAVATAR_PLUGIN_H

#include <memory>
#include <vector>
#include <cover/coVRPluginSupport.h>
#include <PluginUtil/coVR3DTransRotInteractor.h>

#include "ConnectionLine.h"
#include <fabrik.h>
class PLUGINEXPORT FabrikPlugin : public opencover::coVRPlugin
{
public:

    FabrikPlugin();
    FabrikPlugin(const FabrikPlugin &) = delete; // Copy constructor
    FabrikPlugin &operator=(const FabrikPlugin &) = delete; // Copy assignment operator
private:
    std::vector<std::unique_ptr<opencover::coVR3DTransRotInteractor>> m_interactors;
    Skeleton m_skeleton;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> m_boneDummies;
    // void loadAvatar();
    // void key(int type, int keySym, int mod) override;
    bool update() override; //wird in jedem Frame aufgerufen
    // void preFrame() override;
    std::vector<std::unique_ptr<ConnectionLine>> m_connectionLines;

};

#endif // COVER_PLUGIN_FBXAVATAR_PLUGIN_H
