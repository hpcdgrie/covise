#ifndef COVER_PLUGIN_FBXAVATAR_PLUGIN_H
#define COVER_PLUGIN_FBXAVATAR_PLUGIN_H

#include <cover/coVRPluginSupport.h>
#include <PluginUtil/coVR3DTransRotInteractor.h>

#include <fabrik.h>

class PLUGINEXPORT FabrikPlugin : public opencover::coVRPlugin
{
public:

    FabrikPlugin();
private:
    std::unique_ptr<opencover::coVR3DTransRotInteractor> m_interactor;
    Skeleton m_skeleton;
    std::vector<osg::ref_ptr<osg::MatrixTransform>> m_boneDummies;
    // void loadAvatar();
    // void key(int type, int keySym, int mod) override;
    bool update() override; //wird in jedem Frame aufgerufen
    // void preFrame() override;

};

#endif // COVER_PLUGIN_FBXAVATAR_PLUGIN_H
