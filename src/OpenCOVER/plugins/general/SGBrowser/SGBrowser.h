/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _SGBROWSER_PLUGIN_H
#define _SGBROWSER_PLUGIN_H
/****************************************************************************\ 
 **                                                            (C)2007/8 HLRS **
 **                                                                           **
 ** Description: Scenegraph Browser											            **
 **																		                     **
 **                                                                           **
 ** Author: Mario Baalcke	                                                   **
 **                                                                           **
 ** History:  								                                          **
 ** Jun-07   v1	    				       		                                 **
 ** April-08 v2                                                               **
 **                                                                           **
\****************************************************************************/
#include <cover/coVRPluginSupport.h>
#include <cover/coVRPlugin.h>

#include <cover/coVRSelectionManager.h>
#include <cover/coVRShader.h>

#include <map>
#include <memory>
#include <PluginUtil/coVR3DTransformInteractor.h>

#include "vrml97/vrml/VrmlNodeTexture.h"
#include "vrml97/vrml/VrmlMFString.h"
#include "vrml97/vrml/VrmlSFBool.h"
#include "vrml97/vrml/VrmlSFInt.h"

#include "vrml97/vrml/Viewer.h"
#include "cover/coTabletUI.h"

#include <util/coTabletUIMessages.h>
#include <util/coRestraint.h>

using namespace covise;
using namespace opencover;

struct LItem
{
    osg::Image *image;
    int index;
};

class MyNodeVisitor : public osg::NodeVisitor
{
public:
    MyNodeVisitor(TraversalMode tm, coTUISGBrowserTab *sGBrowserTab);
    void apply(osg::Node &node);

    void updateMyParent(osg::Node &node);
    void updateMyChild(osg::Node *node);
    void myUpdate(osg::Node *node);
    void traverseMyParents(osg::Node *node);
    void traverseFindList();
    void sendMyFindList();

    void addMyNode();
    void myInit();

protected:
    coTUISGBrowserTab *sGBrowserTab;
    coVRSelectionManager *selectionManager;

private:
    osg::Node *ROOT;
    osg::Node *selectNode;
    osg::Node *selectParentNode;
    osg::Group *mySelGroupNode;

    std::list<osg::Node *> selNodeList;
    std::list<osg::Node *> selParentList;
    std::list<osg::Group *> selGroupList;
    std::list<osg::Node *> findNodeList;
    std::list<osg::Node *> sendFindList;
    std::list<osg::Node *> sendCurrentList;
};
class TexVisitor : public osg::NodeVisitor
{
public:
    TexVisitor(TraversalMode tm, coTUISGBrowserTab *TextureTab);
    ~TexVisitor();
    void apply(osg::Node &node);
    void clearImageList()
    {
        imageListe.clear();
    };
    void insertImage(LItem item)
    {
        imageListe.push_back(item);
    };
    int findImage(osg::Image *);
    LItem *getLItem(osg::Image *);
    int getListSize()
    {
        return imageListe.size();
    };
    bool getTexFound()
    {
        return texFound;
    };
    void setTexFound(bool state)
    {
        texFound = state;
    };
    int getMax();
    void processStateSet(osg::StateSet *ss);
    osg::Image *getImage(int index);
    void sendImageList();

protected:
    coTUISGBrowserTab *texTab;
    std::vector<LItem> imageListe;

private:
    bool texFound;
};

class SGBrowser : public coVRPlugin, public coTUIListener, public coSelectionListener
{
public:
    SGBrowser();
    ~SGBrowser() override;
    bool init() override;

    bool selectionChanged() override;
    bool pickedObjChanged() override;
    //_____________________________this will be called in PreFrame_____________________________
    void preFrame() override;
    void applyInteraction(std::pair<osg::MatrixTransform *const, std::unique_ptr<opencover::coVR3DTransformInteractor>> &pair);
    void message(int toWhom, int type, int len, const void *buf) override;
    void removeNode(osg::Node *node, bool isGroup, osg::Node *realNode) override;
    void addNode(osg::Node *node, const RenderObject *obj) override;
    bool processTexture(coTUISGBrowserTab *sGBrowserTab, TexVisitor *texvis, osg::StateSet *ss);

    virtual void tabletPressEvent(coTUIElement *tUIItem) override;
    virtual void tabletReleaseEvent(coTUIElement *tUIItem) override;
    virtual void tabletSelectEvent(coTUIElement *tUIItem) override;
    virtual void tabletChangeModeEvent(coTUIElement *tUIItem) override;
    virtual void tabletFindEvent(coTUIElement *tUIItem) override;
    virtual void tabletEvent(coTUIElement *tUIItem) override;
    virtual void tabletCurrentEvent(coTUIElement *tUIItem) override;
    virtual void tabletDataEvent(coTUIElement *tUIItem, TokenBuffer &tb) override;
    virtual void tabletLoadFilesEvent(char *nodeName) override;

    //   virtual void hideNode();
    static SGBrowser *plugin;

private:
    char *idata;
    bool myMes;
    //_____________________________the plugin tab__________________________________________________________

    coVRSelectionManager *selectionManager;
#if 0
    std::vector<coTUISGBrowserTab *> sGBrowserTab;
    std::vector<MyNodeVisitor *> vis;
    std::vector<TexVisitor *> texvis;
#endif

    coRestraint *restraint;
    coVRShaderList *shaderList;

    struct TuiData
    {
        TuiData(coTUISGBrowserTab *tab, MyNodeVisitor *vis, TexVisitor *tex)
            : tab(tab), vis(vis), tex(tex)
        {}

        coTUISGBrowserTab *tab;
        MyNodeVisitor *vis;
        TexVisitor *tex;
    };

    std::vector<TuiData> tuis;

    bool linked;

    osg::ref_ptr<osg::Node> pickedObject;
    std::set<osg::ref_ptr<osg::Switch>> nodesToRemove;
    // map from MatrixTransform node to its 3D interactor
    std::map<osg::MatrixTransform *, std::unique_ptr<opencover::coVR3DTransformInteractor>> m_transformInteractors;

    // helper to create/destroy interactor for a MatrixTransform
    std::map<osg::MatrixTransform *, std::unique_ptr<opencover::coVR3DTransformInteractor>>::iterator addInteractorFor(osg::MatrixTransform *mt);
    void removeInteractorFor(osg::MatrixTransform *mt);
};
#endif
