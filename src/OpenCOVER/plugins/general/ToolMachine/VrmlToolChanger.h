#ifndef COVER_PLUGIN_TOOLCHANGER_VRMLNODE_H
#define COVER_PLUGIN_TOOLCHANGER_VRMLNODE_H

#include "LogicInterface.h"

#include <vrml97/vrml/VrmlNodeChildTemplate.h>
#include <vrml97/vrml/VrmlNodeType.h>

#include <osg/MatrixTransform>
#include <utils/pointer/ResetOnCopy.h>



class ToolChangerNode : public vrml::VrmlNodeChildTemplateTemplate<ToolChangerNode> {
public:
    ToolChangerNode(vrml::VrmlScene *scene);
    ~ToolChangerNode();
    static void initFields(ToolChangerNode *node, vrml::VrmlNodeType *t);
    static const char *name() { return "ToolChangerNode"; }

    vrml::VrmlSFString arm;
    vrml::VrmlSFString changer;
    vrml::VrmlSFString cover;
    vrml::VrmlSFNode toolHead;

    opencover::utils::pointer::ResetOnCopyPtr<LogicInterface> toolChanger;

};

extern std::set<ToolChangerNode *> toolChangers;


osg::MatrixTransform *toOsg(vrml::VrmlNode *node);

#endif // COVER_PLUGIN_TOOLCHANGER_VRMLNODE_H
