#include "VrmlToolChanger.h"
#include <cassert>

#include <plugins/general/Vrml97/ViewerObject.h>

using namespace vrml;

std::set<ToolChangerNode *> toolChangers;


// ToolChangerNode

void ToolChangerNode::initFields(ToolChangerNode *node, VrmlNodeType *t) {
    initFieldsHelper(node, t,
        namedValue("arm", node->arm),
        namedValue("changer", node->changer),
        namedValue("cover", node->cover),
        namedValue("toolHeadNode", node->toolHead)
    );
}

ToolChangerNode::ToolChangerNode(VrmlScene *scene)
: VrmlNodeChildTemplateTemplate<ToolChangerNode>(scene)
{
    initFields(this, nullptr);
    toolChangers.emplace(this);
}

ToolChangerNode::~ToolChangerNode()
{
    toolChangers.erase(this);
}

osg::MatrixTransform *toOsg(VrmlNode *node)
{
    auto g = node->toGroup();
    if(!g)
        return nullptr;
    auto vo = g->getViewerObject();
    if(!vo)
        return nullptr;
    auto pNode = ((osgViewerObject *)vo)->pNode;
    if(!pNode)
        return nullptr;
    auto trans = pNode->asTransform();
    if(!trans)
        return nullptr;
    return trans->asMatrixTransform();
}