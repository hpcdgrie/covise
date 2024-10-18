#include "VrmlNode.h"
#include <cassert>

#include <plugins/general/Vrml97/ViewerObject.h>

using namespace vrml;

std::set<MachineNodeBase *> machineNodes;
std::set<ToolChangerNode *> toolChangers;

void initFields(MachineNodeBase *node, VrmlNodeType *t) {
    initFieldsHelper(node, t,
        namedValue("machineName", &node->machineName),
        namedValue("visualizationType", &node->visualizationType),
        namedValue("toolHeadNode", &node->toolHeadNode),
        namedValue("tableNode", &node->tableNode),
        namedValue("axisOrientations", &node->axisOrientations),
        namedValue("offsets", &node->offsets),
        namedValue("axisNames", &node->axisNames),
        namedValue("toolNumberName", &node->toolNumberName),
        namedValue("toolLengthName", &node->toolLengthName),
        namedValue("toolRadiusName", &node->toolRadiusName),
        namedValue("axisNodes", &node->axisNodes),
        namedValue("opcUaToVrml", &node->opcUaToVrml)
    );
}

MachineNodeBase::MachineNodeBase(VrmlScene *scene)
: VrmlNodeChildTemplate(scene), m_index(machineNodes.size())
{
    initFields(this, nullptr);
    machineNodes.emplace(this);
}

MachineNodeBase::~MachineNodeBase()
{
    machineNodes.erase(this);
}

VrmlNodeType *MachineNodeBase::defineType(VrmlNodeType *t)
{
    assert(t);
    std::cerr << "toolmachine node define type" << std::endl;
    VrmlNodeChildTemplate::defineType(t); // Parent class
    initFields(nullptr, t);
    return t;
}

// array mode
namespace arrayMode{
void initFields(MachineNodeArrayMode *node, VrmlNodeType *t) {
    initFieldsHelper(node, t,
        namedValue("opcuaAxisIndicees", &node->opcuaAxisIndicees),
        namedValue("opcuaArrayName", &node->opcuaArrayName)
    );
}
}

MachineNodeArrayMode::MachineNodeArrayMode(VrmlScene *scene)
: MachineNodeBase(scene)
{
    arrayMode::initFields(this, nullptr);
}

VrmlNodeType *MachineNodeArrayMode::defineType(VrmlNodeType *t)
{
    static VrmlNodeType *st = 0;
    if (!t)
    {
        if (st)
            return st; // Only define the type once.
        t = st = new VrmlNodeType("MachineNodeArrayMode", creator);
    }

    MachineNodeBase::defineType(t); // Parent class
    
    arrayMode::initFields(nullptr, t);

    return t;
}

VrmlNode *MachineNodeArrayMode::creator(VrmlScene *scene)
{
    return new MachineNodeArrayMode(scene);
}

VrmlNodeType *MachineNodeArrayMode::nodeType() const { return defineType(); };

VrmlNode *MachineNodeArrayMode::cloneMe() const 
{
    return new MachineNodeArrayMode(*this);
}

// single mode

namespace singleMode{
void initFields(MachineNodeSingleMode *node, VrmlNodeType *t) {
    initFieldsHelper(node, t,
        namedValue("opcuaNames", &node->opcuaNames)
    );
}
}

MachineNodeSingleMode::MachineNodeSingleMode(VrmlScene *scene)
: MachineNodeBase(scene)
{
    singleMode::initFields(this, nullptr);
}

VrmlNodeType *MachineNodeSingleMode::defineType(VrmlNodeType *t)
{
    static VrmlNodeType *st = 0;
    if (!t)
    {
        if (st)
            return st; // Only define the type once.
        t = st = new VrmlNodeType("MachineNodeSingleMode", creator);
    }

    MachineNodeBase::defineType(t); // Parent class
    
    singleMode::initFields(nullptr, t);

    return t;
}

VrmlNode *MachineNodeSingleMode::creator(VrmlScene *scene)
{
    return new MachineNodeSingleMode(scene);
}

VrmlNodeType *MachineNodeSingleMode::nodeType() const { return defineType(); };

VrmlNode *MachineNodeSingleMode::cloneMe() const 
{
    return new MachineNodeSingleMode(*this);
}

// ToolChangerNode

namespace toolChanger{
void initFields(ToolChangerNode *node, VrmlNodeType *t) {
    initFieldsHelper(node, t,
        namedValue("arm", &node->arm),
        namedValue("changer", &node->changer),
        namedValue("cover", &node->cover),
        namedValue("toolHeadNode", &node->toolHead)
    );
}
}

ToolChangerNode::ToolChangerNode(VrmlScene *scene)
: VrmlNodeChildTemplate(scene), m_index(toolChangers.size())
{
    toolChanger::initFields(this, nullptr);
    toolChangers.emplace(this);
}

ToolChangerNode::~ToolChangerNode()
{
    toolChangers.erase(this);
}

VrmlNode *ToolChangerNode::creator(VrmlScene *scene)
{
    return new ToolChangerNode(scene);
}

VrmlNodeType *ToolChangerNode::nodeType() const { return defineType(); };

VrmlNode *ToolChangerNode::cloneMe() const 
{
    return new ToolChangerNode(*this);
}

VrmlNodeType *ToolChangerNode::defineType(VrmlNodeType *t)
{
    static VrmlNodeType *st = 0;
    if (!t)
    {
        if (st)
            return st; // Only define the type once.
        t = st = new VrmlNodeType("ToolChangerNode", creator);
    }

    MachineNodeBase::defineType(t); // Parent class
    
    toolChanger::initFields(nullptr, t);

    return t;
}

// MachineNode dummy

MachineNode::MachineNode(VrmlScene *scene)
: VrmlNodeChildTemplate(scene), m_index(toolChangers.size())
{}

VrmlNode *MachineNode::creator(VrmlScene *scene)
{
    return new MachineNode(scene);
}

VrmlNodeType *MachineNode::nodeType() const { return defineType(); };

VrmlNode *MachineNode::cloneMe() const 
{
    return new MachineNode(*this);
}

VrmlNodeType *MachineNode::defineType(VrmlNodeType *t)
{
    static VrmlNodeType *st = 0;
    std::cerr << "defining ToolMachine type " << std::endl;
    if (!t)
    {
        if (st)
            return st; // Only define the type once.
        t = st = new VrmlNodeType("ToolMachine", creator);
    }

    MachineNodeBase::defineType(t); // Parent class
    
    return t;
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