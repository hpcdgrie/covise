#include "VrmlMachine.h"
#include <cassert>

#include <plugins/general/Vrml97/ViewerObject.h>

using namespace vrml;

std::set<MachineNodeBase *> machineNodes;

void MachineNodeBase::initFields(vrml::VrmlNodeChildTemplate *base, MachineNodeBase *node, VrmlNodeType *t) {
    initFieldsHelper(base, t,
        namedValue("machineName", node->machineName),
        namedValue("visualizationType", node->visualizationType),
        namedValue("toolHeadNode", node->toolHeadNode),
        namedValue("tableNode", node->tableNode),
        namedValue("axisOrientations", node->axisOrientations),
        namedValue("offsets", node->offsets),
        namedValue("axisNames", node->axisNames),
        namedValue("toolNumberName", node->toolNumberName),
        namedValue("toolLengthName", node->toolLengthName),
        namedValue("toolRadiusName", node->toolRadiusName),
        namedValue("axisNodes", node->axisNodes),
        namedValue("opcUaToVrml", node->opcUaToVrml)
    );
}

MachineNodeBase::MachineNodeBase()
{
    machineNodes.emplace(this);
}

MachineNodeBase::~MachineNodeBase()
{
    machineNodes.erase(this);
}

// array mode
void MachineNodeArrayMode::initFields(MachineNodeArrayMode *node, VrmlNodeType *t) {
    
    MachineNodeBase::initFields(node, node, t);
    initFieldsHelper(node, t,
        namedValue("opcuaAxisIndicees", node->opcuaAxisIndicees),
        namedValue("opcuaArrayName", node->opcuaArrayName)
    );

}

MachineNodeArrayMode::MachineNodeArrayMode(VrmlScene *scene)
: VrmlNodeChildTemplateTemplate(scene)
{
    initFields(this, nullptr);
}

// single mode


MachineNodeSingleMode::MachineNodeSingleMode(VrmlScene *scene)
: VrmlNodeChildTemplateTemplate(scene)
{
    initFields(this, nullptr);
}

void MachineNodeSingleMode::initFields(MachineNodeSingleMode *node, VrmlNodeType *t) {
    MachineNodeBase::initFields(node, node, t);
    initFieldsHelper(node, t,
        namedValue("opcuaNames", node->opcuaNames)
    );
}

// MachineNode dummy

MachineNode::MachineNode(VrmlScene *scene)
: VrmlNode(scene)
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
    return t;
}
