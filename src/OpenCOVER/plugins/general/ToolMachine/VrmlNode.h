#ifndef COVER_PLUGIN_TOOLMACHINE_VRMLNODE_H
#define COVER_PLUGIN_TOOLMACHINE_VRMLNODE_H

#include <vrml97/vrml/VrmlNodeChildTemplate.h>
#include <vrml97/vrml/VrmlNodeType.h>

#include <osg/MatrixTransform>

class LogicInterface {
public:
    virtual void update() = 0;
    virtual ~LogicInterface() = default;
};

class MachineNodeBase : public vrml::VrmlNodeChildTemplate {
public:
    MachineNodeBase(vrml::VrmlScene *scene);
    ~MachineNodeBase();
    static vrml::VrmlNodeType *defineType(vrml::VrmlNodeType *t);

    vrml::VrmlSFString machineName;
    vrml::VrmlSFString visualizationType;
    vrml::VrmlSFNode toolHeadNode;
    vrml::VrmlSFNode tableNode;
    vrml::VrmlMFVec3f axisOrientations;
    vrml::VrmlMFFloat offsets;
    vrml::VrmlMFString axisNames;
    vrml::VrmlSFString toolNumberName;
    vrml::VrmlSFString toolLengthName;
    vrml::VrmlSFString toolRadiusName;
    vrml::VrmlMFNode axisNodes;
    vrml::VrmlSFFloat opcUaToVrml;
    std::shared_ptr<LogicInterface> machine;

private:
    size_t m_index = 0; // used to remove the node from the machineNodes list
};

extern std::set<MachineNodeBase *> machineNodes;

class MachineNodeArrayMode : public MachineNodeBase {
public:
    static VrmlNode *creator(vrml::VrmlScene *scene);
    static vrml::VrmlNodeType *defineType(vrml::VrmlNodeType *t = nullptr);

    MachineNodeArrayMode(vrml::VrmlScene *scene);
    vrml::VrmlNodeType *nodeType() const override;
    vrml::VrmlNode *cloneMe() const override;
    vrml::VrmlSFString opcuaArrayName;
    vrml::VrmlMFInt opcuaAxisIndicees; // array mode expected
};

class MachineNodeSingleMode : public MachineNodeBase {
public:
    static VrmlNode *creator(vrml::VrmlScene *scene);
    static vrml::VrmlNodeType *defineType(vrml::VrmlNodeType *t = nullptr);

    MachineNodeSingleMode(vrml::VrmlScene *scene);
    vrml::VrmlNodeType *nodeType() const override;
    vrml::VrmlNode *cloneMe() const override;
    vrml::VrmlMFString opcuaNames; // axis names on the opcua server
};

class ToolChangerNode : public vrml::VrmlNodeChildTemplate {
public:
    static VrmlNode *creator(vrml::VrmlScene *scene);
    static vrml::VrmlNodeType *defineType(vrml::VrmlNodeType *t = nullptr);
    ToolChangerNode(vrml::VrmlScene *scene);
    ~ToolChangerNode();
    vrml::VrmlNodeType *nodeType() const override;
    vrml::VrmlNode *cloneMe() const override;
    vrml::VrmlSFString arm;
    vrml::VrmlSFString changer;
    vrml::VrmlSFString cover;
    vrml::VrmlSFNode toolHead;
    std::shared_ptr<LogicInterface> toolChanger;

private:
    size_t m_index = 0; // used to remove the node from the toolChanger list
};

extern std::set<ToolChangerNode *> toolChangers;

class MachineNode : public vrml::VrmlNodeChildTemplate { // dummy to load plugin
public:
    static VrmlNode *creator(vrml::VrmlScene *scene);
    static vrml::VrmlNodeType *defineType(vrml::VrmlNodeType *t = nullptr);
    MachineNode(vrml::VrmlScene *scene);

    vrml::VrmlNodeType *nodeType() const override;
    vrml::VrmlNode *cloneMe() const override;

private:
    size_t m_index = 0; // used to remove the node from the toolChanger list
};

osg::MatrixTransform *toOsg(vrml::VrmlNode *node);

#endif // COVER_PLUGIN_TOOLMACHINE_VRMLNODE_H
