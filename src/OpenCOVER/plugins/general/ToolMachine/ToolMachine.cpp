#include "ToolMachine.h"

#include <vrml97/vrml/VrmlNode.h>
#include <vrml97/vrml/VrmlNodeTransform.h>
#include <vrml97/vrml/VrmlNodeType.h>
#include <vrml97/vrml/VrmlNamespace.h>
#include <vrml97/vrml/VrmlSFVec3f.h>
#include <vrml97/vrml/VrmlNodeChild.h>
#include <util/coExport.h>
#include <vrml97/vrml/VrmlScene.h>
#include <cover/ui/Slider.h>
#include <cover/ui/Menu.h>

#include <stdlib.h>
#include <cover/ui/VectorEditField.h>

#include <PluginUtil/OpcUaClient/opcua.h>

using namespace covise;
using namespace opencover;
using namespace vrml;

class MachineNode;
std::vector<MachineNode *> machineNodes;
const std::array<const char*, 5> axisNames{"A", "C", "X", "Y", "Z"};
const std::array<const char*, 5> axisNamesLower{"a", "c", "x", "y", "z"};

static VrmlNode *creator(VrmlScene *scene);

class MachineNode : public vrml::VrmlNodeChild
{
public:
    static VrmlNode *creator(VrmlScene *scene)
    {
        return new MachineNode(scene);
    }
    MachineNode(VrmlScene *scene) : VrmlNodeChild(scene), m_index(machineNodes.size())
    {

        std::cerr << "vrml Machine node created" << std::endl;
        machineNodes.push_back(this);
    }
    ~MachineNode()
    {
        machineNodes.erase(machineNodes.begin() + m_index);
    }

    static VrmlNodeType *defineType(VrmlNodeType *t = 0)
    {
        static VrmlNodeType *st = 0;

        if (!t)
        {
            if (st)
                return st; // Only define the type once.
            t = st = new VrmlNodeType("ToolMachine", creator);
        }

        VrmlNodeChild::defineType(t); // Parent class
        for (size_t i = 0; i < 2; i++)
        {
            t->addEventOut(axisNames[i], VrmlField::SFROTATION);
        }
        for (size_t i = 2; i < axisNames.size(); i++)
        {
            t->addEventOut(axisNames[i], VrmlField::SFVEC3F);
        }
         t->addEventOut("aAxisYOffsetPos", VrmlField::SFVEC3F);
         t->addEventOut("aAxisYOffsetNeg", VrmlField::SFVEC3F);
        return t;
    }

    virtual VrmlNodeType *nodeType() const { return defineType(); };

    VrmlNode *cloneMe() const override
    {
        return new MachineNode(*this);
    }

    void move(const osg::Vec3f &position)
    {
        std::cerr << "moving machine" << std::endl;
        auto t = System::the->time();
        eventOut(t, "X", VrmlSFVec3f{-position.x(), 0, 0});
        eventOut(t, "Y", VrmlSFVec3f{0, 0, -position.y()});
        eventOut(t, "Z", VrmlSFVec3f{0, position.z(), 0});
    }

    void move2(int axis, float value)
    {
        auto t = System::the->time();
        if(axis >= 2)
        {
            osg::Vec3f v;
            v[axis -2] = value /1000;
            eventOut(t, axisNames[axis], VrmlSFVec3f{v.x(), v.z(), -v.y()});
        }
        else{
            osg::Vec3f v;
            v[axis] = 1;
            eventOut(t, axisNames[axis], VrmlSFRotation{v.x(), v.z(), v.y(), -value / 180 *(float)osg::PI});
        }
    }
private:
    size_t m_index = 0;
};

VrmlNode *creator(VrmlScene *scene)
{
    return new MachineNode(scene);
}



COVERPLUGIN(ToolMaschinePlugin)

ToolMaschinePlugin::ToolMaschinePlugin()
:coVRPlugin(COVER_PLUGIN_NAME)
, ui::Owner("ToolMachinePlugin", cover->ui)
, m_menu(new ui::Menu("ToolMachine", this))
, m_server(std::make_unique<ui::EditFieldConfigValue>(m_menu, "server", "", *config(), "", config::Flag::PerModel))
{
    
    m_menu->allowRelayout(true);

    for (size_t i = 0; i < 5; i++)
        m_axisNames[i] = std::make_unique<ui::SelectionListConfigValue>(m_menu, axisNames[i] + std::string("_axis"), 0, *config(), "axles", config::Flag::PerModel);
    for (size_t i = 0; i < 5; i++)
        m_offsets[i] = std::make_unique<ui::EditFieldConfigValue>(m_menu, axisNames[i] + std::string("_offset"), "", *config(), "offsets", config::Flag::PerModel);

    opcua::addOnClientConnectedCallback([this]()
    {
        auto availableFields = opcua::getClient()->availableFields();
        for (size_t i = 0; i < 5; i++)
        {
            m_axisNames[i]->ui()->setList(availableFields);
            m_axisNames[i]->ui()->select(m_axisNames[i]->getValue());
        }

    });


    m_server->setUpdater([this]()
    {
        opcua::connect(m_server->getValue());
    });

    opcua::addOnClientDisconnectedCallback([this]()
    {
        for (size_t i = 0; i < 5; i++)
        {
            m_axisNames[i]->ui()->setList(std::vector<std::string>());
        }
    });
    
    
    VrmlNamespace::addBuiltIn(MachineNode::defineType());

    config()->setSaveOnExit(true);



    // m_offsets = new opencover::ui::VectorEditField(menu, "offsetInMM");
    // m_offsets->setValue(osg::Vec3(-406.401596,324.97962,280.54943));

    
}


void ToolMaschinePlugin::key(int type, int keySym, int mod)
{
    if(!type == osgGA::GUIEventAdapter::KEY_Down)
        return;
    std::string key = "unknown";
    if (!(keySym & 0xff00))
    {
        char buf[2] = { static_cast<char>(keySym), '\0' };
        key = buf;
    }
    float speed = 0.5;
    std::cerr << "Key input  " << key << std::endl;
    
    for (size_t i = 0; i < axisNames.size(); i++)
    {
        auto &axis = m_axisPositions[i];
        if(key == axisNamesLower[i])
        {
            if(mod == 3)
            {
                std::cerr << "decreasing " << axisNames[i] << std::endl;
                speed *= -1;
            } else{
                std::cerr << "increasing " << axisNames[i] << std::endl;
            }
            axis += speed;
            for(auto &m : machineNodes)
            {
                // m->move(v);
                m->move2(i, axis);
            }
        }
    }
}

bool ToolMaschinePlugin::update()
{
    auto client = opcua::getClient();
    if(!client || !client->isConnected())
        return true;
    std::array<double, 5> axisValues;
    // for (size_t i = 0; i < 5; i++)
    // {
    //     axisValues[i] = client->readValue("ENC2_POS|" + std::string(axisNames[i]));
    // }
    
    // for(const auto &m : machineNodes)
    // {
    //     m->move2(0, axisValues[0]);
    //     m->move2(1, axisValues[1]);
    //     m->move2(2, axisValues[2] + m_offsets->value().x());
    //     m->move2(3, axisValues[3] + m_offsets->value().y());
    //     m->move2(4, axisValues[4] + m_offsets->value().z());
    // }
    for (size_t i = 0; i < 5; i++)
        axisValues[i] = client->readValue(m_axisNames[i]->ui()->selectedItem()) + m_offsets[i]->ui()->number();
    for(const auto &m : machineNodes)
    {
        for (size_t i = 0; i < 5; i++)
            m->move2(i, axisValues[i]);
        
    }

    return true;
}
