#include "ToolChanger.h"

#include <osgDB/ReadFile>
#include <osgAnimation/Skeleton>
#include <osgAnimation/RigGeometry>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/BasicAnimationManager>
#include <osgAnimation/RigTransformHardware>
#include <osgAnimation/Animation>
#include <osgAnimation/Skeleton>
#include <osgGA/GUIEventHandler>
#include <osgAnimation/StackedRotateAxisElement>
#include <osg/Material>

#include <cover/coVRPluginSupport.h>
#include <cover/ui/Slider.h>

#include <cassert>

#include <pluginUtil/coColorMap.h>

using namespace opencover;

struct AnimationManagerFinder : public osg::NodeVisitor
{
    osg::ref_ptr<osgAnimation::BasicAnimationManager> m_am;
    AnimationManagerFinder(): osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN) {}
    void apply(osg::Node& node) override{

    if (m_am.valid())
        return;

    if (node.getUpdateCallback()) {       
        m_am = dynamic_cast<osgAnimation::BasicAnimationManager*>(node.getUpdateCallback());
        return;
    }
    
    traverse(node);
}
};

constexpr int numArms = 60;

const float pathSideLength = osg::PI * 1000;
constexpr float pathRadius = 1000;
const float acceletation = 2; //relative to max speed -> number of seconds until max speed
const float pathLength = 2 * pathSideLength + 2 * pathRadius * osg::PI;
const float pathDistance = pathLength / numArms;
const float relativeStreigthLength = 2 * pathSideLength / pathLength;

// const std::array<float, 4> relativeParts = {pathSideLength/ pathLength, pathRadius * osg::PI / pathLength, pathSideLength/ pathLength, pathRadius * osg::PI / pathLength};
const std::array<float, 4> parts = {pathSideLength, pathRadius * osg::PI, pathSideLength, pathRadius * osg::PI};

osg::Matrix calculatePath(int id, float offset)
{

    auto abs = (id + offset) * pathDistance;

    const std::array<std::function<osg::Vec3(float)>, 4> segments = {
        [](float rel) { return osg::Vec3(rel * pathSideLength, 0, 0); },
        [](float rel) { return osg::Vec3(pathRadius * sin(rel * osg::PI), 0, pathRadius - pathRadius * cos(rel * osg::PI)); },
        [](float rel) { return osg::Vec3(-rel * pathSideLength, 0, 0); },
        [](float rel) { return osg::Vec3(-pathRadius * sin(rel * osg::PI), 0, -pathRadius + pathRadius * cos(rel * osg::PI)); }
    };
    const std::array<std::function<osg::Vec3(float)>, 4> segmentsNormals = {
        [](float rel) { return osg::Vec3(0, 0, 1); },
        [](float rel) { return osg::Vec3(-sin(rel * osg::PI), 0, cos(rel * osg::PI)); },
        [](float rel) { return osg::Vec3(0, 0, -1); },
        [](float rel) { return osg::Vec3(-sin(rel * osg::PI), 0, -cos(rel * osg::PI)); }
    };
    std::array<float, 4> distanceInSection = {0, 0, 0, 0};
    float pos = 0;
    for (size_t i = 0; i < 4; i++)
    {
        pos += parts[i];
        if(abs <= pos)
        {
            distanceInSection[i] = abs - (pos - parts[i]); 
            break;
        }else
        {
            distanceInSection[i] = parts[i];
        } 
    }
    osg::Vec3 posVec;
    osg::Vec3 ortho = osg::Vec3(0, -1, 0) ^ segments[0](1);
    osg::Vec3 lastAddition{0, 0, -1};
    size_t secNum = 0;
    for (size_t i = 0; i < 4; i++)
    {
        if(distanceInSection[i] == 0)
        break;

        lastAddition = segments[i](distanceInSection[i] / parts[i]);
        ortho = segmentsNormals[i](distanceInSection[i] / parts[i]);
        posVec += lastAddition;
        secNum = i;
    }
    ortho.normalize();
    osg::Matrix posMat;
    
    posMat.makeTranslate(posVec);
    osg::Quat q;
    auto angle = acos(ortho * osg::Vec3(0, 0, 1));
    if(secNum==1)
    angle *= -1;
    q.makeRotate(angle, osg::Vec3{0, 1, 0});
    posMat.setRotate(q);
    return posMat;
}

class Arm{
public:
    Arm(osg::ref_ptr<osg::Node> model, int id, const osg::Vec4& color)
    : m_model(static_cast<osg::Node*>(model->clone(osg::CopyOp::DEEP_COPY_ALL)))
    , m_transform (new osg::MatrixTransform)
    , m_id(id)
    {
        m_transform->addChild(m_model);
        cover->getObjectsRoot()->addChild(m_transform);
        position(0);

        m_model->accept(m_animation);
        auto &animations = m_animation.m_am->getAnimationList();
        // for(auto &a : m_animation.m_am->getAnimationList())
        // {
        //     std::cerr << "arm " << id << " has animation " << a->getName() << std::endl;
        // }
        applyColor(color);
    }

    void position(float offset)
    {
        m_transform->setMatrix(calculatePath(m_id, offset));
        m_distance = m_id + offset;
    }

    void play()
    {
        auto animation = m_animation.m_am->getAnimationList().front();
        animation->setPlayMode(osgAnimation::Animation::ONCE);
        m_animation.m_am->playAnimation(animation, 1, 1); // Play once
    }

    float getDistance() const
    {
        return m_distance;
    }

private:
    osg::ref_ptr<osg::Node> m_model;
    osg::ref_ptr<osg::MatrixTransform> m_transform;
    int m_id = -1;
    AnimationManagerFinder m_animation;
    float m_distance = 0;

    void applyColor(const osg::Vec4& color)
    {
        osg::ref_ptr<osg::Material> material = new osg::Material;
        material->setDiffuse(osg::Material::FRONT_AND_BACK, color);

        osg::StateSet* stateSet = m_model->getOrCreateStateSet();
        stateSet->setAttributeAndModes(material, osg::StateAttribute::ON);
        stateSet->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    }
};

ToolChanger::ToolChanger(ui::Menu* menu)
{
    covise::ColorMapSelector *colorMapSelector = new covise::ColorMapSelector(*menu);
    osg::ref_ptr model = osgDB::readNodeFile("C:/Users/Dennis/Data/IfW/Werkzeugwechsler/WerkzeugArmAnimation.fbx"); 
    for (size_t i = 0; i < numArms; i++)
    {
        auto c = colorMapSelector->getColor(i, 0, numArms);
        m_arms.push_back(std::make_unique<Arm>(model, i, c));
    }
    // osg::MatrixTransform *mt = new osg::MatrixTransform;
    // osg::Matrix m;
    // m.makeScale(2,2,2);
    // m.setTrans(0,-1000,0);
    // mt->setMatrix(m);
    // mt->addChild(model);
    // cover->getObjectsRoot()->addChild(mt);

    m_anim = new ui::EditField(menu, "selectTool");
    m_anim->setValue("3");
    // m_anim->setCallback([this](const std::string &value){
    //     m_arms[m_anim->number()]->play();
    // });
    m_maxSpeed = new ui::Slider(menu, "speed");
    m_maxSpeed->setBounds(-1, 1);
    m_maxSpeed->setValue(0.5);

    m_action = new ui::Action(menu, "changeTool");
    m_action->setCallback([this](){
        m_changeTool = true;
        m_selectedArm = m_arms[m_anim->number()].get();
        m_distanceToSeletedArm = m_selectedArm->getDistance();
    });

}

ToolChanger::~ToolChanger() = default;

// float numFramesUntilStop(float speed, float distance)
// {
//     int i = 0;
//     auto d = distance;
//     while (d > 0)
//     {
//         d -= (speed + speed / acceletation * i);
//         ++i;
//     }
//     i;
    
//     auto part = sqrt(pow(speed, 2) + 4 * speed/ acceletation *distance);
//     auto retval = (-speed + part) / (2 * speed / acceletation);
//     assert(retval > 0);

//     std::cerr << "frames until stop: " << retval << " numerisch: " << i << std::endl;
//     return i;
// }

void ToolChanger::update()
{
    if(!m_selectedArm)
        return;
    
    if(m_selectedArm->getDistance() <= 1)
    {
        positionArms(-m_selectedArm->getDistance());
        m_selectedArm->play();
        m_selectedArm = nullptr;
        m_speed = 0;
        decellerate = false;
        std::cerr << "position reached " << std::endl;
        return;
    }
    auto maxSpeed = m_maxSpeed->value() * cover->frameDuration();
    //decelerate
    auto a = acceletation *  cover->frameDuration();
    if(m_speed * a / 2 > numArms - m_selectedArm->getDistance()) //fix distance juming when becoming > numArms
    {
        m_speed -= maxSpeed / a;
        std::cerr << "decellerating " << m_speed << std::endl;
    }
    else if(m_speed < maxSpeed)
        m_speed += maxSpeed  / a;
    // if(!decellerate && m_speed > 0 && numFramesUntilStop(m_speed, m_selectedArm->getDistance()) <= acceletation)
    // {
    //     decellerate = true;
    //     std::cerr << "start decellerating" << std::endl;
    // }
    // if(decellerate)
    // {
    //     m_speed -= m_maxSpeed->value() / acceletation;
    //     std::cerr << "decellerating " << m_speed << std::endl;
    // }
    // else if(m_speed < m_maxSpeed->value())
    //     m_speed += m_maxSpeed->value() / acceletation;

    m_offset += m_speed ;
    if(m_maxSpeed->value() == 0)
        return;
    if(m_offset > numArms)
        m_offset = 0;
    if(m_offset < 0)
        m_offset = numArms;
    positionArms(m_offset);
    

}

void ToolChanger::positionArms(float offset)
{
    for (size_t i = 0; i < numArms; i++)
    {
        auto offset = m_offset;
        if(offset + i> numArms)
            offset -= numArms;
        m_arms[i]->position(offset);
    }
}


