#include "Bone.h"
#include <osgAnimation/UpdateBone>
#include <osgAnimation/Bone>
#include <osgAnimation/StackedRotateAxisElement>
#include <osgAnimation/StackedMatrixElement>
#include <osgAnimation/StackedScaleElement>
#include <array>
#include <osg/Geode>
#include <osg/ShapeDrawable>
BoneParser::BoneParser()
    : osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
}
osg::MatrixTransform *createSphere(int id)
{
    auto m_handSphere = new osg::MatrixTransform();
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->setName("HandSphereGeode");
    osg::ref_ptr<osg::ShapeDrawable> sphere = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0, 0, 0), 0.2));
    // Set color based on id
    osg::Vec4 color(1, 1, 5, 1); // default white
    switch (id % 6)
    {
    case 0:
        color.set(1, 0, 0, 1);
        break; // red
    case 1:
        color.set(0, 1, 0, 1);
        break; // green
    case 2:
        color.set(0, 0, 1, 1);
        break; // blue
    case 3:
        color.set(1, 1, 0, 1);
        break; // yellow
    case 4:
        color.set(1, 0, 1, 1);
        break; // magenta
    case 5:
        color.set(0, 1, 1, 1);
        break; // cyan
    }
    sphere->setColor(color);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    geode->addDrawable(sphere);
    m_handSphere->addChild(geode);
    return m_handSphere;
}

void BoneParser::apply(osg::Node &node)
{

    if (auto bone = dynamic_cast<osgAnimation::Bone *>(&node))
    {
        std::cerr << "bone: " << ikId << " " << bone->getName() << std::endl;
        Bone *parent = nullptr;
        if (!bone->getBoneParent())
        {
            std::cerr << "root bone: " << bone->getName() << std::endl;
            root = bone;
        }
        else
        {
            parent = &nodeToIk[bone->getBoneParent()];
        }

        auto &stacked = dynamic_cast<osgAnimation::UpdateBone *>(node.getUpdateCallback())->getStackedTransforms();
        osgAnimation::StackedTranslateElement *ste = nullptr;
        osg::Vec3 basePos(0, 0, 0);
        for (const auto &i : stacked)
        {
            std::cerr << "stacked transform: " << i->className() << std::endl;
            if (auto translate = dynamic_cast<osgAnimation::StackedTranslateElement *>(i.get()))
            {
                ste = translate;
                basePos = translate->getTranslate();
            } else if(auto translate = dynamic_cast<osgAnimation::StackedMatrixElement *>(i.get()))
            {
                basePos = translate->getMatrix().getTrans();
            }
            else if(auto translate = dynamic_cast<osgAnimation::StackedScaleElement *>(i.get()))
            {
                std::cerr << "stacked scale: " << translate->getScale().x() << " " << translate->getScale().y() << " " << translate->getScale().z() << std::endl;
            }
            else if(auto translate = dynamic_cast<osgAnimation::StackedRotateAxisElement *>(i.get()))
            {
                std::cerr << "stacked scale: " << translate->getAxis().x() << " " << translate->getAxis().y() << " " << translate->getAxis().z() << std::endl;
                std::cerr << "angle: " << translate->getAngle() << std::endl;
            }
        }
        // auto sphere = createSphere(ikId++);
        // bone->addChild(sphere);
        auto sqe = new osgAnimation::StackedQuaternionElement;
        Bone &ikBone = nodeToIk.emplace(std::make_pair(&node, Bone{sqe, basePos, parent, &node})).first->second;
        stacked.push_back(sqe);
        for (size_t i = 0; i < effectorName.size(); i++)
        {
            if (node.getName() == effectorName[i])
            {
                size_t chainLength = effectorChainLenghts[i];
                auto origin = bone;
                for (size_t i = 0; i < chainLength + 1; i++)
                {
                    auto &stacked = dynamic_cast<osgAnimation::UpdateBone *>(origin->getUpdateCallback())->getStackedTransforms();
                    stacked.erase(std::remove_if(stacked.begin(), stacked.end(), [](const osgAnimation::StackedTransform::value_type &t)
                                                 { return dynamic_cast<osgAnimation::StackedRotateAxisElement *>(t.get()); }),
                                  stacked.end());
                    origin = origin->getBoneParent();
                }
            }
        }
    }
    traverse(node);
}

BoneParser::NodeMap::iterator BoneParser::findNode(const std::string &name)
{
    return std::find_if(nodeToIk.begin(), nodeToIk.end(), [&name](const NodeMap::value_type &p)
                        { return p.first->getName() == name; });
}

osg::Vec3 BoneParser::claculateBoneDistance(const std::string &boneName1, const std::string &boneName2)
{
    auto bone1it = findNode(boneName1);
    auto bone2it = findNode(boneName2);
    auto rootIt = findNode("mixamorig:Hips");
    if (bone1it == nodeToIk.end() || bone2it == nodeToIk.end() || rootIt == nodeToIk.end())
        return osg::Vec3(0, 0, 0);

    const osg::Node *bone1 = bone1it->first;
    const osg::Node *bone2 = bone2it->first;
    const osg::Node *root = rootIt->first;
    auto bone1Pos = bone1->getWorldMatrices(root)[0].getTrans();
    auto bone2Pos = bone2->getWorldMatrices(root)[0].getTrans();
    return bone2Pos - bone1Pos;
}