#include "Bone.h"
#include <osgAnimation/UpdateBone>
#include <osgAnimation/Bone>
#include <osgAnimation/StackedRotateAxisElement>
#include <array>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
BoneParser::BoneParser() 
: osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
{
}


constexpr std::array<Constraint, numBones> jointConstraints{ //order see body_number.jpg
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 1  Hips
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 2  LeftShoulder
Constraint{0.0, 1.4, 0.0, 0.0, 0.7, Constraint::Type::Hinge},     // 3  LeftArm
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 4  LeftForeArm
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Effector},  // 5  LeftHand
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 6  RightShoulder
Constraint{0.0, 1.4, 0.0, 0.0, 0.7, Constraint::Type::Hinge},     // 7  RightArm
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 8  RightForeArm
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Effector},  // 9  RightHand
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 10 LeftUpLeg
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 11 RightUpLeg
Constraint{0.0, 0.0, 0.0, 1.4, 0.7, Constraint::Type::Hinge},     // 12 LeftLeg
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 13 LeftFoot
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Effector},  // 14 LeftToeBase
Constraint{0.0, 0.0, 0.0, 1.4, 0.7, Constraint::Type::Hinge},     // 15 RightLeg
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 16 RightFoot
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Effector},  // 17 RightToeBase
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Ball},      // 18 Neck
Constraint{1.4, 1.4, 1.4, 1.4, 0.7, Constraint::Type::Effector},  // 19 Head
};

osg::MatrixTransform* createSphere(const osg::Vec3& pos, float radius)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), radius)));
    osg::ref_ptr<osg::Material> material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(1, 0, 0, 1));
    geode->getOrCreateStateSet()->setAttribute(material);
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrix::translate(pos));
    mt->addChild(geode);
    static size_t count = 0;
    mt->setName("sphere" + std::to_string(count++));
    return mt;
}

void BoneParser::apply(osg::Node& node) {

    if(auto bone = dynamic_cast<osgAnimation::Bone*>(&node))
    {
        
        std::cerr << "bone: " << node.getName() << std::endl;
        auto parent = bone;
        if(!parent->getBoneParent())
        {
            m_rootBone = bone;
        }

        // osg::Vec3 pos;
        // std::cerr << "parents: ";
        // while(parent = parent->getBoneParent())
        // {
        //     std::cerr <<  parent->getName() << ", ";
        //     auto &stacked = dynamic_cast<osgAnimation::UpdateBone*>(parent->getUpdateCallback())->getStackedTransforms();
        //     for (const auto &i : stacked)
        //     {
        //         if(auto translate = dynamic_cast<osgAnimation::StackedTranslateElement*>(i.get()))
        //         {
        //             pos += translate->getTranslate();
        //         } 
        //     }
        // }
        std::cerr << std::endl;
        for (size_t i = 0; i < numBones; i++)
        {
            if(node.getName() == boneNames[i])
            {
                auto pos = bone->getMatrixInSkeletonSpace().getTrans() - m_rootBone->getMatrixInSkeletonSpace().getTrans();
                pos = {pos.x(), pos.z(), pos.y()}; // swap y and z
                m_skeleton[i].initialPosition = pos;
                m_skeleton[i].position = pos;
                m_skeleton[i].constraint = jointConstraints[i];
                m_bones[i] = bone;
                auto &stacked = dynamic_cast<osgAnimation::UpdateBone*>(node.getUpdateCallback())->getStackedTransforms();
                stacked.getMatrix();
                // stacked.erase(std::remove_if(stacked.begin(), stacked.end(), [](const osgAnimation::StackedTransform::value_type& t){
                //             return dynamic_cast<osgAnimation::StackedRotateAxisElement*>(t.get());
                //         }), stacked.end());
                m_joints[i] = new osgAnimation::StackedQuaternionElement;
                stacked.push_back(m_joints[i]);
                break;
            }
        }
    }    
    traverse(node);
}

void BoneParser::update()
{
    for (size_t i = 0; i < numBones; i++)
    {
        auto bone = m_bones[i];
        auto joint = m_skeleton[i];
        auto posGlobal = joint.position;
        posGlobal = {posGlobal.x(), posGlobal.z(), posGlobal.y()}; // swap y and z
        if(!m_boneDummies[i])
        {
            m_boneDummies[i] = createSphere(posGlobal, 2);
            m_boneDummies[i]->setName(std::string("dummy_") + boneNames[i]);
            m_rootBone->addChild(m_boneDummies[i]);
        }
        else
        {
            m_boneDummies[i]->setMatrix(osg::Matrix::translate(posGlobal));
        }
        if(!bone->getBoneParent())
        {
            auto &stacked = dynamic_cast<osgAnimation::UpdateBone*>(bone->getUpdateCallback())->getStackedTransforms();
            
            for (const auto &i : stacked)
            {
                if(auto translate = dynamic_cast<osgAnimation::StackedTranslateElement*>(i.get()))
                {
                    translate->setTranslate(joint.position);
                } 
            }
        }

    }
}