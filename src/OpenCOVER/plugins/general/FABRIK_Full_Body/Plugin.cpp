
#include "Plugin.h"
#include <iostream>
#include <fstream>
#include <iomanip>

#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/MatrixTransform>

using namespace opencover;

COVERPLUGIN(FabrikPlugin);

std::vector<osg::Vec3> getPositions(const std::string &filename)
{
    std::vector<osg::Vec3> positions;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return positions;
    }

    float x, y, z;
    while (file >> x >> y >> z)
    {
        positions.emplace_back(x, y, z);
    }
    return positions;
}

std::vector<osg::Quat> getOrientations(const std::string &filename)
{
    std::vector<osg::Quat> orientations;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return orientations;
    }

    float x, y, z, w;
    while (file >> x >> y >> z >> w)
    {
        orientations.emplace_back(x, y, z, w);
    }
    return orientations;
}

std::vector<Constraint::Type> getConstraintTypes(const std::string &filename)
{
    std::vector<Constraint::Type> constraints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return constraints;
    }
    std::array<const char *, 3> constraint_types = {"BALL", "hinge", "endEffector"};

    std::string type;
    while (file >> type)
    {
        bool found = false;
        for (size_t i = 0; i < constraint_types.size(); ++i)
        {
            if (type == constraint_types[i])
            {
                constraints.push_back(static_cast<Constraint::Type>(i));
                found = true;
                break;
            }
        }
        if (!found)
            std::cerr << "Unknown constraint type: " << type << std::endl;
    }
    return constraints;
}

std::vector<float> getBoneTwistConstraints(const std::string &filename)
{
    std::vector<float> constraints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return constraints;
    }

    float constraint;
    while (file >> constraint)
    {
        constraints.push_back(constraint);
    }
    return constraints;
}

osg::Vec4 getColorFromCount(size_t count) {
    
    if(count > 1 && count <= 4) //right arm
    {
        return osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f); // Green
    }
    else if(count > 4 && count <= 8) //left arm
    {
        return osg::Vec4(1.0f, 1.0f, 0.0f, 1.0f); // Yellow
    }
    else if(count == 9 ||count == 11 ||count == 12 ||count == 13) // right leg
    {
        return osg::Vec4(1.0f, 0.5f, 0.0f, 1.0f); // Orange
    }
    else if(count == 10 ||count == 14 ||count == 15 ||count == 16)
    {
        return osg::Vec4(1.0f, 0.0f, 1.0f, 1.0f); // Magenta
    }
    return osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f); // white

}

osg::MatrixTransform* createSphere(const osg::Vec3& pos, float radius, const std::string & name)
{
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), radius)));
    static size_t count = 0;

    osg::ref_ptr<osg::Material> material = new osg::Material;
    material->setDiffuse(osg::Material::FRONT_AND_BACK, getColorFromCount(count++));
    geode->getOrCreateStateSet()->setAttribute(material);
    osg::MatrixTransform* mt = new osg::MatrixTransform;
    mt->setMatrix(osg::Matrix::translate(pos));
    mt->addChild(geode);
    mt->setName(name);
    return mt;
}

FabrikPlugin::FabrikPlugin()
    : coVRPlugin(COVER_PLUGIN_NAME)
{



    const std::string dir = "C:/Users/Dennis/Apps/FBRIK_Full_Body/FABRIK_Full_Body_C++/test/inputs/";

    auto joints_position = getPositions(dir + "joints_position.txt");
    auto joints_position_fixed = getPositions(dir + "joints_position_fixed.txt");
    auto orientation = getOrientations(dir + "orientation.txt");
    auto joints_constraint = getOrientations(dir + "joints_constraint.txt");
    auto constraintTypes = getConstraintTypes(dir + "constraint_type.txt");
    auto bone_twist_constraints = getBoneTwistConstraints(dir + "bone_twist_constraints.txt");

    for (size_t i = 0; i < joints_position.size(); i++)
    {
        Joint joint;
        joint.initialPosition = joints_position_fixed[i];
        joint.position = joints_position[i];
        joint.rotation = orientation[i];
        joint.constraint.type = constraintTypes[i];
        joint.constraint.adduction = joints_constraint[i][0];
        joint.constraint.abduction = joints_constraint[i][1];
        joint.constraint.flexion = joints_constraint[i][2];
        joint.constraint.extension = joints_constraint[i][3];
        joint.constraint.twist = bone_twist_constraints[i];
        m_skeleton[i] = joint;
        m_boneDummies.push_back(createSphere(joint.position, 0.05, JointNames[i]));
        cover->getObjectsRoot()->addChild(m_boneDummies.back());
        m_boneDummies.back()->setName("boneDummy" + std::to_string(i));
    }

    osg::Matrix m;
    auto interSize = 0.2;
    m.setTrans(m_skeleton[9].position);
    m_interactor.reset(new coVR3DTransRotInteractor(m, interSize, vrui::coInteraction::InteractionType::ButtonA, "hand", "targetInteractor", vrui::coInteraction::InteractionPriority::Medium));
    m_interactor->enableIntersection();
    m_interactor->show();

}

bool FabrikPlugin::update()
{
    auto target = m_interactor->getMatrix();
    Fabrik fabrik(m_skeleton, target.getTrans(), target.getRotate());
    fabrik.solve();
    for (size_t i = 0; i < m_skeleton.size(); i++)
    {
        auto m = m_boneDummies[i]->getMatrix();
        m.setTrans(m_skeleton[i].position);
        m_boneDummies[i]->setMatrix(m);
    }
    return true;
}
