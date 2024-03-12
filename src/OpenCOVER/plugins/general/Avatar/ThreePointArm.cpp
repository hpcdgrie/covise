#include "ThreePointArm.h"
#include <iostream>
#include <osgAnimation/UpdateBone>
#include <osgAnimation/StackedRotateAxisElement>
#include <cover/coVRPluginSupport.h>
#include <cassert>
#include <OpenVRui/osg/mathUtils.h>
using namespace osg;
using namespace osgAnimation;
using namespace opencover;
struct NodeFinder : public NodeVisitor {
  NodeFinder(const std::string &name)
      : NodeVisitor(NodeVisitor::TRAVERSE_ALL_CHILDREN), name(name) {}
  void apply(Node &node) override {
    if (node.getName() == name) {
      if (auto bone = dynamic_cast<Bone *>(&node)) {
        joint.name = name;
        auto &stacked = dynamic_cast<UpdateBone *>(node.getUpdateCallback())
                ->getStackedTransforms();
        
        stacked.erase(std::remove_if(stacked.begin(), stacked.end(), [](const StackedTransform::value_type& t){
            return dynamic_cast<StackedRotateAxisElement*>(t.get());
        }), stacked.end());

        joint.basePos = dynamic_cast<StackedTranslateElement*>(std::find_if(stacked.begin(), stacked.end(), [](const StackedTransform::value_type& t){
            return dynamic_cast<StackedTranslateElement*>(t.get());
        })->get());
        joint.rot = new osgAnimation::StackedQuaternionElement;
        stacked.push_back(joint.rot);
        joint.bone = bone;
      }
      return;
    } else
      traverse(node);
  }
  Joint joint;
  std::string name;
};

Joint findNode(Node &node, const std::string &name) {
  NodeFinder nf(name);
  node.accept(nf);
  if (!nf.joint.bone)
    std::cerr << "Node " << name << " not found" << std::endl;
  return std::move(nf.joint);
}

ThreePointArm::ThreePointArm(Node &model, const std::string &base,
                             const std::string &mid, const std::string &end)
: m_base(findNode(model, base))
, m_mid(findNode(model, mid))
, m_end(findNode(model, end))
, m_armLength(m_mid.basePos->getTranslate().length() + m_end.basePos->getTranslate().length())
{
    m_randomRot /= m_randomRot.length();
    std::cerr << "Arm length: " << m_armLength << std::endl;
    // auto f = findNode(model, "mixamorig:LeftToeBase");
    // std::cerr << "LeftToeBase: " << f.basePos->getTranslate().x() << " " << f.basePos->getTranslate().y() << " " << f.basePos->getTranslate().z() << std::endl;
    m_end.rot->setQuaternion(Quat(PI_2, Vec3(1, 0, 0)));
}


Matrix transformCoords(const Matrix &mInFrom, const Node *from, const Node *to)
{
    auto toFrom = to->getWorldMatrices(from)[0];
    auto fromTo = Matrix::inverse(toFrom);
    return  mInFrom * fromTo;
}

float getAngle(const Quat &quat, Vec3 axis)
{

    // Normalize the axis
    axis.normalize();

    // Get the axis and angle of the quaternion
    Vec3 quatAxis(quat.x(), quat.y(), quat.z());
    double quatAngle;
    quat.getRotate(quatAngle, quatAxis);

    // Project the quaternion's axis onto the given axis
    Vec3 projectedAxis = axis * (quatAxis * axis);

    // The magnitude of the projected axis gives you the component of the quaternion's rotation that is in the direction of the given axis
    double angleAroundAxis = projectedAxis.length();

    return angleAroundAxis;
}

float getAngle2(const Quat &quat, Vec3 axis)
{
double angle;
Vec3d quatAxis;
quat.getRotate(angle, quatAxis);
quatAxis.normalize();
axis.normalize();
auto x1 = quatAxis.x() * axis.x();
auto y1 = quatAxis.y() * axis.y();
auto z1 = quatAxis.z() * axis.z();
auto s = x1 + y1 + z1;
return quatAxis * axis;
}

//  has singularity in case of swing_rotation close to 180 degrees rotation.
//  if the input quaternion is of non-unit length, the outputs are non-unit as well
//  otherwise, outputs are both unit

inline void swing_twist_decomposition(const Quat& rotation,
                                       const Vec3& direction,
                                       Quat& swing,
                                       Quat& twist)
{
    auto d = direction;
    d.normalize();
    Vec3 ra(rotation.x(), rotation.y(), rotation.z()); // rotation axis
    Vec3 p = d * (ra * d); // projection of ra onto direction (parallel component)
    auto l = sqrt(p.x() * p.x() + p.y() * p.y() + p.z() * p.z() + rotation.w() * rotation.w());
    twist = Quat(p.x(), p.y(), p.z(), rotation.w()) / l;
    swing = rotation * twist.conj();
}

inline void swing_twist_decomposition2(const Quat& q,
                                       const Vec3& twistAxis,
                                       Quat& swing,
                                       Quat& twist)
{
  Vec3 r(q.x(), q.y(), q.z());
 
  // singularity: rotation by 180 degree
  auto epsilon = 1e-7;
  if (r.length2() < epsilon)
  {
    Vec3 rotatedTwistAxis = q * twistAxis;
    Vec3 swingAxis = twistAxis ^ rotatedTwistAxis;
 
    if (swingAxis.length2() > epsilon)
    {
      swing.makeRotate(twistAxis, rotatedTwistAxis);
    }
    else
    {
      // more singularity: 
      // rotation axis parallel to twist axis
      swing = Quat(); // no swing
    }
 
    // always twist 180 degree on singularity
    twist.makeRotate(180.0f, twistAxis);
    return;
  }
 
  // meat of swing-twist decomposition
  Vec3 p = twistAxis * (twistAxis * r);
  twist = Quat(p.x(), p.y(), p.z(), q.w());
  twist /= twist.length();
  swing = q * twist.conj();
}

void ThreePointArm::update(const Matrix &target, float angle)
{
    //das muss in base->parent + base pos transformiert werden
    // auto targetBase1 = transformCoords(target, cover->getObjectsRoot(), m_base.bone->getBoneParent());
    // targetBase1.setTrans(targetBase1.getTrans() + m_base.basePos->getTranslate());
    auto targetBase = transformCoords(target, cover->getObjectsRoot(), m_base.bone);
    targetBase.postMult(Matrix::rotate(m_base.rot->getQuaternion()));
    Quat r;
    Vec3 tp = targetBase.getTrans();
    tp.normalize();
    auto bp = m_mid.basePos->getTranslate();
    bp.normalize();
    
    auto randotTp = m_randomRot * tp;
    r.makeRotate(bp, randotTp);
    r = r * m_randomRot.conj();
    Quat baseRot, midRot;
    baseRot = r  * Quat(DegreesToRadians(angle), tp);
    Quat twist, swing;
    swing_twist_decomposition2(baseRot, tp, swing, twist);
    auto test = swing * twist;
    if(targetBase.getTrans().length() < m_armLength){
        //calculate angles in a triangle given by targetBase lengh, m_mid.basePos->getTranslate().length() and m_end.basePos->getTranslate().length()
        auto a = m_mid.basePos->getTranslate().length();
        auto b = m_end.basePos->getTranslate().length();
        auto c = targetBase.getTrans().length();
        auto cosA = (b*b + c*c - a*a) / (2*b*c);
        auto cosB = (a*a + c*c - b*b) / (2*a*c);
        auto cosC = (a*a + b*b - c*c) / (2*a*b);
        auto angleA = acos(cosA);
        auto angleB = acos(cosB);
        auto angleC = acos(cosC);

        auto n  = targetBase.getTrans() ^ m_controlPoint;
        n.normalize();
        midRot.makeRotate(PI - angleC, n);
        
        Quat baseRotOffset(angleB, n);
        baseRot = baseRotOffset * baseRot;

    }
    m_base.rot->setQuaternion(baseRot);
    m_mid.rot->setQuaternion(midRot);


}