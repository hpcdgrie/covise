#include <osgAnimation/RigTransformHardware>

#include <osgAnimation/BoneMapVisitor>
#include <osgAnimation/RigGeometry>
//mostly same as default, but:
// - uses coVRShaderList to load the shader
// - uses int instead of uint for nbBonesPerVertex (uint is not supported by OpenGL ES 2.0)
struct coVRRigTransformHardware : public osgAnimation::RigTransformHardware
{
    const int _maxmatrix = 99;
    bool init(osgAnimation::RigGeometry& rig) override;
};