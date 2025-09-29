#ifndef COVER_PLUGIN_GHOSTAVATAR_GhostAvatar_H
#define COVER_PLUGIN_GHOSTAVATAR_GhostAvatar_H

#include <array>
#include <memory>

#include <osg/Matrix>
#include <osg/MatrixTransform>
#include <osg/Vec3>
#include <osg/ref_ptr>

#include <cover/coVRPluginSupport.h>
#include <cover/ui/Action.h>
#include <cover/ui/Button.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Owner.h>
#include <cover/ui/VectorEditField.h>
#include <PluginUtil/coVR3DTransformInteractor.h>
#include <osgAnimation/Skeleton>


#include "Bone.h"
const std::string ARM_NODE_NAME = "RightArm"; // "LeftArm"
const std::string HEAD_NODE_NAME = "Bone.002";
class GhostAvatar
{
public:
    GhostAvatar(int id, osg::Matrix adjustMatrix, osg::Matrix permFix = osg::Matrix::identity());
    ~GhostAvatar();

    void loadAvatar();

    // positions avatar at m_interactorFloor and makes the avatar's arm follow m_interactorHand
    bool update();

    // debugging
    void drawLine(const osg::Vec3 &armBase, const osg::Vec3 &targetPos);
    void cleanUpDebugLines();
    int ID() const { return m_id; }

    void showFrames(bool show) { m_showFrames = show; }
    void showTargetLine(bool show) { m_showTargetLine = show; }
    void setArmBaseVector(const osg::Vec3 &vec) { m_armBaseVec = vec; }
    void setAdjustMatrix(const osg::Matrix &mat) { m_adjustMatrix = mat; }
    void setPermFix(const osg::Matrix &mat) { m_permFix = mat; m_axisFix->setMatrix(m_permFix); }
    
private:
    osg::ref_ptr<osg::MatrixTransform> m_avatarTrans;
    osg::ref_ptr<osg::MatrixTransform> m_axisFix;
    BoneParser m_parser;
    std::unique_ptr<opencover::coVR3DTransformInteractor> m_interactorHead, m_interactorFloor, m_interactorHand;
    osg::Vec3 m_armBaseVec = {0, 1, 0};
    osg::Matrix m_adjustMatrix = osg::Matrix::identity();
    osg::Matrix m_permFix = osg::Matrix::identity();

    void createInteractors();

    // settings
    // std::string m_pathToFbx = "C:\\Users\\Dennis\\Data\\Starts\\ghost_noCloth.fbx";
    std::string m_pathToFbx = "C:\\Users\\Dennis\\Data\\Starts\\PLANEE5.fbx";

    // debugging
    osg::ref_ptr<osg::MatrixTransform> m_targetLine;
    osg::ref_ptr<osg::MatrixTransform> m_globalFrame;
    osg::ref_ptr<osg::MatrixTransform> m_armLocalFrame;

    osgAnimation::Bone* m_head = nullptr;
    osgAnimation::Bone* m_arm = nullptr;
    osgAnimation::Bone* m_feet = nullptr;
    osgAnimation::Skeleton* m_skeleton = nullptr;

    const int m_id = -1;
    bool m_showFrames = false;
    bool m_showTargetLine = false;
    bool m_firstUpdate = true;


    
};

#endif // COVER_PLUGIN_GHOSTAVATAR_GhostAvatar_H