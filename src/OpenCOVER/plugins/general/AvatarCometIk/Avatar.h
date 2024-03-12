/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _Animated_Avatar_Plugin
#define _Animated_Avatar_Plugin

#include "Bone.h"

#include <cover/coVRPluginSupport.h>
#include <cover/ui/CovconfigLink.h>
#include <cover/ui/FileBrowser.h>
#include <cover/ui/Owner.h>
#include <cover/ui/VectorEditField.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>
#include <cover/ui/SelectionList.h>
#include <ik/ik.h>

#include <PluginUtil/coVR3DTransRotInteractor.h>
#include <map>

#include <osgAnimation/StackedTranslateElement>
#include <osgAnimation/BasicAnimationManager>

using namespace covise;
using namespace opencover;
using namespace ui;

namespace opencover{
class VRAvatar;
}
struct BoneParser;
struct AnimationManagerFinder : public osg::NodeVisitor
{
    osg::ref_ptr<osgAnimation::BasicAnimationManager> m_am;
    AnimationManagerFinder();
    void apply(osg::Node& node) override;
};
class LoadedAvatar{
public:
    bool loadAvatar(const std::string &filename, VRAvatar *partnerAvatar, ui::Menu *menu);
    void update(const osg::Vec3 &target);
    void update();

private:
    void positionAndScaleModel(osg::Node* model, osg::MatrixTransform *pos);
    void createAnimationSliders(ui::Menu *menu);
    void buidCompleteIkModel();
    osgAnimation::AnimationList m_animations;
    AnimationManagerFinder m_animationFinder;
    bool m_rotateButtonPresed = false;
    osg::ref_ptr<osg::Node> model;
    osg::ref_ptr<osg::MatrixTransform> modelTrans, modelRot, modelScale, dummySphere2;
    opencover::ui::Button *lockA, *lockTheta, *lockB, *flipDeltaW, *flipTheta, *flipA, *flipB;
    opencover::ui::EditField *AOffset, *ThethaOffset, *BOffset;
    osg::ref_ptr<osg::MatrixTransform> targetDummy, ellbowDummy;
    ik_solver_t* ikSolver;
    ik_node_t* ikShoulder, *ikEllbow, *ikWrist;
    ik_effector_t* ikTarget;
    osg::MatrixTransform *m_targetTransform, *m_headTransform;

    ui::Button *m_useInteractor = nullptr; 
    ui::SelectionList* m_effectorSelector = nullptr; 
    ui::Slider* m_modelHeight = nullptr; 
    ui::Slider* m_leftLegDistance = nullptr; 
    ui::Slider* m_rightLegDistance = nullptr; 
    ui::VectorEditField *m_controllPoint = nullptr;
    std::array<ui::Slider*, 3> m_angles;
    std::unique_ptr<opencover::coVR3DTransRotInteractor> m_interactor, m_interactorFloor; 
    BoneParser *skeleton;
    float m_modelHeightValue = 0.0f;


};







#endif
