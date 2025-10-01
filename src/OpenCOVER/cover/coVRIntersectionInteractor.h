/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#ifndef _CO_VR_INTERSECTION_INTERACTOR_H
#define _CO_VR_INTERSECTION_INTERACTOR_H

#include "coIntersection.h"
#include <OpenVRUI/coCombinedButtonInteraction.h>
#include <OpenVRUI/sginterface/vruiIntersection.h>
#include <OpenVRUI/osg/OSGVruiNode.h>
#include <osg/MatrixTransform>
#include <memory>
#include <functional>

namespace osg
{
class Geode;
class Node;
}
namespace vrb
{
class SharedStateBase;
template<class T>
class SharedState;
}

namespace opencover
{
class coVRLabel;

// selectable sphere which can be positioned
// the sphere is modeled in the origin
// transDCS contains the matrix in object coordinates
//
//
//                         scene
//                          |
//                      xformDCS
//                          |
//                      scaleDCS
//                       |      |
//       coviseObjectsRoot   moveTransform
//       |                      |
//      dcs                 scaleTransform
//       |                      |
//      TracerSeq           geometryNode
//      |       |
//   TrGeode TrGeode
//
//

class COVEREXPORT coVRIntersectionInteractor : public vrui::coAction, public vrui::coCombinedButtonInteraction
{
private:
    osg::StateSet *loadDefaultGeostate();
    bool constantInteractorSize_ = true;
    float iconSize_;
    bool firstTime = true;
    bool _highliteHitNodeOnly;

    osg::Geode *findGeode(osg::Node *n);

protected:
    osg::ref_ptr<osg::Node> geometryNode; ///< Geometry node
    osg::ref_ptr<osg::MatrixTransform> moveTransform;
    osg::ref_ptr<osg::MatrixTransform> scaleTransform;
    osg::ref_ptr<osg::MatrixTransform> interactorCaseTransform;
    osg::MatrixTransform *parent = nullptr;
    char *_interactorName = nullptr;
    char *labelStr_ = nullptr;

    bool _hit = false;
    bool _intersectionEnabled = true;
    bool _justHit = false;
    bool _wasHit = false;
    bool _standardHL = true;
    osg::ref_ptr<osg::Node> _hitNode; // current node under cursor
    osg::Node *_interactionHitNode = nullptr; // this is the node which was hit, when interaction started

    osg::Vec3 _hitPos;
    vrui::OSGVruiNode *vNode = nullptr;

    osg::ref_ptr<osg::StateSet> _selectedHl, _intersectedHl, _oldHl;

    coVRLabel *label_ = nullptr;
    bool m_isInitializedThroughSharedState = false;
    float _interSize; // size in mm in world coordinates
    float _scale = 1.; // scale factor for retaining screen size of interactor
    std::unique_ptr<vrb::SharedStateBase> m_sharedState;

    // --- Change tracking state ---
    osg::Matrix m_lastMoveMatrix{osg::Matrix::identity()};
    float m_lastScale = 1.f;
    bool m_posChanged = false;
    bool m_rotChanged = false;
    bool m_scaleChanged = false;
    bool m_transformChanged = false;

    // --- Change callbacks ---
    std::function<void(const osg::Vec3 &)> m_onPositionChanged;
    std::function<void(const osg::Quat &)> m_onRotationChanged;
    std::function<void(float)> m_onScaleChanged;
    std::function<void(const osg::Matrix &)> m_onTransformChanged;
    // the geosets are created in the derived classes
    virtual void createGeometry() = 0;

    // scale sphere to keep the size when the world scaling changes
    virtual void keepSize();
    float getScale() const;
    osg::Vec3 restrictToVisibleScene(osg::Vec3);

    const osg::Matrix &getPointerMat() const;

    //! reimplement in derived class for updating value of m_sharedState
    virtual void updateSharedState();

    
public:
    // size: size in world coordinates, the size of the sphere is fixed, even if the user scales the world
    // buttonId: ButtonA, ButtonB etc.
    // iconName: name of the inventor file in covise/icons
    // interactorName: name which appears in the scene graph
    // priority: interaction priority, default medium
    // highliteHitNodeOnly:  true: only the node under the cursor gets highlited - false: if any child node of the geometryNode gets hit all children are highlited
    coVRIntersectionInteractor(float size, coInteraction::InteractionType buttonId, const char *iconName, const char *interactorName, enum coInteraction::InteractionPriority priority, bool highliteHitNodeOnly = false);

    // delete scene graph
    virtual ~coVRIntersectionInteractor();

    // make the interactor intersection sensitive
    void enableIntersection();

    // check whether interactor is enabled
    bool isEnabled();

    // make the interactor intersection insensitive
    void disableIntersection();

    // called every time when the geometry is intersected
    virtual int hit(vrui::vruiHit *hit);

    // called once when the geometry is not intersected any more
    virtual void miss();

    // start the interaction (set selected hl, store dcsmat)
    virtual void startInteraction();

    // move the interactor relatively to it's old position
    // according to the hand movements
    virtual void doInteraction();

    // stop the interaction
    virtual void stopInteraction();

    // make the interactor visible
    void show();

    // make the interactor invisible
    void hide();

    void setInteractorSize(float s);

    // gives information whether this item has been initialized through a sharedState call
    bool isInitializedThroughSharedState();
    //! make state shared among partners in a collaborative session
    virtual void setShared(bool state);

    //! query whether Element state is shared among collaborative partners
    virtual bool isShared() const;

    virtual void addIcon(); // highlight and add

    virtual void removeIcon(); // remove

    virtual void resetState(); // un-highlight

    // return the intersected state
    int isIntersected()
    {
        return _hit;
    }

    // return true if just intesected
    bool wasHit()
    {
        return _wasHit;
    }

    // return hit positon
    osg::Vec3 getHitPos()
    {
        return _hitPos;
    }

    // called in preframe, does the interaction
    virtual void preFrame();

    //return interactor name
    char *getInteractorName()
    {
        return _interactorName;
    }

    ///< class methods for traversing children
    //static vector<coVRIntersectionInteractor*> *interactors; ///< class variable for storing references of children
    //static int traverseIndex; ///< class variable for traversing children

    //static void startTraverseInteractors();
    //static void traverseInteractors();
    //static void stopTraverseInteractors();

    //static osg::Vec3 currentInterPos;
    //static bool isTraverseInteractors;

    osg::Matrix getMatrix()
    {
        return moveTransform->getMatrix();
    }

    const osg::Matrix& getMatrix() const
    {
        return moveTransform->getMatrix();
    }

    float getInteractorSize() const
    {
        return _interSize;
    }
    
    void setCaseTransform(osg::MatrixTransform *);

    // --- Change tracking API ---
    // Derived classes have to mark the corresponding flags when interaction happens for this to work
    // Return whether the respective transform component changed.
    // When reset=true (default), the flag is cleared after reading.
    bool positionChanged(bool reset = true);
    bool rotationChanged(bool reset = true);
    bool scaleChanged(bool reset = true);
    bool changed(bool reset = true);
    void resetChangeFlags();

    // Register callbacks invoked when a change is detected in preFrame.
    void setPositionChangedCallback(std::function<void(const osg::Vec3 &)> cb);
    void setRotationChangedCallback(std::function<void(const osg::Quat &)> cb);
    void setScaleChangedCallback(std::function<void(float)> cb);
    void setTransformChangedCallback(std::function<void(const osg::Matrix &)> cb);
};
}
#endif
