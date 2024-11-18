/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

//
//  Vrml 97 library
//  Copyright (C) 2001 Uwe Woessner
//
//  %W% %G%
//  VrmlNodeCOVISEObject.h

#ifndef _VrmlNodeCOVISEObject_
#define _VrmlNodeCOVISEObject_

#include <util/coTypes.h>

#include <vrml97/vrml/VrmlNode.h>
#include <vrml97/vrml/VrmlSFBool.h>
#include <vrml97/vrml/VrmlSFFloat.h>
#include <vrml97/vrml/VrmlSFVec3f.h>
#include <vrml97/vrml/VrmlSFString.h>
#include <vrml97/vrml/VrmlSFRotation.h>
#include <vrml97/vrml/VrmlNodeChild.h>
#include <vrml97/vrml/VrmlScene.h>

using namespace vrml;
using namespace opencover;

namespace vrml
{

class VRML97COVEREXPORT VrmlNodeCOVISEObject : public VrmlNodeChild
{

public:
    // Define the fields of ARSensor nodes
    static void initFields(VrmlNodeCOVISEObject *node, vrml::VrmlNodeType *t);
    static const char *name();

    VrmlNodeCOVISEObject(VrmlScene *scene = 0);
    VrmlNodeCOVISEObject(const VrmlNodeCOVISEObject &n);
    virtual ~VrmlNodeCOVISEObject();

    virtual VrmlNodeCOVISEObject *toCOVISEObject() const;

    virtual void render(Viewer *);

    static void addNode(osg::Node *n);

    static list<VrmlNodeCOVISEObject *> COVISEObjectNodes;

private:
    // Fields
    VrmlSFString d_objectName;

    void addCoviseNode(osg::Node *node);

    osg::ref_ptr<osg::Group> group;

    Viewer::Object d_viewerObject;
};
}

#endif //_VrmlNodeCOVISEObject_
