/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

//
//  Vrml 97 library
//  Copyright (C) 2001 Uwe Woessner
//
//  %W% %G%
//  VrmlNodeClippingPlane.h

#ifndef _VrmlNodeClippingPlane_
#define _VrmlNodeClippingPlane_

#include <util/coTypes.h>

#include <vrml97/vrml/VrmlNode.h>
#include <vrml97/vrml/VrmlSFBool.h>
#include <vrml97/vrml/VrmlSFFloat.h>
#include <vrml97/vrml/VrmlSFInt.h>
#include <vrml97/vrml/VrmlSFVec3f.h>
#include <vrml97/vrml/VrmlSFString.h>
#include <vrml97/vrml/VrmlSFRotation.h>
#include <vrml97/vrml/VrmlNodeChild.h>
#include <vrml97/vrml/VrmlScene.h>

using namespace vrml;
using namespace opencover;

namespace vrml
{

class VRML97COVEREXPORT VrmlNodeClippingPlane : public VrmlNodeGroup
{

public:
    static void initFields(VrmlNodeClippingPlane *node, vrml::VrmlNodeType *t);
    static const char *name();

    VrmlNodeClippingPlane(VrmlScene *scene = 0);
    VrmlNodeClippingPlane(const VrmlNodeClippingPlane &n);

    virtual void render(Viewer *);

    bool isEnabled()
    {
        return d_enabled.get();
    }

private:
    // Fields
    VrmlSFBool d_global;
    VrmlSFBool d_enabled;
    VrmlSFVec3f d_position;
    VrmlSFRotation d_orientation;
    VrmlSFInt d_number;

    Viewer::Object d_clipObject;
};
}

#endif //_VrmlNodeClippingPlane_
