/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

//
//  Vrml 97 library
//  Copyright (C) 2001 Uwe Woessner
//
//  %W% %G%
//  VrmlNodeVariant.h

#ifndef _VRMLNODEVariant_
#define _VRMLNODEVariant_

#include <util/coTypes.h>

#include <vrml97/vrml/VrmlNode.h>
#include <vrml97/vrml/VrmlSFBool.h>
#include <vrml97/vrml/VrmlSFInt.h>
#include <vrml97/vrml/VrmlSFFloat.h>
#include <vrml97/vrml/VrmlSFVec3f.h>
#include <vrml97/vrml/VrmlSFString.h>
#include <vrml97/vrml/VrmlSFRotation.h>
#include <vrml97/vrml/VrmlNodeChild.h>
#include <vrml97/vrml/VrmlScene.h>

class VariantConnection;

namespace opencover
{
class ARMarker;
}
using namespace opencover;
using namespace vrml;

class VrmlNodeVariant : public VrmlNodeChild
{

public:
    static VrmlNodeVariant *theVariantNode;
    static VrmlNodeVariant *instance();
    // Define the fields of Variant nodes
    static void initFields(VrmlNodeVariant *node, VrmlNodeType *t);
    static const char *name();

    VrmlNodeVariant(VrmlScene *scene = 0);
    VrmlNodeVariant(const VrmlNodeVariant &n);
    virtual ~VrmlNodeVariant();
    virtual void addToScene(VrmlScene *s, const char *);

    virtual VrmlNodeVariant *toVariant() const;

    virtual void render(Viewer *);


    std::string getVariant(){return d_variant.get();};
    void setVariant(std::string varName);


private:
    // Fields
    
    VrmlSFString d_variant;
};

template<>
inline VrmlNode *VrmlNode::creator<VrmlNodeVariant>(vrml::VrmlScene *scene){
    return VrmlNodeVariant::instance();
}

#endif //_VRMLNODEVariant_
