/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

//
//  Vrml 97 library
//  Copyright (C) 1998 Chris Morley
//  %W% %G%
#ifndef _VRMLNODEPROTO_
#define _VRMLNODEPROTO_

//
// A VrmlNodeProto object represents an instance of a PROTOd node.
// The definition of the PROTO is stored in a VrmlNodeType object;
// the VrmlNodeProto object stores a local copy of the implementation
// nodes.
//

#include "config.h"
#include "VrmlNode.h"
#include "VrmlNodeType.h"
#include "Viewer.h"

namespace vrml
{

class VrmlMFNode;

class VRMLEXPORT VrmlNodeProto : public VrmlNode
{

public:
    virtual VrmlNodeType *nodeType() const;

    VrmlNodeProto(VrmlNodeType *nodeDef, VrmlScene *scene);
    VrmlNodeProto(const VrmlNodeProto &);
    virtual ~VrmlNodeProto();

    virtual VrmlNode *cloneMe() const;

    virtual void addToScene(VrmlScene *, const char *relUrl);
    virtual std::ostream &printFields(std::ostream &os, int indent);

    virtual void render(Viewer *);

    virtual void eventIn(double timeStamp,
                         const char *eventName,
                         const VrmlField *fieldValue);

    virtual bool isModified() const;

    virtual void setField(const char *fieldName, const VrmlField &fieldValue);

    virtual const VrmlField *getField(const char *fieldName) const;

    virtual void accumulateTransform(VrmlNode *);

    // LarryD  Feb 11/99
    int size();
    // LarryD  Feb 11/99
    VrmlNode *child(int index);

    // LarryD Feb 11/99
    VrmlMFNode *getNodes()
    {
        return d_nodes;
    }

    // Field name/value pairs specified in PROTO instantiation
    typedef struct
    {
        char *name;
        VrmlField *value;
    } NameValueRec;

    virtual Viewer::Object getViewerObject()
    {
        return d_viewerObject;
    };

private:
    VrmlNode *firstNode() const;

    // Instantiate the proto by cloning the node type implementation nodes.
    void instantiate(const char* relUrl = nullptr, int parentId = -1);

    // Find a field by name
    NameValueRec *findField(const char *fieldName) const;

    VrmlNodeType *d_nodeType; // Definition

    bool d_instantiated;
    VrmlNamespace *d_scope; // Node type and name bindings

    VrmlMFNode *d_nodes; // Local copy of implementation nodes.

    std::list<NameValueRec *> d_fields; // Field values

    // Dispatch eventIns from outside the PROTO to internal eventIns
    typedef struct
    {
        char *name;
        VrmlNodeType::ISMap ismap;
    } EventDispatch;

    typedef std::list<EventDispatch *> EventDispatchList;

    EventDispatchList d_eventDispatch;

    Viewer::Object d_viewerObject; // move to VrmlNode.h ? ...
};
}
#endif //_VRMLNODEPROTO_
