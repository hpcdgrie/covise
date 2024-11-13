/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

//
//  Vrml 97 library
//  Copyright (C) 1998 Chris Morley
//
//  %W% %G%
//  VrmlNode.h

#ifndef _VRMLNODE_
#define _VRMLNODE_

#include "config.h"
#include "System.h"

#include <iostream>
#include <list>

#include "VrmlSFNode.h"

namespace vrml
{

class VrmlNode;
typedef std::list<VrmlNode *> VrmlNodeList;

class Route;
class Viewer;

class VrmlNamespace;
class VrmlNodeType;
class VrmlField;
class VrmlScene;


class VRMLEXPORT VrmlNode
{
    friend std::ostream &operator<<(std::ostream &os, const VrmlNode &f);

public:
    typedef std::list<VrmlNode *> ParentList;
    ParentList parentList;

    // Define the fields of all built in VrmlNodeTypes
    static VrmlNodeType *defineType(VrmlNodeType *t);
    virtual VrmlNodeType *nodeType() const;

    // VrmlNodes are reference counted, optionally named objects
    // The reference counting is manual (that is, each user of a
    // VrmlNode, such as the VrmlMFNode class, calls reference()
    // and dereference() explicitly). Should make it internal...

    VrmlNode(VrmlScene *s);
    VrmlNode(const VrmlNode &);
    virtual ~VrmlNode() = 0;

    // Copy the node, defining its name in the specified scope.
    // Uses the flag to determine whether the node is a USEd node.
    VrmlNode *clone(VrmlNamespace *);
    virtual VrmlNode *cloneMe() const = 0;
    virtual void cloneChildren(VrmlNamespace *);

    // Copy the ROUTEs
    virtual void copyRoutes(VrmlNamespace *ns);

    // Add/remove references to a VrmlNode. This is silly, as it
    // requires the users of VrmlNode to do the reference/derefs...
    VrmlNode *reference();
    void dereference();

    template<typename Derived>
    Derived *as() {
        return dynamic_cast<Derived*>(this);
    }
    template<typename Derived>
    const Derived *as() const {
        return dynamic_cast<const Derived*>(this);
    }
    template<typename...Deriveds>
    bool is() const {
        return (... || dynamic_cast<const Deriveds*>(this));
    }
    // Node DEF/USE/ROUTE name
    void setName(const char *nodeName, VrmlNamespace *ns = 0);
    inline const char *name() const
    {
        return d_name.c_str();
    };
    VrmlNamespace *getNamespace() const;

    // Add to a scene. A node can belong to at most one scene for now.
    // If it doesn't belong to a scene, it can't be rendered.
    virtual void addToScene(VrmlScene *, const char *relativeUrl);

    // Write self
    std::ostream &print(std::ostream &os, int indent) const;
    virtual std::ostream &printFields(std::ostream &os, int indent);
    static std::ostream &printField(std::ostream &, int, const char *, const VrmlField &);

    // Indicate that the node state has changed, need to re-render
    void setModified();
    void clearModified();

    virtual bool isModified() const;
    void forceTraversal(bool once = true, int increment = 1);
    void decreaseTraversalForce(int num = -1);
    int getTraversalForce();
    bool haveToRender();

    // A generic flag (typically used to find USEd nodes).
    void setFlag()
    {
        d_flag = true;
    }
    virtual void clearFlags(); // Clear childrens flags too.
    bool isFlagSet()
    {
        return d_flag;
    }

    // Add a ROUTE from a field in this node
    Route *addRoute(const char *fromField, VrmlNode *toNode, const char *toField);

    void removeRoute(Route *ir);
    void addRouteI(Route *ir);

    // Delete a ROUTE from a field in this node
    void deleteRoute(const char *fromField, VrmlNode *toNode, const char *toField);

    void repairRoutes();

    // Pass a named event to this node. This method needs to be overridden
    // to support any node-specific eventIns behaviors, but exposedFields
    // (should be) handled here...
    virtual void eventIn(double timeStamp,
                         const char *eventName,
                         const VrmlField *fieldValue);

    // Set a field by name (used by the parser, not for external consumption).
    virtual void setField(const char *fieldName,
                          const VrmlField &fieldValue);

    // Get a field or eventOut by name.
    virtual const VrmlField *getField(const char *fieldName) const;

    // Return an eventOut/exposedField value. Used by the script node
    // to access the node fields.
    const VrmlField *getEventOut(const char *fieldName) const;

    // Do nothing. Renderable nodes need to redefine this.
    virtual void render(Viewer *);

    // Do nothing. Grouping nodes need to redefine this.
    virtual void accumulateTransform(VrmlNode *);

    virtual VrmlNode *getParentTransform();

    // Compute an inverse transform (either render it or construct the matrix)
    virtual void inverseTransform(Viewer *);
    virtual void inverseTransform(double *mat);

    // return number of coordinates in Coordinate/CoordinateDouble node
    virtual int getNumberCoordinates()
    {
        return 0;
    }

    // search for a node with the name of a EXPORT (AS) command
    // either inside a Inline node or inside a node created at real time
    virtual VrmlNode *findInside(const char *)
    {
        return NULL;
    }

    VrmlScene *scene() const
    {
        return d_scene;
    }

    virtual bool isOnlyGeometry() const;

protected:
    enum
    {
        INDENT_INCREMENT = 4
    };

    // Send a named event from this node.
    void eventOut(double timeStamp,
                  const char *eventName,
                  const VrmlField &fieldValue);

    // Scene this node belongs to
    VrmlScene *d_scene;

    // True if a field changed since last render
    bool d_modified;
    bool d_flag;

    // Routes from this node (clean this up, add RouteList ...)
    Route *d_routes;
    Route *d_incomingRoutes;

    //
    VrmlNamespace *d_myNamespace;
    // true if this node is on the static node stack
    bool isOnStack(VrmlNode *);
    static VrmlNodeList nodeStack;

    // <0:  render every frame
    // >=0: render at this very frame
    int d_traverseAtFrame;

    bool d_isDeletedInline;

private:
    int d_refCount; // Number of active references
    std::string d_name;

    VrmlSFNode d_metadata;
};

// Routes
class Route
{
public:
    Route(const char *fromEventOut, VrmlNode *toNode, const char *toEventIn, VrmlNode *fromNode);
    Route(const Route &);
    ~Route();

    char *fromEventOut()
    {
        return d_fromEventOut;
    }
    char *toEventIn()
    {
        return d_toEventIn;
    }
    VrmlNode *toNode()
    {
        return d_toNode;
    }
    VrmlNode *fromNode()
    {
        return d_fromNode;
    }

    void addFromImportName(const char *name);
    void addToImportName(const char *name);

    Route *newFromRoute(VrmlNode *newFromNode);
    Route *newToRoute(VrmlNode *newToNode);

    VrmlNode *newFromNode(void);
    VrmlNode *newToNode(void);

    Route *prev()
    {
        return d_prev;
    }
    Route *next()
    {
        return d_next;
    }
    void setPrev(Route *r)
    {
        d_prev = r;
    }
    void setNext(Route *r)
    {
        d_next = r;
    }
    Route *prevI()
    {
        return d_prevI;
    }
    Route *nextI()
    {
        return d_nextI;
    }
    void setPrevI(Route *r)
    {
        d_prevI = r;
    }
    void setNextI(Route *r)
    {
        d_nextI = r;
    }

private:
    char *d_fromEventOut;
    VrmlNode *d_toNode;
    VrmlNode *d_fromNode;
    char *d_toEventIn;

    char *d_fromImportName;
    char *d_toImportName;

    Route *d_prev, *d_next;
    Route *d_prevI, *d_nextI;
};
}
// Ugly macro used in printFields
#define PRINT_FIELD(_f) printField(os, indent + INDENT_INCREMENT, #_f, d_##_f)

// Ugly macros used in setField
// use VrmlNodeChildTemplate to avoid setField and this mess in the future

#define TRY_FIELD(_f, _t)                                                                                                                                  \
    (strcmp(fieldName, #_f) == 0)                                                                                                                          \
    {                                                                                                                                                      \
        if (fieldValue.to##_t())                                                                                                                           \
            d_##_f = (Vrml##_t &)fieldValue;                                                                                                               \
        else                                                                                                                                               \
            System::the->error("Invalid type (%s) for %s field of %s node (expected %s).\n", fieldValue.fieldTypeName(), #_f, nodeType()->getName(), #_t); \
    }

// For SFNode fields. Allow un-fetched EXTERNPROTOs to succeed...
#define TRY_SFNODE_FIELD(_f, _n)                                                                                                                           \
    (strcmp(fieldName, #_f) == 0)                                                                                                                          \
    {                                                                                                                                                      \
        VrmlSFNode *x = (VrmlSFNode *)&fieldValue;                                                                                                         \
        if (fieldValue.toSFNode() && ((!x->get()) || x->get()->to##_n() || x->get()->toProto()))                                                           \
            d_##_f = (VrmlSFNode &)fieldValue;                                                                                                             \
        else                                                                                                                                               \
            System::the->error("Invalid type (%s) for %s field of %s node (expected %s).\n", fieldValue.fieldTypeName(), #_f, nodeType()->getName(), #_n); \
    }

#define TRY_NAMED_FIELD(_f, _n, _t)                                                                                                                                \
    (strcmp(fieldName, #_n) == 0)                                                                                                                                  \
    {                                                                                                                                                              \
        if (fieldValue.to##_t())                                                                                                                                   \
            d_##_f = (Vrml##_t &)fieldValue;                                                                                                                       \
        else                                                                                                                                                       \
            System::the->error("Invalid type (%s) for %s/%s field of %s node (expected %s).\n", fieldValue.fieldTypeName(), #_f, #_n, nodeType()->getName(), #_t); \
    }

#define TRY_SFNODE_FIELD2(_f, _n1, _n2)                                                                                                                                 \
    (strcmp(fieldName, #_f) == 0)                                                                                                                                       \
    {                                                                                                                                                                   \
        VrmlSFNode *x = (VrmlSFNode *)&fieldValue;                                                                                                                      \
        if (fieldValue.toSFNode() && ((!x->get()) || x->get()->to##_n1() || x->get()->to##_n2() || x->get()->toProto()))                                                \
            d_##_f = (VrmlSFNode &)fieldValue;                                                                                                                          \
        else                                                                                                                                                            \
            System::the->error("Invalid type (%s) for %s field of %s node (expected %s or %s).\n", fieldValue.fieldTypeName(), #_f, nodeType()->getName(), #_n1, #_n2); \
    }

#define TRY_SFNODE_FIELD3(_f, _n1, _n2, _n3)                                                                                                                                        \
    (strcmp(fieldName, #_f) == 0)                                                                                                                                                   \
    {                                                                                                                                                                               \
        VrmlSFNode *x = (VrmlSFNode *)&fieldValue;                                                                                                                                  \
        if (fieldValue.toSFNode() && ((!x->get()) || x->get()->to##_n1() || x->get()->to##_n2() || x->get()->to##_n3() || x->get()->toProto()))                                     \
            d_##_f = (VrmlSFNode &)fieldValue;                                                                                                                                      \
        else                                                                                                                                                                        \
            System::the->error("Invalid type (%s) for %s field of %s node (expected %s or %s or %s).\n", fieldValue.fieldTypeName(), #_f, nodeType()->getName(), #_n1, #_n2, #_n3); \
    }

#define TRY_SFNODE_FIELD4(_f, _n1, _n2, _n3, _n4)                                                                                                                                               \
    (strcmp(fieldName, #_f) == 0)                                                                                                                                                               \
    {                                                                                                                                                                                           \
        VrmlSFNode *x = (VrmlSFNode *)&fieldValue;                                                                                                                                              \
        if (fieldValue.toSFNode() && ((!x->get()) || x->get()->to##_n1() || x->get()->to##_n2() || x->get()->to##_n3() || x->get()->to##_n4() || x->get()->toProto()))                          \
            d_##_f = (VrmlSFNode &)fieldValue;                                                                                                                                                  \
        else                                                                                                                                                                                    \
            System::the->error("Invalid type (%s) for %s field of %s node (expected %s or %s or %s or $s).\n", fieldValue.fieldTypeName(), #_f, nodeType()->getName(), #_n1, #_n2, #_n3, #_n4); \
    }

#define TRY_SFNODE_FIELD5(_f, _n1, _n2, _n3, _n4, _n5)                                                                                                                                        \
    (strcmp(fieldName, #_f) == 0)                                                                                                                                                             \
    {                                                                                                                                                                                         \
        VrmlSFNode *x = (VrmlSFNode *)&fieldValue;                                                                                                                                            \
        if (fieldValue.toSFNode() && ((!x->get()) || x->get()->to##_n1() || x->get()->to##_n2() || x->get()->to##_n3() || x->get()->to##_n4() || x->get()->to##_n5() || x->get()->toProto())) \
            d_##_f = (VrmlSFNode &)fieldValue;                                                                                                                                                \
        else                                                                                                                                                                                  \
            System::the->error("Invalid type (%s) for %s field of %s node (expected %s or %s or %s or %s or %s).\n", fieldValue.fieldTypeName(), #_f, nodeType()->getName(), #_n1, #_n2, #_n3, #_n4, #_n5);

#define TRY_SFNODE_FIELD6(_f, _n1, _n2, _n3, _n4, _n5, _n6)                                                                                                                                                             \
    (strcmp(fieldName, #_f) == 0)                                                                                                                                                                                       \
    {                                                                                                                                                                                                                   \
        VrmlSFNode *x = (VrmlSFNode *)&fieldValue;                                                                                                                                                                      \
        if (fieldValue.toSFNode() && ((!x->get()) || x->get()->to##_n1() || x->get()->to##_n2() || x->get()->to##_n3() || x->get()->to##_n4() || x->get()->to##_n5() || x->get()->to##_n6() || x->get()->toProto()))    \
            d_##_f = (VrmlSFNode &)fieldValue;                                                                                                                                                                          \
        else                                                                                                                                                                                                            \
            System::the->error("Invalid type (%s) for %s field of %s node (expected %s or %s or %s or %s or %s or %s).\n", fieldValue.fieldTypeName(), #_f, nodeType()->getName(), #_n1, #_n2, #_n3, #_n4, #_n5, #_n6); \
    }
#endif //_VRMLNODE_
