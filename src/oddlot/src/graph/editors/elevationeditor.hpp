/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 /**************************************************************************
 ** ODD: OpenDRIVE Designer
 **   Frank Naegele (c) 2010
 **   <mail@f-naegele.de>
 **   21.06.2010
 **
 **************************************************************************/

#ifndef ELEVATIONEDITOR_HPP
#define ELEVATIONEDITOR_HPP

#include "projecteditor.hpp"

class ProjectData;

class TopviewGraph;
class ProfileGraph;

class RSystemElementRoad;
class ElevationSection;

class SectionHandle;
class ElevationMoveHandle;

class RoadSystemItem;

class ElevationRoadSystemItem;
class ElevationRoadItem;
class ElevationSectionItem;
class ElevationRoadPolynomialItem;

#include <QPointF>
#include <QMap>
#include <QMultiMap>
#include <QRectF>
#include <QGraphicsItem>


class ElevationEditor : public ProjectEditor
{
    Q_OBJECT

        //################//
        // FUNCTIONS      //
        //################//

public:
    explicit ElevationEditor(ProjectWidget *projectWidget, ProjectData *projectData, TopviewGraph *topviewGraph, ProfileGraph *profileGraph);
    virtual ~ElevationEditor();

    // TODO look for better solution
    SectionHandle *getInsertSectionHandle();

    // ProfileGraph //
    //
    ProfileGraph *getProfileGraph() const
    {
        return profileGraph_;
    }

    // Smooth Radius //
    //
    double getSmoothRadius() const
    {
        return smoothRadius_;
    }

    //Get selected roads //
    //
    ElevationRoadPolynomialItem *getSelectedElevationItem()
    {
        return selectedElevationItem_;
    };

    ElevationRoadPolynomialItem *getSelectedAdjacentElevationItem()
    {
        return selectedAdjacentElevationItem_;
    };

    // Selected Roads //
    //
    void addSelectedRoad(ElevationRoadPolynomialItem *roadItem);
    void delSelectedRoad(RSystemElementRoad *road);
    void delSelectedRoads();
    void insertSelectedRoad(RSystemElementRoad *road);
    void initBox();
    void fitView();
    QMap<QGraphicsItem *, ElevationSection *> getSelectedElevationSections(int count);

    // MoveHandles //
    //
    void registerMoveHandle(ElevationMoveHandle *handle);
    int unregisterMoveHandle(ElevationMoveHandle *handle);
    bool translateMoveHandles(const QPointF &pressPos, const QPointF &mousePos);
    bool selectionChangedRoadSection();

    // ElevationSections //
    //
    bool registerElevationSection(QGraphicsItem *item, ElevationSection *section);
    bool deregisterElevationSection(QGraphicsItem *item);

    // Tool, Mouse & Key //
    //
    virtual void toolAction(ToolAction *toolAction);
    // virtual void mouseAction(MouseAction *mouseAction);
    // virtual void   keyAction(KeyAction * keyAction);


protected:
    virtual void init();
    virtual void kill();

private:
    ElevationEditor(); /* not allowed */
    ElevationEditor(const ElevationEditor &); /* not allowed */
    ElevationEditor &operator=(const ElevationEditor &); /* not allowed */

    void arrangeAdjacentRoads(ElevationSection *section, RSystemElementRoad *road, ElevationSection *adjacentSection, RSystemElementRoad *adjacentRoad);
    void clearToolObjectSelection();

    //################//
    // SLOTS          //
    //################//

public slots:
    // Parameter Settings //
    //
    virtual void apply();
    virtual void reject();
    virtual void reset();

    //################//
    // PROPERTIES     //
    //################//

private:
    // Graph //
    //
    ElevationRoadSystemItem *roadSystemItem_;

    // ProfileGraph //
    //
    ProfileGraph *profileGraph_;
    RoadSystemItem *roadSystemItemPolyGraph_;

    // TODO look for better solution
    SectionHandle *insertSectionHandle_;

    // Smooth Radius and Slope //
    //
    double smoothRadius_;
    double slope_;

    // ProfileGraph: Selected Item //
    //
    QList<RSystemElementRoad *> selectedRoads_;
    ElevationRoadPolynomialItem *selectedElevationItem_;
    ElevationRoadPolynomialItem *selectedAdjacentElevationItem_;

    QMultiMap<int, ElevationMoveHandle *> selectedMoveHandles_;

    // Bounding Box for all selected roads //
    //
    QRectF boundingBox_;
    qreal xtrans_;


    // Elevation Editing //
    //
    QGraphicsItem *elevationSectionItem_;
    QGraphicsItem *elevationSectionAdjacentItem_;
};

#endif // ELEVATIONEDITOR_HPP
