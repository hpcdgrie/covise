/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

/**************************************************************************
** ODD: OpenDRIVE Designer
**   Frank Naegele (c) 2010
**   <mail@f-naegele.de>
**   05.07.2010
**
**************************************************************************/

#include "signalroadsystemitem.hpp"

// Data //
//
#include "src/data/roadsystem/roadsystem.hpp"
#include "src/data/roadsystem/rsystemelementroad.hpp"
#include "src/data/roadsystem/rsystemelementcontroller.hpp"

// Graph //
//
#include "src/graph/items/roadsystem/signal/signalroaditem.hpp"
#include "src/graph/items/roadsystem/controlleritem.hpp"

//################//
// CONSTRUCTOR    //
//################//

SignalRoadSystemItem::SignalRoadSystemItem(TopviewGraph *topviewGraph, RoadSystem *roadSystem)
    : RoadSystemItem(topviewGraph, roadSystem)
{
    init();
}

SignalRoadSystemItem::~SignalRoadSystemItem()
{
}

void
SignalRoadSystemItem::init()
{
    foreach (RSystemElementRoad *road, getRoadSystem()->getRoads())
    {
        new SignalRoadItem(this, road);
    }

    // Controllers //
    //
    foreach (RSystemElementController *controller, getRoadSystem()->getControllers())
    {
        ControllerItem *item = new ControllerItem(this, controller);
        item->setZValue(-1.0);
        controllerItems_.append(item);
    }
}

bool
SignalRoadSystemItem::removeControllerItem(ControllerItem *controllerItem)
{
    return controllerItems_.removeAll(controllerItem);
}

void
SignalRoadSystemItem::setControllersSelectable(bool selectable)
{
    foreach(ControllerItem * item, controllerItems_)
    {
        item->setFlag(QGraphicsItem::ItemIsSelectable, selectable);
    }
}


//##################//
// Observer Pattern //
//##################//

void
SignalRoadSystemItem::updateObserver()
{
    // Parent //
    //
    RoadSystemItem::updateObserver();
    if (isInGarbage())
    {
        return; // will be deleted anyway
    }

    // RoadSystem //
    //
    int changes = getRoadSystem()->getRoadSystemChanges();

    if (changes & RoadSystem::CRS_RoadChange)
    {
        // A road has been added (or deleted - but that will be handled by the road item itself).
        //
        foreach (RSystemElementRoad *road, getRoadSystem()->getRoads())
        {
            if ((road->getDataElementChanges() & DataElement::CDE_DataElementCreated)
                || (road->getDataElementChanges() & DataElement::CDE_DataElementAdded))
            {
                // RoadSignalItem //
                //
                new SignalRoadItem(this, road);
            }
        }
    }

    if (changes & RoadSystem::CRS_ControllerChange)
    {
        // Controllers //
        //
        foreach (RSystemElementController *controller, getRoadSystem()->getControllers())
        {
            if ((controller->getDataElementChanges() & DataElement::CDE_DataElementCreated)
                || (controller->getDataElementChanges() & DataElement::CDE_DataElementAdded))
            {
                ControllerItem *item = new ControllerItem(this, controller);
                item->setZValue(-1.0);
                controllerItems_.append(item);
            }
        }
    }
}
