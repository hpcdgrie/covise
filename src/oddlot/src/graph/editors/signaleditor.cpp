/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 /**************************************************************************
 ** ODD: OpenDRIVE Designer
 **   Frank Naegele (c) 2010
 **   <mail@f-naegele.de>
 **   15.03.2010
 **
 **************************************************************************/

#include "signaleditor.hpp"

 // Project //
 //
#include "src/gui/projectwidget.hpp"

//MainWindow //
//
#include "src/mainwindow.hpp"

// Data //
//
#include "src/data/projectdata.hpp"
#include "src/data/signalmanager.hpp"
#include "src/data/roadsystem/roadsystem.hpp"
#include "src/data/roadsystem/rsystemelementcontroller.hpp"
#include "src/data/roadsystem/rsystemelementroad.hpp"
#include "src/data/roadsystem/sections/signalobject.hpp"
#include "src/data/roadsystem/sections/lanesection.hpp"

// Commands //
//
#include "src/data/commands/controllercommands.hpp"
#include "src/data/commands/signalcommands.hpp"
#include "src/data/commands/roadsectioncommands.hpp"
#include "src/data/commands/dataelementcommands.hpp"

// Graph //
//
#include "src/graph/topviewgraph.hpp"
#include "src/graph/graphscene.hpp"
#include "src/graph/graphview.hpp"
#include "src/graph/profilegraph.hpp"
#include "src/graph/profilegraphscene.hpp"
#include "src/graph/profilegraphview.hpp"

#include "src/graph/items/roadsystem/signal/signalroadsystemitem.hpp"
#include "src/graph/items/roadsystem/signal/signalroaditem.hpp"
#include "src/graph/items/roadsystem/signal/signalsectionpolynomialitems.hpp"
#include "src/graph/items/roadsystem/signal/signalitem.hpp"
#include "src/graph/items/roadsystem/signal/objectitem.hpp"
#include "src/graph/items/roadsystem/signal/bridgeitem.hpp"

#include "src/graph/items/roadsystem/signal/signalhandle.hpp"
#include "src/graph/items/roadsystem/controlleritem.hpp"

// Tools //
//
#include "src/gui/tools/signaleditortool.hpp"
#include "src/gui/mouseaction.hpp"

// Tree //
//
#include "src/tree/signaltreewidget.hpp"

// GUI //
//
#include "src/gui/parameters/toolvalue.hpp"
#include "src/gui/parameters/toolparametersettings.hpp"

// Visitor //
//
//#include "src/graph/visitors/roadmarkvisitor.hpp"

// Qt //
//
#include <QGraphicsItem>
#include <QUndoStack>

//################//
// CONSTRUCTORS   //
//################//

SignalEditor::SignalEditor(ProjectWidget *projectWidget, ProjectData *projectData, TopviewGraph *topviewGraph, ProfileGraph *profileGraph)
    : ProjectEditor(projectWidget, projectData, topviewGraph)
    , topviewGraph_(topviewGraph)
    , profileGraph_(profileGraph)
    , signalRoadSystemItem_(NULL)
    , insertSignalHandle_(NULL)
    , controller_(NULL)
{
    MainWindow *mainWindow = projectWidget->getMainWindow();
    signalTree_ = mainWindow->getSignalTree();
    signalManager_ = mainWindow->getSignalManager();
}

SignalEditor::~SignalEditor()
{
    kill();
}

//################//
// FUNCTIONS      //
//################//

/**
*
*/
void
SignalEditor::init()
{
    if (!signalRoadSystemItem_)
    {
        // Root item //
        //
        signalRoadSystemItem_ = new SignalRoadSystemItem(topviewGraph_, getProjectData()->getRoadSystem());
        topviewGraph_->getScene()->addItem(signalRoadSystemItem_);

        signalRoadSystemItem_->setJunctionsSelectable(false);

        // Signal Handle //
        //
        insertSignalHandle_ = new SignalHandle(signalRoadSystemItem_);
        insertSignalHandle_->hide();

    }

    profileGraph_->getScene()->setSceneRect(-200.0, -50.0, 400.0, 100.0);

    // Raise Signal Tree //
    //
    signalTree_->setSignalEditor(this);
}

/*!
*/
void
SignalEditor::kill()
{
    if (tool_)
    {
        reset();
        ODD::mainWindow()->showParameterDialog(false);
    }

    delete signalRoadSystemItem_;
    signalRoadSystemItem_ = NULL;
    signalTree_->setSignalEditor(NULL);
}


SignalHandle *
SignalEditor::getInsertSignalHandle() const
{
    if (!insertSignalHandle_)
    {
        qDebug("ERROR 1003281422! SignalEditor not yet initialized.");
    }
    return insertSignalHandle_;
}

void
SignalEditor::duplicate()
{
    QList<QGraphicsItem *> selectedItems = topviewGraph_->getScene()->selectedItems();

    foreach(QGraphicsItem * item, selectedItems)
    {
        SignalItem *signalItem = dynamic_cast<SignalItem *>(item);
        if (signalItem)
        {
            signalItem->duplicate();
        }
        else
        {
            ObjectItem *objectItem = dynamic_cast<ObjectItem *>(item);
            if (objectItem)
            {
                objectItem->duplicate();
            }
            else
            {
                BridgeItem *bridgeItem = dynamic_cast<BridgeItem *>(item);
                if (bridgeItem)
                {
                    bridgeItem->duplicate();
                }
            }
        }
    }

}

void
SignalEditor::move(QPointF &diff)
{
    QList<QGraphicsItem *> selectedItems = topviewGraph_->getScene()->selectedItems();

    foreach(QGraphicsItem * item, selectedItems)
    {
        SignalItem *signalItem = dynamic_cast<SignalItem *>(item);
        if (signalItem)
        {
            signalItem->move(diff);
        }
        else
        {
            ObjectItem *objectItem = dynamic_cast<ObjectItem *>(item);
            if (objectItem)
            {
                objectItem->move(diff);
            }
            else
            {
                BridgeItem *bridgeItem = dynamic_cast<BridgeItem *>(item);
                if (bridgeItem)
                {
                    bridgeItem->move(diff);
                }
            }
        }
    }

}

void
SignalEditor::translateSignal(SignalItem *signalItem, QPointF &diff)
{
    Signal *signal = signalItem->getSignal();
    RSystemElementRoad *road = signal->getParentRoad();
    double s;
    QVector2D vec;
    double dist;
    QPointF to = road->getGlobalPoint(signal->getSStart(), signal->getT()) + diff;
    RSystemElementRoad *newRoad = getProjectData()->getRoadSystem()->findClosestRoad(to, s, dist, vec);

    if (newRoad != road)
    {
        RemoveSignalCommand *removeSignalCommand = new RemoveSignalCommand(signal, road);
        getProjectGraph()->executeCommand(removeSignalCommand);

        AddSignalCommand *addSignalCommand = new AddSignalCommand(signal, newRoad);
        getProjectGraph()->executeCommand(addSignalCommand);
        signal->setElementSelected(false);

        road = newRoad;
    }

    LaneSection *laneSection = road->getLaneSection(s);
    int validToLane = 0;
    int validFromLane = 0;
    if (dist < 0)
    {
        validToLane = laneSection->getRightmostLaneId();
    }
    else
    {
        validFromLane = laneSection->getLeftmostLaneId();
    }

    SetSignalPropertiesCommand *signalPropertiesCommand = new SetSignalPropertiesCommand(signal, signal->getId(), signal->getName(), dist, signal->getDynamic(), signal->getOrientation(), signal->getZOffset(), signal->getCountry(), signal->getType(), signal->getTypeSubclass(), signal->getSubtype(), signal->getValue(), signal->getHeading(), signal->getPitch(), signal->getRoll(), signal->getUnit(), signal->getText(), signal->getWidth(), signal->getHeight(), signal->getPole(), signal->getSize(), validFromLane, validToLane, signal->getCrossingProbability(), signal->getResetTime());
    getProjectGraph()->executeCommand(signalPropertiesCommand);
    MoveRoadSectionCommand *moveSectionCommand = new MoveRoadSectionCommand(signal, s, RSystemElementRoad::DRS_SignalSection);
    getProjectGraph()->executeCommand(moveSectionCommand);
}

void
SignalEditor::translate(QPointF &diff)
{

    getProjectData()->getUndoStack()->beginMacro(QObject::tr("Move Signal"));

    QList<QGraphicsItem *> selectedItems = topviewGraph_->getScene()->selectedItems();

    foreach(QGraphicsItem * item, selectedItems)
    {
        SignalItem *signalItem = dynamic_cast<SignalItem *>(item);
        if (signalItem)
        {
            translateSignal(signalItem, diff);
        }
        else
        {
            ObjectItem *objectItem = dynamic_cast<ObjectItem *>(item);
            if (objectItem)
            {
                translateObject(objectItem, diff);
            }
            else
            {
                BridgeItem *bridgeItem = dynamic_cast<BridgeItem *>(item);
                if (bridgeItem)
                {
                    translateBridge(bridgeItem, diff);
                }
            }
        }
    }

    getProjectData()->getUndoStack()->endMacro();
}

Signal *
SignalEditor::addSignalToRoad(RSystemElementRoad *road, double s, double t, bool isProfileGraph, double zOffset)
{
    int validToLane = 0; // make a new signal //
    int validFromLane = 0;

    LaneSection *laneSection = road->getLaneSection(s);
    if (t < 0)
    {
        validToLane = laneSection->getRightmostLaneId();
    }
    else
    {
        validFromLane = laneSection->getLeftmostLaneId();
    }

    SignalContainer *lastSignal = signalManager_->getSelectedSignalContainer();
    Signal *newSignal = NULL;

    if (lastSignal)
    {
        if (!isProfileGraph)
        {
            if (t < 0)
            {
                t -= lastSignal->getSignalDistance();
            }
            else
            {
                t += lastSignal->getSignalDistance();
            }
            zOffset = lastSignal->getSignalheightOffset();
        }
        newSignal = new Signal(odrID::invalidID(), "signal", s, t, false, Signal::NEGATIVE_TRACK_DIRECTION, zOffset, signalManager_->getCountry(lastSignal), lastSignal->getSignalType(), lastSignal->getSignalTypeSubclass(), lastSignal->getSignalSubType(), lastSignal->getSignalValue(), 0.0, 0.0, 0.0, lastSignal->getSignalUnit(), lastSignal->getSignalText(), lastSignal->getSignalWidth(), lastSignal->getSignalHeight(), true, 2, validFromLane, validToLane);
        AddSignalCommand *command = new AddSignalCommand(newSignal, road, NULL);
        getProjectGraph()->executeCommand(command);
    }
    else
    {
        newSignal = new Signal(odrID::invalidID(), "signal", s, t, false, Signal::NEGATIVE_TRACK_DIRECTION, zOffset, "Germany", "-1", "", "-1", 0.0, 0.0, 0.0, 0.0, "hm/h", "", 0.0, 0.0, true, 2, validFromLane, validToLane);
        AddSignalCommand *command = new AddSignalCommand(newSignal, road, NULL);
        getProjectGraph()->executeCommand(command);
    }

    return newSignal;
}

void
SignalEditor::addShieldToRoad(Signal *signal)
{
    double s = signal->getSStart();
    if (!shieldScenes_.contains(s))
    {
        SignalSectionPolynomialItems *signalSectionPolynomialItems = new SignalSectionPolynomialItems(this, signalManager_, signal->getParentRoad(), s);
        shieldScenes_.insert(s, signalSectionPolynomialItems);
        profileGraph_->getScene()->addItem(signalSectionPolynomialItems);
    }

    QRectF boundingBox = QRectF(-10.0, -5.0, 20.0, 10.0);
    profileGraph_->getView()->fitInView(boundingBox, Qt::KeepAspectRatio);
    profileGraph_->getView()->zoomOut(Qt::Horizontal | Qt::Vertical);

}

int
SignalEditor::delShieldFromRoad(Signal *signal)
{
    double s = signal->getSStart();
    SignalSectionPolynomialItems *signalSectionPolynomialItems = shieldScenes_.take(s);

    if (!signalSectionPolynomialItems)
    {
        return 0;
    }
    else
    {
        signalSectionPolynomialItems->deselectSignalPoles(signal);
        profileGraph_->getScene()->removeItem(signalSectionPolynomialItems);
        signalSectionPolynomialItems->registerForDeletion();
        signalSectionPolynomialItems->deleteLater();

        return 1;
    }
}


void
SignalEditor::translateObject(ObjectItem *objectItem, QPointF &diff)
{
    Object *object = objectItem->getObject();
    RSystemElementRoad *road = object->getParentRoad();
    double s;
    QVector2D vec;
    double dist;
    QPointF to = road->getGlobalPoint(object->getSStart(), object->getT()) + diff;
    RSystemElementRoad *newRoad = getProjectData()->getRoadSystem()->findClosestRoad(to, s, dist, vec);

    if (newRoad != road)
    {
        RemoveObjectCommand *removeObjectCommand = new RemoveObjectCommand(object, road);
        getProjectGraph()->executeCommand(removeObjectCommand);

        AddObjectCommand *addObjectCommand = new AddObjectCommand(object, newRoad);
        getProjectGraph()->executeCommand(addObjectCommand);
        object->setElementSelected(false);

        road = newRoad;
    }

    Object::ObjectProperties objectProps = object->getProperties();
    objectProps.t = dist;
    Object::ObjectRepeatRecord repeatProps = object->getRepeatProperties();
    repeatProps.s = s;
    SetObjectPropertiesCommand *objectPropertiesCommand = new SetObjectPropertiesCommand(object, object->getId(), object->getName(), objectProps, repeatProps, object->getTextureFileName());
    getProjectGraph()->executeCommand(objectPropertiesCommand);
    MoveRoadSectionCommand *moveSectionCommand = new MoveRoadSectionCommand(object, s, RSystemElementRoad::DRS_ObjectSection);
    getProjectGraph()->executeCommand(moveSectionCommand);
}

Object *
SignalEditor::addObjectToRoad(RSystemElementRoad *road, double s, double t)
{
    ObjectContainer *lastObject = signalManager_->getSelectedObjectContainer();
    Object *newObject = NULL;

    if (lastObject)
    {
        if (t < 0)
        {
            t -= lastObject->getObjectDistance();
        }
        else
        {
            t += lastObject->getObjectDistance();
        }

        Object::ObjectProperties objectProps{ t, Signal::NEGATIVE_TRACK_DIRECTION, 0.0, lastObject->getObjectType(), 0.0, lastObject->getObjectLength(), lastObject->getObjectWidth(),
            lastObject->getObjectRadius(), lastObject->getObjectHeight(), lastObject->getObjectHeading(),
            0.0, 0.0, false };

        Object::ObjectRepeatRecord repeatProps{ s, 0.0, lastObject->getObjectRepeatDistance(), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };  // TODO: add properties to container
        newObject = new Object(odrID::invalidID(), "object", s, objectProps, repeatProps, lastObject->getObjectFile());
        /*  newObject = new Object("object", "", lastObject->getObjectType(), s, t, 0.0, 0.0, Object::NEGATIVE_TRACK_DIRECTION, lastObject->getObjectLength(),
                    lastObject->getObjectWidth(), lastObject->getObjectRadius(), lastObject->getObjectHeight(), lastObject->getObjectHeading(),
                            0.0, 0.0, false, s, 0.0, lastObject->getObjectRepeatDistance(), lastObject->getObjectFile()); */
        AddObjectCommand *command = new AddObjectCommand(newObject, road, NULL);
        getProjectGraph()->executeCommand(command);
    }
    else
    {
        Object::ObjectProperties objectProps{ t, Signal::NEGATIVE_TRACK_DIRECTION, 0.0, "", 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, false };
        Object::ObjectRepeatRecord repeatProps{ s, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        Object *newObject = new Object(odrID::invalidID(), "object", s, objectProps, repeatProps, "");
        AddObjectCommand *command = new AddObjectCommand(newObject, road, NULL);
        getProjectGraph()->executeCommand(command);
    }

    return newObject;
}

void
SignalEditor::translateBridge(BridgeItem *bridgeItem, QPointF &diff)
{
    Bridge *bridge = bridgeItem->getBridge();
    RSystemElementRoad *road = bridge->getParentRoad();

    double s;
    QVector2D vec;
    double dist;
    QPointF to = road->getGlobalPoint(bridge->getSStart()) + diff;
    RSystemElementRoad *newRoad = getProjectData()->getRoadSystem()->findClosestRoad(to, s, dist, vec);


    if (newRoad != road)
    {
        RemoveBridgeCommand *removeBridgeCommand = new RemoveBridgeCommand(bridge, road);
        getProjectGraph()->executeCommand(removeBridgeCommand);

        AddBridgeCommand *addBridgeCommand = new AddBridgeCommand(bridge, newRoad);
        getProjectGraph()->executeCommand(addBridgeCommand);
        bridge->setElementSelected(false);

        road = newRoad;
    }

    SetBridgePropertiesCommand *bridgePropertiesCommand = new SetBridgePropertiesCommand(bridge, bridge->getId(), bridge->getFileName(), bridge->getName(), bridge->getType(), bridge->getLength());
    getProjectGraph()->executeCommand(bridgePropertiesCommand);
    MoveRoadSectionCommand *moveSectionCommand = new MoveRoadSectionCommand(bridge, s, RSystemElementRoad::DRS_BridgeSection);
    getProjectGraph()->executeCommand(moveSectionCommand);
}


//################//
// MOUSE & KEY    //
//################//

/*! \brief .
*
*/
void
SignalEditor::mouseAction(MouseAction *mouseAction)
{
    ProjectEditor::mouseAction(mouseAction);

    // SELECT //
    //
    ODD::ToolId currentToolId = getCurrentTool();
    if (mouseAction->getMouseActionType() == MouseAction::ATM_DROP)
    {
        if ((currentToolId == ODD::TSG_SIGNAL) || (currentToolId == ODD::TSG_OBJECT) || (currentToolId == ODD::TSG_BRIDGE) || (currentToolId == ODD::TSG_TUNNEL))
        {
            QGraphicsSceneDragDropEvent *mouseEvent = mouseAction->getDragDropEvent();
            QPointF mousePoint = mouseEvent->scenePos();

            QList<QGraphicsItem *> underMouseItems = topviewGraph_->getScene()->items(mousePoint);

            RSystemElementRoad *road = NULL;
            double s, t;
            for (int i = 0; i < underMouseItems.size(); i++)
            {
                SignalRoadItem *roadItem = dynamic_cast<SignalRoadItem *>(underMouseItems.at(i));
                if (roadItem)
                {
                    road = roadItem->getRoad();
                    s = road->getSFromGlobalPoint(mousePoint);
                    t = road->getTFromGlobalPoint(mousePoint, s);

                    break;
                }
            }

            if (!road)  // find the closest road //
            {
                QVector2D vec;
                road = getProjectData()->getRoadSystem()->findClosestRoad(mousePoint, s, t, vec);
            }

            if (road)
            {
                switch (currentToolId)
                {
                case ODD::TSG_SIGNAL:
                    addSignalToRoad(road, s, t);
                    break;
                case ODD::TSG_OBJECT:
                    addObjectToRoad(road, s, t);
                    break;
                case ODD::TSG_BRIDGE: {
                    // Add new bridge //
                    //
                    Bridge *newBridge = new Bridge(getProjectData()->getRoadSystem()->getID("bridge", odrID::ID_Bridge), "", "", Bridge::BT_CONCRETE, s, 100.0);
                    AddBridgeCommand *command = new AddBridgeCommand(newBridge, road, NULL);

                    getProjectGraph()->executeCommand(command);
                    break; }
                case ODD::TSG_TUNNEL: {
                    // Add new tunnel //
                    //
                    Tunnel *newTunnel = new Tunnel(getProjectData()->getRoadSystem()->getID("tunnel", odrID::ID_Bridge), "", "", Tunnel::TT_STANDARD, s, 100.0, 0.0, 0.0);
                    AddBridgeCommand *command = new AddBridgeCommand(newTunnel, road, NULL);

                    getProjectGraph()->executeCommand(command);
                    break; }
                default:
                    break;
                }
            }
        }
    }
    else if (mouseAction->getMouseActionType() == MouseAction::PATM_DROP)
    {
        if (currentToolId == ODD::TSG_SIGNAL)
        {
            if (!shieldScenes_.isEmpty())
            {
                SignalSectionPolynomialItems *signalSectionPolynomialItems = shieldScenes_.first();
                RSystemElementRoad *road = signalSectionPolynomialItems->getRoad();
                double s = signalSectionPolynomialItems->getS();

                if (road)
                {
                    QGraphicsSceneDragDropEvent *mouseEvent = mouseAction->getDragDropEvent();
                    QPointF mousePoint = mouseEvent->scenePos();
                    double t = mousePoint.x();
                    double zOffset = mousePoint.y();

                    if (signalManager_->getSelectedSignalContainer())
                    {
                        Signal *signal = addSignalToRoad(road, s, signalSectionPolynomialItems->getClosestT(t), true, zOffset);

                        mouseAction->intercept();
                        return;
                    }
                }
            }
        } 
    }

 
    ProjectEditor::mouseAction(mouseAction);
}


//################//
// TOOL           //
//################//

/*! \brief .
*
*/
void
SignalEditor::toolAction(ToolAction *toolAction)
{
    if (tool_ && !tool_->containsToolId(toolAction->getToolId()))
    {
        resetTool();
    }

    // Parent //
    //
    ProjectEditor::toolAction(toolAction);

    ODD::ToolId currentTool = getCurrentTool();

    SignalEditorToolAction *signalEditorToolAction = dynamic_cast<SignalEditorToolAction *>(toolAction);
    if (signalEditorToolAction)
    {
        // Create Controller //
            //
        if (signalEditorToolAction->getToolId() == ODD::TSG_CONTROLLER)
        {
            signalRoadSystemItem_->setControllersSelectable(false);
            ODD::ToolId paramTool = getCurrentParameterTool();

            if ((paramTool == ODD::TNO_TOOL) && !tool_)
            {
                tool_ = new Tool(ODD::TSG_CONTROLLER, 4);
                assignParameterSelection(ODD::TSG_CONTROLLER);
                ToolValue<Signal> *param = new ToolValue<Signal>(ODD::TSG_CONTROLLER, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT_LIST, "Select/Remove", true);
                tool_->readParams(param);

                createToolParameterSettingsApplyBox(tool_, ODD::ESG);
                ODD::mainWindow()->showParameterDialog(true, "Create controller from signals", "SELECT/DESELECT signals and press APPLY to create Controller");

                applyCount_ = 1;

                // verify if apply can be displayed //

                int objectCount = tool_->getObjectCount(currentTool, ODD::TPARAM_SELECT);
                if (objectCount >= applyCount_)
                {
                    settingsApplyBox_->setApplyButtonVisible(true);
                }
            }
        }

        else if (signalEditorToolAction->getToolId() == ODD::TSG_REMOVE_CONTROL_ENTRY)
        {
            ODD::ToolId paramTool = getCurrentParameterTool();

            if ((paramTool == ODD::TNO_TOOL) && !tool_)
            {
                ToolValue<RSystemElementController> *param;
                if (controller_)
                {
                    if (!controller_->isElementSelected())
                    {
                        SelectDataElementCommand *command = new SelectDataElementCommand(controller_, NULL);
                        getProjectGraph()->executeCommand(command);
                    }
                    param = new ToolValue<RSystemElementController>(ODD::TSG_SELECT_CONTROLLER, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT, "Select Controller", true, "", controller_->getIdName(), controller_);
                }
                else
                {
                    param = new ToolValue<RSystemElementController>(ODD::TSG_SELECT_CONTROLLER, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT, "Select Controller", true);
                }
                tool_ = new Tool(ODD::TSG_REMOVE_CONTROL_ENTRY, 4);
                tool_->readParams(param);
                assignParameterSelection(ODD::TSG_REMOVE_CONTROL_ENTRY);
                ToolValue<Signal> *signalParam;
                if (controller_)
                {
                    signalParam = new ToolValue<Signal>(ODD::TSG_REMOVE_CONTROL_ENTRY, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT_LIST, "Select/Remove", true);
                }
                else
                {
                    signalParam = new ToolValue<Signal>(ODD::TSG_REMOVE_CONTROL_ENTRY, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT_LIST, "Select/Remove", false);
                }
                tool_->readParams(signalParam);

                createToolParameterSettingsApplyBox(tool_, ODD::ESG);
                ODD::mainWindow()->showParameterDialog(true, "Remove signals from controller", "SELECT a controller, SELECT/DESELECT signals and press APPLY");

                applyCount_ = 1;

                // verify if apply can be displayed //

                int objectCount = tool_->getObjectCount(currentTool, ODD::TPARAM_SELECT);
                if ((objectCount >= applyCount_) && controller_)
                {
                    settingsApplyBox_->setApplyButtonVisible(true);
                }
            }
        }
        else if (signalEditorToolAction->getToolId() == ODD::TSG_ADD_CONTROL_ENTRY)
        {
            ODD::ToolId paramTool = getCurrentParameterTool();

            if ((paramTool == ODD::TNO_TOOL) && !tool_)
            {
                ToolValue<RSystemElementController> *param;
                if (controller_)
                {
                    if (!controller_->isElementSelected())
                    {
                        SelectDataElementCommand *command = new SelectDataElementCommand(controller_, NULL);
                        getProjectGraph()->executeCommand(command);
                    }
                    param = new ToolValue<RSystemElementController>(ODD::TSG_SELECT_CONTROLLER, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT, "Select Controller", true, "", controller_->getIdName(), controller_);
                }
                else
                {
                    param = new ToolValue<RSystemElementController>(ODD::TSG_SELECT_CONTROLLER, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT, "Select Controller", true);
                }
                tool_ = new Tool(ODD::TSG_ADD_CONTROL_ENTRY, 4);
                tool_->readParams(param);
                assignParameterSelection(ODD::TSG_ADD_CONTROL_ENTRY);
                ToolValue<Signal> *signalParam;
                if (controller_)
                {
                    signalParam = new ToolValue<Signal>(ODD::TSG_ADD_CONTROL_ENTRY, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT_LIST, "Select/Remove", true);
                }
                else
                {
                    signalParam = new ToolValue<Signal>(ODD::TSG_ADD_CONTROL_ENTRY, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT_LIST, "Select/Remove", false);
                }
                tool_->readParams(signalParam);

                createToolParameterSettingsApplyBox(tool_, ODD::ESG);
                ODD::mainWindow()->showParameterDialog(true, "Add signals to controller", "SELECT a controller, SELECT/DESELECT signals and press APPLY");

                applyCount_ = 1;

                // verify if apply can be displayed //

                int objectCount = tool_->getObjectCount(currentTool, ODD::TPARAM_SELECT);
                if ((objectCount >= applyCount_) && controller_)
                {
                    settingsApplyBox_->setApplyButtonVisible(true);
                }
            }
        }
    }
    else
    {
        ParameterToolAction *action = dynamic_cast<ParameterToolAction *>(toolAction);
        if (action)
        {
            if ((action->getToolId() == ODD::TSG_CONTROLLER) || (action->getToolId() == ODD::TSG_ADD_CONTROL_ENTRY) || (action->getToolId() == ODD::TSG_REMOVE_CONTROL_ENTRY))
            {
                if (action->getParamToolId() == ODD::TPARAM_SELECT)
                {
                    if (!action->getState())
                    {

                        QList<Signal *> signalList = tool_->getValues<Signal>(action->getParamId());
                        foreach(Signal *signal, signalList)
                        {
                            DeselectDataElementCommand *command = new DeselectDataElementCommand(signal, NULL);
                            getProjectGraph()->executeCommand(command);
                        }
                    }
                }
            }
        }
    }

}

void
SignalEditor::assignParameterSelection(ODD::ToolId toolId)
{
    if ((toolId == ODD::TSG_CONTROLLER) || (toolId == ODD::TSG_ADD_CONTROL_ENTRY) || (toolId == ODD::TSG_REMOVE_CONTROL_ENTRY))
    {
        QList<QGraphicsItem *> selectedItems = topviewGraph_->getScene()->selectedItems();

        for (int i = 0; i < selectedItems.size(); i++)
        {
            QGraphicsItem *item = selectedItems.at(i);
            SignalItem *signalItem = dynamic_cast<SignalItem *>(item);
            if (signalItem)
            {
                Signal *signal = signalItem->getSignal();
                if (!selectedSignals_.contains(signal))
                {
                    ToolValue<Signal> *signalParam = new ToolValue<Signal>(toolId, ODD::TPARAM_SELECT, 1, ToolParameter::ParameterTypes::OBJECT_LIST, "Select/Remove", true, "", signal->getIdName(), signal);
                    tool_->readParams(signalParam);
                    selectedSignals_.append(signal);
                }
            }
            else if (toolId == ODD::TSG_CONTROLLER)
            {
                item->setSelected(false);
            }
            else
            {
                ControllerItem *controllerItem = dynamic_cast<ControllerItem *>(item);
                if (!controllerItem || (controllerItem->getController() != controller_))
                {
                    item->setSelected(false);
                }
            }
        }

    }
}

void 
SignalEditor::registerSignal(Signal *signal)
{
    createToolParameters<Signal>(signal);
    selectedSignals_.append(signal);

    int objectCount = tool_->getObjectCount(getCurrentTool(), getCurrentParameterTool());
    if (objectCount >= applyCount_)
    {
        settingsApplyBox_->setApplyButtonVisible(true);
    }
}

void 
SignalEditor::deregisterSignal(Signal *signal)
{
    removeToolParameters<Signal>(signal);
    selectedSignals_.removeOne(signal);

    int objectCount = tool_->getObjectCount(getCurrentTool(), getCurrentParameterTool());
    if (objectCount < applyCount_)
    {
        settingsApplyBox_->setApplyButtonVisible(false);
    }
}

bool 
SignalEditor::registerController(RSystemElementController *controller)
{
    if (controller != controller_)
    {
        DeselectDataElementCommand *command = new DeselectDataElementCommand(controller_);
        getProjectGraph()->executeCommand(command);
       
        controller_ = controller;

        if (tool_)
        {
            setToolValue<RSystemElementController>(controller_, controller_->getIdName());

            int objectCount = tool_->getObjectCount(getCurrentTool(), getCurrentParameterTool());
            if (objectCount >= applyCount_)
            {
                settingsApplyBox_->setApplyButtonVisible(true);
            }
        }

        return true;
    }
    return false;
}

//################//
// SLOTS          //
//################//

void
SignalEditor::apply()
{
    ODD::ToolId toolId = tool_->getToolId();
    if (toolId == ODD::TSG_CONTROLLER)
    {
        getProjectGraph()->beginMacro("Add Controller");

        QList<ControlEntry *>controlEntryList;
        RSystemElementController *newController = new RSystemElementController("controller", getProjectData()->getRoadSystem()->getID(odrID::ID_Controller), 0, "", 0.0, controlEntryList);
        AddControllerCommand *command = new AddControllerCommand(newController, getProjectData()->getRoadSystem(), NULL);

        getProjectGraph()->executeCommand(command);

        foreach(Signal * signal, selectedSignals_)
        {
            ControlEntry *controlEntry = new ControlEntry(signal->getId(), signal->getType());
            AddControlEntryCommand *addControlEntryCommand = new AddControlEntryCommand(newController, controlEntry, signal);
            getProjectGraph()->executeCommand(addControlEntryCommand);
        }

        getProjectGraph()->endMacro();
    }
    else if (toolId == ODD::TSG_ADD_CONTROL_ENTRY)
    {
        getProjectGraph()->beginMacro("Add ControlEntry");

        foreach(Signal * signal, selectedSignals_)
        {
            ControlEntry *controlEntry = new ControlEntry(signal->getId(), signal->getType());
            AddControlEntryCommand *addControlEntryCommand = new AddControlEntryCommand(controller_, controlEntry, signal);
            getProjectGraph()->executeCommand(addControlEntryCommand);
        }

        getProjectGraph()->endMacro();
    }
    else if (toolId == ODD::TSG_REMOVE_CONTROL_ENTRY)
    {
        getProjectGraph()->beginMacro("Remove ControlEntry");

        foreach(Signal * signal, selectedSignals_)
        {
            foreach(ControlEntry * controlEntry, controller_->getControlEntries())
            {
                if (controlEntry->getSignalId() == signal->getId())
                {
                    DelControlEntryCommand *delControlEntryCommand = new DelControlEntryCommand(controller_, controlEntry, signal);
                    getProjectGraph()->executeCommand(delControlEntryCommand);
                    break;
                }
            }
        }

        getProjectGraph()->endMacro();
    }

}


void
SignalEditor::clearToolObjectSelection()
{
    // deselect all //

    foreach(Signal * signal, selectedSignals_)
    {
        DeselectDataElementCommand *command = new DeselectDataElementCommand(signal, NULL);
        getProjectGraph()->executeCommand(command);
    }

    selectedSignals_.clear();
}

void
SignalEditor::reset()
{
    clearToolObjectSelection();
    delToolParameters();
}

void
SignalEditor::resetTool()
{
    if (tool_)
    {
        ODD::ToolId toolId = tool_->getToolId();
        if (toolId == ODD::TSG_CONTROLLER) 
        {
            signalRoadSystemItem_->setControllersSelectable(true);
        }
        clearToolObjectSelection();
        delToolParameters();
        ODD::mainWindow()->showParameterDialog(false);
    }
}

void SignalEditor::reject()
{
    ProjectEditor::reject();

    resetTool();
}
