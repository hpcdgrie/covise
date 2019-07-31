/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

#define XK_MISCELLANY
#if !defined(_WIN32) && !defined(__APPLE__)
#include <X11/keysymdef.h>
#endif
#ifndef _WIN32
#include <sys/time.h>
#include <time.h>
#endif
#include "ViewPoint.h"
#include "ViewDesc.h"
#include "QuickNavDrawable.h"
#include "coVRDOMDocument.h"
#include <QDomDocument>
#include <QFile>
#include <QString>
#include <QTextStream>
#include <cover/coVRMSController.h>
#include <cover/coVRFileManager.h>
#include <config/CoviseConfig.h>
#include <cover/ui/Menu.h>
#include <cover/ui/Action.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>
#include <net/message_types.h>
#include <net/message.h>
#include <util/unixcompat.h>
#include <grmsg/coGRMsg.h>
#include <grmsg/coGRCreateViewpointMsg.h>
#include <grmsg/coGRCreateDefaultViewpointMsg.h>
#include <grmsg/coGRDeleteViewpointMsg.h>
#include <grmsg/coGRShowViewpointMsg.h>
#include <grmsg/coGRKeyWordMsg.h>
#include <grmsg/coGRToggleFlymodeMsg.h>
#include <grmsg/coGRChangeViewpointMsg.h>
#include <grmsg/coGRChangeViewpointIdMsg.h>
#include <grmsg/coGRChangeViewpointNameMsg.h>
#include <grmsg/coGRViewpointChangedMsg.h>
#include <grmsg/coGRActivatedViewpointMsg.h>
#include <grmsg/coGRToggleVPClipPlaneModeMsg.h>
#include <grmsg/coGRTurnTableAnimationMsg.h>
#include <cover/OpenCOVER.h>
#include <cover/VRSceneGraph.h>
#include <cover/coVRConfig.h>
#include <osg/ClipNode>
#include <cover/coVRPluginList.h>

using namespace osg;
using namespace opencover;
using namespace grmsg;
using namespace covise;

bool ViewPoints::showViewpoints = false;
bool ViewPoints::showFlightpath = false;
bool ViewPoints::showCamera = false;
bool ViewPoints::showInteractors = false;
bool ViewPoints::shiftFlightpathToEyePoint = false;
Vec3 ViewPoints::tangentOut;
Vec3 ViewPoints::tangentIn;

ViewDesc *ViewPoints::lastVP = NULL;

Interpolator::EasingFunction ViewPoints::curr_easingFunc = Interpolator::LINEAR_EASING;
Interpolator::TranslationMode ViewPoints::curr_translationMode = Interpolator::BEZIER;
Interpolator::RotationMode ViewPoints::curr_rotationMode = Interpolator::QUATERNION;

SharedActiveVPData *ViewPoints::actSharedVPData = 0;

ViewPoints::ViewPoints()
: ui::Owner("ViewPoints", cover->ui)
{
}


ViewPoints *ViewPoints::inst=NULL;

bool ViewPoints::init()
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::init\n");
    inst = this;
    //init VPData for clipplane
    actSharedVPData = new SharedActiveVPData();
    actSharedVPData->totNum = 0;

    for (int i = 0; i < 6; i++)
        actSharedVPData->hasClipPlane[i] = 0;

    actSharedVPData->index = -1;
    actSharedVPData->isEnabled = 0;

    flyingStatus = false;
    fileNumber = 0;
    record = false;
    videoBeingCaptured = false;
    id_ = 0;
    initTime = 0;
    flight_index = 0;
    frameNumber = 0;
    numberOfDefaultVP = 0;

    turnTableAnimation_ = false;
    turnTableStep_ = false;

    turnTableStepAlpha_ = -45.0;

    turnTableViewingAngle_ = coCoviseConfig::getFloat("COVER.Plugin.ViewPoint.TurntableViewingAngle", 45.0f);

    eyepoint[0] = coCoviseConfig::getFloat("x", "COVER.ViewerPosition", 0.0f);
    eyepoint[1] = coCoviseConfig::getFloat("y", "COVER.ViewerPosition", -1500.0f);
    eyepoint[2] = coCoviseConfig::getFloat("z", "COVER.ViewerPosition", 0.0f);

    flyingMode = coCoviseConfig::isOn("COVER.Plugin.ViewPoint.FlyingMode", true);

    loopMode = coCoviseConfig::isOn("COVER.Plugin.ViewPoint.Loop", loopMode);

    // send flyingMode to Gui
    sendFlyingModeToGui();

    // default value of 3.0 is static
    flightTime = coCoviseConfig::getFloat("COVER.Plugin.ViewPoint.FlightTime", 10.0);

    // read the viewpoints from covise.config
    bool blockConfigInFlight = !coCoviseConfig::isOn("COVER.Plugin.ViewPoint.FlyConfig", false);
    isQuickNavEnabled = coCoviseConfig::isOn("COVER.Plugin.ViewPoint.QuickNav", false);

    curr_scale = 1.0;
    vwpPath = coVRConfig::instance()->viewpointsFile;
    if(vwpPath == "")
    {
        vwpPath = coVRFileManager::instance()->getViewPointFile();
    }

    vp_index = 0;

    // create the menu
    viewPointMenu_ = new ui::Menu("ViewPoints", this);
    viewPointMenu_->setText("Viewpoints");

    saveButton_ = new ui::Action(viewPointMenu_, "CreateViewpoint");
    saveButton_->setText("Create viewpoint");
    saveButton_->setCallback([this](){
        saveViewPoint();
    });

    useClipPlanesCheck_ = new ui::Button(viewPointMenu_, "SetClipPlanes");
    useClipPlanesCheck_->setText("Set clip planes");
    useClipPlanesCheck_->setState(false);
    useClipPlanesCheck_->setCallback([this](bool state){
        //sendClipplaneModeToGui();
    });

    flyingModeCheck_ = new ui::Button(viewPointMenu_, "FlyingMode");
    flyingModeCheck_->setText("Flying mode");
    flyingModeCheck_->setState(flyingMode);
    flyingModeCheck_->setCallback([this](bool state){
        flyingMode = state;
        // send to gui
        sendFlyingModeToGui();
        if (!flyingMode)
        {
            showCameraCheck_->setState(false);
            showCamera = false;
        }
    });

    startStopRecButton_ = new ui::Action(viewPointMenu_, "StartRecord");
    startStopRecButton_->setText("Start record");
    startStopRecButton_->setCallback([this](){
        startStopRec();
    });

    turnTableStepButton_ = new ui::Action(viewPointMenu_, "TurntableStep");
    turnTableStepButton_->setText("Turntable step");
    turnTableStepButton_->setCallback([this](){
        turnTableStep();
    });

    turnTableAnimationCheck_ = new ui::Button(viewPointMenu_, "StartTurntableAnimation");
    turnTableAnimationCheck_->setText("Start turntable animation");
    turnTableAnimationCheck_->setState(false);
    turnTableAnimationCheck_->setCallback([this](bool state){
        startTurnTableAnimation(20.0);
    });

    // create the "flight" menu
    flightMenu_ = new ui::Menu(viewPointMenu_, "FlightConfig");
    flightMenu_->setText("Flight configuration");


    runMenu_ = new ui::Menu(viewPointMenu_, "run");
    runMenu_->setText("Run flight");


    runButton = new ui::Button(runMenu_, "RunFlight");
    runButton->setText("Run flight");
    runButton->setCallback([this](bool state){
        completeFlight(state);
    });
    
    pauseButton = new ui::Button(runMenu_, "Pause");
    pauseButton->setText("Pause flight");
    pauseButton->setState(false);
    pauseButton->setCallback([this](bool state){
        pauseFlight(state);
    });
    
    speedSlider = new ui::Slider(runMenu_, "animSpeed");
    speedSlider->setBounds(0., 30.);
    speedSlider->setValue(flightTime);
    speedSlider->setIntegral(false);
    speedSlider->setText("flight speed");
    speedSlider->setPresentation(ui::Slider::AsDial);
    speedSlider->setCallback([this](double value, bool released){
        flightTime = value;
    });
    animPositionSlider = new ui::Slider(runMenu_, "animPosition");
    animPositionSlider->setBounds(0., 100.);
    animPositionSlider->setValue(0.0);
    animPositionSlider->setIntegral(false);
    animPositionSlider->setText("flight time");
    //animPositionSlider->setPresentation(ui::Slider::AsSilder);
    animPositionSlider->setCallback([this](double value, bool released){
        flightTime = value;
	float delta = 100.0/(viewpoints.size()-1);
	flight_index = value/delta;
	float lambda = (value - flight_index * delta)/delta * flightTime;
	initTime = -((lambda * flightTime) - cover->frameTime());
    });
    

    // Edit VP menu

    editVPMenu_ = new ui::Menu(viewPointMenu_, "EditViewpoints");
    editVPMenu_->setText("Edit viewpoints");

    showFlightpathCheck_ = new ui::Button(editVPMenu_, "VisualizeFlightpath");
    showFlightpathCheck_->setText("Visualize flight path");
    showFlightpathCheck_->setState(showFlightpath);
    showFlightpathCheck_->setCallback([this](bool state){
        showFlightpath = state;
        flightPathVisualizer->showFlightpath(showFlightpath);
    });

    shiftFlightpathCheck_ = new ui::Button(editVPMenu_, "ShiftFlightpath");
    shiftFlightpathCheck_->setText("Shift flightpath");
    shiftFlightpathCheck_->setState(shiftFlightpathToEyePoint);
    shiftFlightpathCheck_->setCallback([this](bool state){
        shiftFlightpathToEyePoint = state;

        ViewDesc *currentPoint;
        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            currentPoint = (*it);
            currentPoint->shiftFlightpath(shiftFlightpathToEyePoint);
        }

        flightPathVisualizer->shiftFlightpath(shiftFlightpathToEyePoint);
        flightPathVisualizer->updateDrawnCurve();
        flightPathVisualizer->showFlightpath(showFlightpath);
    });

    showCameraCheck_ = new ui::Button(editVPMenu_, "VisualizeCamera");
    showCameraCheck_->setText("Visualize camera");
    showCameraCheck_->setState(showCamera);
    showCameraCheck_->setCallback([this](bool state){
        showCamera = state;
        if ((flightPathVisualizer->getCameraDCS() == NULL) && (ssize_t(viewpoints.size()) > numberOfDefaultVP))
        {
            if (showCamera)
                flyingMode = 1;
            flyingModeCheck_->setState(showCamera);

            if (lastVP != NULL)
                flightPathVisualizer->createCameraGeometry(lastVP);
            else
                flightPathVisualizer->createCameraGeometry(viewpoints[numberOfDefaultVP]);
        }
        else
        {
            if (flightPathVisualizer->getCameraDCS() != NULL)
                flightPathVisualizer->deleteCameraGeometry();
        }
    });

    showViewpointsCheck_ = new ui::Button(editVPMenu_, "ShowHideViewpoints");
    showViewpointsCheck_->setText("Show/hide all viewpoints");
    showViewpointsCheck_->setState(showViewpoints);
    showViewpointsCheck_->setCallback([this](bool state){
        showViewpoints = state;
        ViewDesc *currentPoint;
        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            currentPoint = (*it);
            if (showViewpoints == false)
            {
                currentPoint->showMoveInteractors(false);
                currentPoint->showTangentInteractors(false);
                showInteractorsCheck_->setState(false);
            }
            currentPoint->showGeometry(showViewpoints);
            currentPoint->showTangent(showViewpoints);
        }
    });

    showInteractorsCheck_ = new ui::Button(editVPMenu_, "ShowHideInteractors");
    showInteractorsCheck_->setText("Show/hide all interactors");
    showInteractorsCheck_->setState(showInteractors);
    showInteractorsCheck_->setCallback([this](bool state){
        showInteractors = state;
        ViewDesc *currentPoint;
        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            currentPoint = (*it);
            if (showInteractors)
            {
                currentPoint->showGeometry(true);
                currentPoint->showTangent(true);
                showViewpointsCheck_->setState(true);
            }
            currentPoint->showMoveInteractors(showInteractors);
            currentPoint->showTangentInteractors(showInteractors);
        }
    });

    vpSaveButton_ = new ui::Action(editVPMenu_, "SaveViewpoints");
    vpSaveButton_->setText("Save viewpoints");
    vpSaveButton_->setCallback([this](){
        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            ViewDesc *curr = (*it);
            if (curr != NULL)
            {
                if (curr->hasMatrix())
                {
                    changeViewDesc(curr->getMatrix(), curr->getScale(), curr->getTangentIn(), curr->getTangentOut(), curr->getId(), curr->getName(), curr);
                    if (cover->debugLevel(3))
                        fprintf(stderr, "\n Viewpoint [%i] changed and saved\n", curr->getId());
                }
            }
        }
    });

    vpReloadButton_ = new ui::Action(editVPMenu_, "ReloadViewpoints");
    vpReloadButton_->setText("Reload viewpoints");
    vpReloadButton_->setCallback([this](){
    });

    // Align Menu
    //========================================================================================
    alignMenu_ = new ui::Menu(editVPMenu_, "AlignViewpoints");
    alignMenu_->setText("Align viewpoints");

    setCatmullRomButton_ = new ui::Action(alignMenu_, "SetCatmullRomTangents");
    setCatmullRomButton_->setText("Set Catmull-Rom tangents");
    setCatmullRomButton_->setCallback([this](){
        setCatmullRomTangents();
    });

    setStraightButton_ = new ui::Action(alignMenu_, "SetTangentsToViewpoint");
    setStraightButton_->setText("Set tangents to direction of viewpoint");
    setStraightButton_->setCallback([this](){
        setStraightTangents();
    });

    setEqualTangentsButton_ = new ui::Action(alignMenu_, "SetEqualTangents");
    setEqualTangentsButton_->setText("Set equal tangents (C1)");
    setEqualTangentsButton_->setCallback([this](){
        setEqualTangents();
    });

    alignXButton_ = new ui::Action(alignMenu_, "AlignX");
    alignXButton_->setText("Align X");
    alignXButton_->setCallback([this](){
        alignViewpoints('x');
    });
    alignYButton_ = new ui::Action(alignMenu_, "AlignY");
    alignYButton_->setText("Align Y");
    alignYButton_->setCallback([this](){
        alignViewpoints('y');
    });
    alignZButton_ = new ui::Action(alignMenu_, "AlignZ");
    alignZButton_->setText("Align Z");
    alignZButton_->setCallback([this](){
        alignViewpoints('z');
    });

    // the slave and the master have to read the default viewpoints from config

    coCoviseConfig::ScopeEntries viewpointEntries = coCoviseConfig::getScopeEntries("COVER.Plugin.ViewPoint", "Viewpoints");
    const char **it = viewpointEntries.getValue();
    while (it && *it)
    {
        string n = *it;
        it++;
        const char *vp = *it;
        it++;
        string vpname = n.substr(11);
        const char *name = vpname.c_str();

        if (name && vp)
        {
            ViewDesc *newVP;
            if (strncasecmp(name, "left", 4) == 0)
            {
                name = "left";
                if (isOn(vp))
                {
                    Vec3 hpr(90, 0, 0);
                    newVP = new ViewDesc(name, id_, hpr, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "right", 5) == 0)
            {
                name = "right";
                if (isOn(vp))
                {
                    Vec3 hpr(-90, 0, 0);
                    newVP = new ViewDesc(name, id_, hpr, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "front", 5) == 0)
            {
                name = "front";
                if (isOn(vp))
                {
                    Vec3 hpr(0, 0, 0);
                    newVP = new ViewDesc(name, id_, hpr, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "back", 4) == 0)
            {
                name = "back";
                if (isOn(vp))
                {
                    Vec3 hpr(180, 0, 0);
                    newVP = new ViewDesc(name, id_, hpr, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "top", 3) == 0)
            {
                name = "top";
                if (isOn(vp))
                {
                    Vec3 hpr(0, 90, 0);
                    newVP = new ViewDesc(name, id_, hpr, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "bottom", 6) == 0)
            {
                name = "bottom";
                if (isOn(vp))
                {
                    Vec3 hpr(0, -90, 0);
                    newVP = new ViewDesc(name, id_, hpr, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "halftop", 7) == 0)
            {
                name = "halftop";
                if (isOn(vp))
                {
                    Vec3 hpr(45, 20, -20);
                    newVP = new ViewDesc(name, id_, hpr, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "center", 6) == 0)
            {
                if (isOn(vp))
                {
                    newVP = new ViewDesc(name, id_, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strncasecmp(name, "behind", 6) == 0)
            {
                if (isOn(vp))
                {
                    newVP = new ViewDesc(name, id_, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }

            else if (strncasecmp(name, "onfloor", 7) == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  onfloor id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }

            else if (strcmp(name, "0.001") == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  0.001 id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, 0.001, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strcmp(name, "0.01") == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  0.01 id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, 0.01, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strcmp(name, "0.1") == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  0.1 id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, 0.1, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strcmp(name, "1") == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  1 id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, 1, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strcmp(name, "10") == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  10 id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, 10, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    //fprintf(stderr,"vor  10 id_=%d\n", id_);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strcmp(name, "100") == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  100 id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, 100, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else if (strcmp(name, "1000") == 0)
            {
                if (isOn(vp))
                {
                    //fprintf(stderr,"vor  1000 id_=%d\n", id_);
                    newVP = new ViewDesc(name, id_, 1000, viewPointMenu_, flightMenu_, editVPMenu_, this);
                    viewpoints.push_back(newVP);
                    if (blockConfigInFlight)
                        newVP->setFlightState(false);
                    numberOfDefaultVP++;
                }
            }
            else
            {
                newVP = new ViewDesc(name, id_, vp, viewPointMenu_, flightMenu_, editVPMenu_, this);
                viewpoints.push_back(newVP);
                if (blockConfigInFlight)
                    newVP->setFlightState(false);
                numberOfDefaultVP++;
            }
            id_++;
        }
    }

    //fprintf(stderr,"---- nach default viepoints ist id_=%d\n", id_);
    // read the viewpoints from the file if given
    activated_ = false;
    
    // add quickNavNode to SG
    qnNode = new Geode();
	qnNode->setName("qnNode");
    ref_ptr<QuickNavDrawable> qnDraw = new QuickNavDrawable();
    qnNode->addDrawable(qnDraw.get());
    cover->getObjectsRoot()->addChild(qnNode.get());
    activeVP = NULL;

    flightPathVisualizer = new FlightPathVisualizer(cover, &viewpoints);
    flightPathVisualizer->showFlightpath(false);
    vpInterpolator = new Interpolator();

    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::init done\n");
    return true;
}

bool ViewPoints::init2()
{
    
    if(vwpPath == "")
    {
        vwpPath = coVRFileManager::instance()->getViewPointFile();
    }
    if ((!vwpPath.empty()) && (vwpPath[0] != '\0'))
        readFromDom();

    updateSHMData();
    if (loopMode)
        completeFlight(true);

    return true;
}
ViewPoints::~ViewPoints()
{
    while (!viewpoints.empty())
    {
        viewpoints.back()->deleteGeometry();
        delete viewpoints.back();
        viewpoints.pop_back();
    }

    //delete flightmanager;
    delete vpInterpolator;
    delete flightPathVisualizer;

	if (qnNode)
	{
		while (qnNode->getNumParents())
		{
			qnNode->getParent(0)->removeChild(qnNode);
		}
	}

    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::~ViewPoints done\n");
}

void ViewPoints::readFromDom()
{
    if (!coVRDOMDocument::instance()->read(vwpPath.c_str()))
        return;
    //fprintf(stderr, "ViewPoints::readFromDom %s\n", vwpPath);
    ViewDesc *newVP;
    int i, j;
    bool foundName = false, foundScale, foundPos, foundEuler;
    QDomElement rootElement = coVRDOMDocument::instance()->getRootElement();

    QDomNodeList vpNodeList = rootElement.elementsByTagName("VIEWPOINT");

    int nvp = vpNodeList.count();
    if (cover->debugLevel(3))
        fprintf(stderr, "dom contains %d viewpoints\n", nvp);

    // loop over all read viewpoints
    for (i = 0; i < nvp; i++)
    {
        if (cover->debugLevel(3))
            fprintf(stderr, "processing vp %d\n", i);

        QDomNode vpNode = vpNodeList.item(i);
        QDomNodeList vpChildList = vpNode.childNodes();
        int childCount = vpChildList.count();

        if (childCount < 4)
        {
            if (cover->debugLevel(3))
                fprintf(stderr, "this is not a viewpoint node\n");
            else
                fprintf(stderr, "WARNING in ViewPoints: Found invalid entry in vwp file\n");
        }
        else
        {
            bool changeable=false;
            // for all children (<name>, <scale>, <position>, <euler>, <clipplane>*n)
            for (j = 0; j < childCount; j++)
            {
                QDomNode node = vpChildList.item(j);
                if (node.isElement())
                {
                    QDomElement element = node.toElement();
                    QString tagName = element.tagName();
                        if (tagName.compare("isChangeable") == 0)
                        {
                            if(element.text() == "true")
                                changeable = true;
                        }
                }
            }
            // for all children (<name>, <scale>, <position>, <euler>, <clipplane>*n)
            for (j = 0; j < childCount; j++)
            {
                QDomNode node = vpChildList.item(j);
                if (node.isElement())
                {
                    QDomElement element = node.toElement();
                    QString tagName = element.tagName();
                    QString text = element.text();

                    if (tagName.compare("NAME") == 0)
                    {
                        if (cover->debugLevel(3))
                            fprintf(stderr, "found viewpoint element with name %s\n", (const char *)text.toLatin1());
                        foundName = true;
                        newVP = new ViewDesc(text.toLatin1(), id_, viewPointMenu_, flightMenu_, editVPMenu_, this,changeable);
                        id_++;
                        break;
                    }
                }
                else if (childCount < 4)
                {
                    if (cover->debugLevel(3))
                        fprintf(stderr, "node in vwp file is not an element\n");
                }
            } // end (for all children)

            if (foundName)
            {
                foundName = foundScale = foundPos = foundEuler = false;

                // we found the right viewpoint node, now we have to extract pos, euler, scale
                for (j = 0; j < childCount; j++)
                {
                    QDomNode node = vpChildList.item(j);
                    if (node.isElement())
                    {
                        QDomElement element = node.toElement();
                        QString tagName = element.tagName();
                        if (tagName.compare("POSITION") == 0)
                        {
                            foundPos = true;
                            newVP->setPosition(element.text().toLatin1());
                        }
                        else if (tagName.compare("EULER") == 0)
                        {
                            foundEuler = true;
                            newVP->setEuler(element.text().toLatin1());
                        }
                        else if (tagName.compare("SCALE") == 0)
                        {
                            foundScale = true;
                            newVP->setScale(element.text().toLatin1().constData());
                        }
                        else if (tagName.compare("CLIPPLANE") == 0)
                        {
                            newVP->addClipPlane(element.text().toLatin1());
                        }
                        else if (tagName.compare("TANGENT_IN") == 0)
                        {
                            newVP->setTangentIn(element.text().toLatin1());
                        }
                        else if (tagName.compare("TANGENT_OUT") == 0)
                        {
                            newVP->setTangentOut(element.text().toLatin1());
                        }
                        else if (tagName.compare("inFlightPath") == 0)
                        {
                            newVP->setFlightState(false);
                            if(element.text() == "true")
                                newVP->setFlightState(true);
                        }
                    }
                    else
                    {
                        if (cover->debugLevel(3))
                            fprintf(stderr, "node in vwp file is not an element\n");
                    }
                }
                if (foundPos && foundScale && foundEuler)
                {
                    if (cover->debugLevel(3))
                        fprintf(stderr, "appending viewpoint [%s]\n", newVP->getName());
                    viewpoints.push_back(newVP);

                    // if this is the first time we read a corect viewpoint, we activate it
                    if (!activated_)
                    {
                        newVP->activate(useClipPlanesCheck_->state());
                        activated_ = true;
                    }
                }
            }

            else // !foundName
            {
                if (cover->debugLevel(3))
                    fprintf(stderr, "this viewpoint does not have a name\n");
            }
        }
    }
}

void ViewPoints::addNode(Node *n, const RenderObject *)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::addNode()\n");
    if (!activeVP)
    {
        if (n)
        {
            std::vector<Node *> nodes;
            std::string substr = "coVR";
            addNodes(n, substr, nodes);
            if (nodes.size() > 0)
            {
                for (std::vector<Node *>::iterator it = nodes.begin(); it < nodes.end(); it++)
                {
                    string name = (*it)->getName().substr(5);
                    if (name[0] == 'V')
                    {
                        float s;
                        coCoord coord;
                        string::size_type location;
                        while ((location = name.find('_', 0)) != string::npos)
                            name.replace(location, 1, ".");
                        sscanf(name.c_str(), "VS%f=X%f=Y%f=Z%f=H%f=P%f=R%f",
                               &s, &coord.xyz[0], &coord.xyz[1], &coord.xyz[2],
                               &coord.hpr[0], &coord.hpr[1], &coord.hpr[2]);
                        if (cover->debugLevel(3))
                            fprintf(stderr, "Scale: %f\nX: %f Y:%f Z: %f\nH: %f P: %f R: %f\n",
                                    s, coord.xyz[0], coord.xyz[1], coord.xyz[2],
                                    coord.hpr[0], coord.hpr[1], coord.hpr[2]);

                        cover->setScale(s);
                        Matrix m;
                        coord.makeMat(m);
                        cover->getObjectsXform()->setMatrix(m);
                        break;
                    }
                }
            }
        }
    }
}

void ViewPoints::message(int, int, int, const void *data)
{
    const char *chbuf = (const char *)data;
    if (strncmp(chbuf, "SnapshotPlugin", strlen("SnapshotPlugin")) == 0)
    {
        chbuf += strlen("SnapshotPlugin");
        // look if a viewpoint with this name is already present
        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            if (strcmp((*it)->getName(), chbuf) == 0)
            {
                //Module->menuCallback(Module->viewpoints.current(),
                //                &(Module->viewpoints.current()->menuEntry));
                return;
            }
        }
        saveViewPoint(chbuf);
    }

    if (strncmp(chbuf, "saveViewpoint", strlen("saveViewpoint")) == 0)
    {
        //fprintf(stderr,"ViewPoints::message saveViewpoint\n");
        saveViewPoint(NULL);
    }

    if (strncmp(chbuf, "completeFlight", strlen("completeFlight")) == 0)
    {
        //cerr << "calling completeFlight" << endl;
        completeFlight(true);
    }

    if (strncmp(chbuf, "loadViewpoint", strlen("loadViewpoint")) == 0)
    {
        chbuf += strlen("loadViewpoint ");
        char *name = new char[strlen(chbuf) + 1];
        sscanf(chbuf, "%s", name);

        if (cover->debugLevel(4))
            fprintf(stderr, "calling loadViewpoint %s\n", chbuf);

        loadViewpoint(name);
    }

    if (strncmp(chbuf, "startingCapture", strlen("startingCapture")) == 0)
    {
        videoBeingCaptured = true;
    }
    if (strncmp(chbuf, "stoppingCapture", strlen("stoppingCapture")) == 0)
    {
        videoBeingCaptured = false;
    }

    if (strncmp(chbuf, "readViewpointsFile", strlen("readViewpointsFile")) == 0)
    {
        init2();
    }
}

void ViewPoints::guiToRenderMsg(const char *msg)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "\n--- Plugin ViewPoints coVRGuiToRenderMsg %s\n", msg);

    string fullMsg(string("GRMSG\n") + msg);
    coGRMsg grMsg(fullMsg.c_str());
    if (grMsg.isValid())
    {
        // gui  tells cover to load a certain viewpoint
        if (grMsg.getType() == coGRMsg::SHOW_VIEWPOINT)
        {
            coGRShowViewpointMsg vMsg(fullMsg.c_str());
            loadViewpoint(vMsg.getViewpointId());
            if (cover->debugLevel(3))
                fprintf(stderr, "--- SHOW_VIEWPOINT id=%d\n", vMsg.getViewpointId());
        }
        // when gui loads a project, it already has viewpoints with positions and orientations
        else if (grMsg.getType() == coGRMsg::CREATE_VIEWPOINT)
        {
            coGRCreateViewpointMsg vMsg(fullMsg.c_str());
            if (cover->debugLevel(3))
                fprintf(stderr, "ladeVP[%i]\n", vMsg.getViewpointId());
            createViewPoint(vMsg.getName(), vMsg.getViewpointId(), vMsg.getView(), vMsg.getClipplane());

            if (cover->debugLevel(3))
                fprintf(stderr, "--- CREATE_VIEWPOINT id=%d\n", vMsg.getViewpointId());
        }
        // when gui loads a project, it already has viewpoints with positions and orientations
        else if (grMsg.getType() == coGRMsg::CHANGE_VIEWPOINT)
        {
            coGRChangeViewpointMsg vMsg(fullMsg.c_str());
            changeViewPoint(vMsg.getViewpointId());

            if (cover->debugLevel(3))
                fprintf(stderr, "--- CHANGE_VIEWPOINT id=%d\n", vMsg.getViewpointId());
        }
        else if (grMsg.getType() == coGRMsg::CHANGE_VIEWPOINT_NAME)
        {
            coGRChangeViewpointNameMsg vMsg(fullMsg.c_str());
            changeViewPointName(vMsg.getId(), vMsg.getName());

            if (cover->debugLevel(3))
                fprintf(stderr, "--- CHANGE_VIEWPOINT id=%d\n", vMsg.getId());
        }
        else if (grMsg.getType() == coGRMsg::DELETE_VIEWPOINT)
        {
            coGRDeleteViewpointMsg vMsg(fullMsg.c_str());
            deleteViewPoint(vMsg.getViewpointId());

            if (cover->debugLevel(3))
                fprintf(stderr, "--- DELETE_VIEWPOINT id=%d\n", vMsg.getViewpointId());
        }

        // gui tells cover to create a new viewpoint, gui doesn't know the position yet
        else if (grMsg.getType() == coGRMsg::KEYWORD)
        {
            coGRKeyWordMsg keyWordMsg(fullMsg.c_str());
            const char *keyword = keyWordMsg.getKeyWord();
            if (strcmp(keyword, "saveViewPoint") == 0)
            {
                saveViewPoint();
                if (cover->debugLevel(3))
                    fprintf(stderr, "saveVP");
            }
            else if (strcmp(keyword, "sendDefaultViewPoint") == 0)
            {
                sendDefaultViewPoint();
                if (cover->debugLevel(3))
                    fprintf(stderr, "sendDefaultVP");
            }
            else if (strcmp(keyword, "turntableRotate45") == 0)
            {
                turnTableStep();
            }

            if (cover->debugLevel(3))
                fprintf(stderr, "--- KEYWORD [%s]\n", keyWordMsg.getKeyWord());
        }

        // gui changed fly mode
        else if (grMsg.getType() == coGRMsg::FLYMODE_TOGGLE)
        {
            coGRToggleFlymodeMsg flymodeMsg(fullMsg.c_str());
            //printf("GUI CHANGED FLYMODE %d\n", flymodeMsg.getMode());
            flyingMode = (bool)flymodeMsg.getMode();
            // set flyMode in menu
            flyingModeCheck_->setState(flyingMode);

            if (cover->debugLevel(3))
                fprintf(stderr, "--- FLYMODE_TOGGLE \n");
        }

        // gui changed clipplane mode
        else if (grMsg.getType() == coGRMsg::VPCLIPPLANEMODE_TOGGLE)
        {
            coGRToggleVPClipPlaneModeMsg clipplanemodeMsg(fullMsg.c_str());
            useClipPlanesCheck_->setState((bool)clipplanemodeMsg.getMode());
            if (cover->debugLevel(3))
                fprintf(stderr, "--- VPCLIPPLANEMODE_TOGGLE \n");
        }
        else if (grMsg.getType() == coGRMsg::TURNTABLE_ANIMATION)
        {
            coGRTurnTableAnimationMsg msg(fullMsg.c_str());
            startTurnTableAnimation(msg.getAnimationTime());
        }
        else
        {
            if (cover->debugLevel(3))
                fprintf(stderr, "NOT-USED\n");
        }
    }
}

void ViewPoints::key(int type, int keySym, int mod)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::key(type=%d,keySym=%d,mod=%d)\n", type, keySym, mod);

    if ((keySym >= osgGA::GUIEventAdapter::KEY_F1)
        && (keySym <= osgGA::GUIEventAdapter::KEY_F12) /*&& (type == osgGA::GUIEventAdapter::KEYUP)*/)
    {
        if (ssize_t(viewpoints.size()) > (keySym - osgGA::GUIEventAdapter::KEY_F1))
        {
            if (flyingMode)
            {
                Matrix m = cover->getObjectsXform()->getMatrix();
                currentMat_ = m;
                curr_coord = m;
                curr_scale = cover->getScale();

                double initTime = cover->frameTime();

                flyingStatus = true;
	        runButton->setState(flyingStatus);
            }
            else
            {
                viewpoints[keySym - osgGA::GUIEventAdapter::KEY_F1]->activate(useClipPlanesCheck_->state());
            }
            sendLoadViewpointMsgToGui(keySym - osgGA::GUIEventAdapter::KEY_F1);
            activeVP = viewpoints[keySym - osgGA::GUIEventAdapter::KEY_F1];
            updateViewPointIndex();

            if (cover->debugLevel(3))
                fprintf(stderr, "activated the viewpoint %s\n", viewpoints[osgGA::GUIEventAdapter::KEY_F1]->getName());
        }
        else
        {
            if (cover->debugLevel(3))
                fprintf(stderr, "Viewpoint %d not defined\n", (osgGA::GUIEventAdapter::KEY_F1 + 1));
        }
    }
    if (keySym == osgGA::GUIEventAdapter::KEY_KP_7)
    {
        saveViewPoint(NULL);
    }
    if (keySym == osgGA::GUIEventAdapter::KEY_KP_9)
    {
        completeFlight(true);
    }
//#ifndef _WIN32
    // QuickNav
    if (isQuickNavEnabled)
    {
        // toggle HUD
        if (keySym == osgGA::GUIEventAdapter::KEY_Shift_R)
        {
            if (type == 2)
            {
                enableHUD();
            }
            else
            {
                disableHUD();
                loadViewpoint(vp_index);
                sendLoadViewpointMsgToGui(vp_index);
            }
        }

        if ((mod & osgGA::GUIEventAdapter::MODKEY_SHIFT) && (type == osgGA::GUIEventAdapter::KEYUP))
        {
            //this is counter-intuitive
            if (keySym == osgGA::GUIEventAdapter::KEY_Right)
            {
                enableHUD();
                nextViewpoint();
            }
            else if (keySym == osgGA::GUIEventAdapter::KEY_Left)
            {
                enableHUD();
                previousViewpoint();
            }
        }
    }
//#endif
}

void ViewPoints::addNodes(Node *node, std::string s, std::vector<osg::Node *> &nodes)
{
    unsigned int i;
    Group *g = node->asGroup();
    if (g != NULL)
    {
        for (i = 0; i < g->getNumChildren(); i++)
        {
            addNodes(g->getChild(i), s, nodes);
        }
    }
    if (node->getName().compare(0, s.length(), s, 0, s.length()) == 0) //strncmp(s.c_str(),node->getName(),s.length())==0)
    {
        nodes.push_back(node);
    }
}

void ViewPoints::createViewPoint(const char *name, int guiId, const char *desc, const char *plane)
{
    string n = name;
    string ds = desc;

    if (cover->debugLevel(3))
        fprintf(stderr, "---- ViewPoints::createViewPoint() planeId=%s\n", plane);

    stringstream clipplane;
    clipplane << plane;

    // remove the viewpoint if it exists
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        if ((strcmp(name, (*it)->getName()) == 0) && (strcmp(desc, (*it)->getLine()) == 0))
        {
            //sendChangeIdMsgToGui(guiId, viewpoints.current()->getId());
            viewpoints.erase(it);
            (*it)->deleteGeometry();
            delete (*it);
            //fprintf(stderr,"---- ViewPoints::createViewPoint() erase %s\n" , (*it)->getName());
            break;
        }
    }

    // create new viewpoint
    ViewDesc *viewDesc = NULL;
    if (name && strlen(name) > 0)
    {
        viewDesc = new ViewDesc(n.c_str(), id_, ds.c_str(), viewPointMenu_, flightMenu_, editVPMenu_, this, true);
        //fprintf(stderr,"---- ViewPoints::createViewPoint() name=[%s] Id=%d\n" , n.c_str(), id_);
        //if (guiId!=id_)
        sendChangeIdMsgToGui(guiId, id_);
        id_++;
        activated_ = true; // when reading a file, don't activate automatically
    }

    // add Clipplanes
    int numClipplanes;
    int id;
    float a, b, c, d;
    clipplane >> numClipplanes;
    if (numClipplanes > 0)
    {
        for (int i = 1; i <= numClipplanes; i++)
        {
            clipplane >> id >> a >> b >> c >> d;
            stringstream clipToAdd;
            clipToAdd << id << " " << a << " " << b << " " << c << " " << d;
            //fprintf(stderr, "clipToAdd %s\n", clipToAdd.str().c_str());
            viewDesc->addClipPlane(clipToAdd.str().c_str());
        }
    }

    viewpoints.push_back(viewDesc);
    flightPathVisualizer->addViewpoint(viewDesc);

    // initially show viewpoint
    viewDesc->showGeometry(showViewpoints);
    viewDesc->showMoveInteractors(showInteractors);
    viewDesc->showTangentInteractors(showInteractors);
    flightPathVisualizer->showFlightpath(showFlightpath);
}

void ViewPoints::deleteViewPoint(int id)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::deleteViewPoint(id=%d) size=%d\n", id, (int)viewpoints.size());

    ViewDesc *currentPoint;
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        currentPoint = (*it);
        //fprintf(stderr,"currentPoint->getId() =%d\n", currentPoint->getId() );
        if (currentPoint->getId() == id)
        {
            //fprintf(stderr,"erase Point id=%d size=%d\n", currentPoint->getId(),(int)viewpoints.size() );
            flightPathVisualizer->removeViewpoint(currentPoint);
            viewpoints.erase(it);
            currentPoint->deleteGeometry();
            delete currentPoint;
            break;
        }
    }

    flightPathVisualizer->showFlightpath(showFlightpath);
}

void ViewPoints::saveViewPoint(const char *suggestedName)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::saveViewPoint()\n");
    static int vpnum = 1;
    char newName[100];
    char guiString[1000];

    osg::Matrix m = cover->getObjectsXform()->getMatrix();
    coCoord coord;
    coord = m;

    Vec3 tangentOut;
    tangentOut = Vec3(0, 500, 0);
    Vec3 tangentIn;
    tangentIn = Vec3(0, -500, 0);

    sprintf(guiString, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
            cover->getScale(),
            m(0, 0), m(0, 1), m(0, 2), m(0, 3),
            m(1, 0), m(1, 1), m(1, 2), m(1, 3),
            m(2, 0), m(2, 1), m(2, 2), m(2, 3),
            m(3, 0), m(3, 1), m(3, 2), m(3, 3),
            tangentOut[0], tangentOut[1], tangentOut[2], tangentIn[0], tangentIn[1], tangentIn[2]);

    ViewDesc *viewDesc = NULL;
    if (suggestedName && strlen(suggestedName) > 0)
    {
        //viewDesc = new ViewDesc(suggestedName, id_, cover->getScale(), m, viewPointMenu_,flightMenu_,editVPMenu_,this, true);
        viewDesc = new ViewDesc(suggestedName, id_, guiString, viewPointMenu_, flightMenu_, editVPMenu_, this, true);
        //fprintf(stderr,"---- ViewPoints::saveViewPoint() suggestedName=[%s] Id=%d\n" ,suggestedName, id_);
        strcpy(newName, suggestedName);
        activated_ = true; // when reading a file, don't activate automatically
    }
    else
    {
        // if suggestedName is empty save new viewpoint
        //  with default name (if it doesnt already exist)
        sprintf(newName, "NewViewpoint%d", vpnum);

        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            if (!strcmp((*it)->getName(), newName))
                sprintf(newName, "NewViewpoint%d", ++vpnum);
        }
        //viewDesc = new ViewDesc(newName, id_, cover->getScale(), m, viewPointMenu_,flightMenu_,editVPMenu_,this,true);
        viewDesc = new ViewDesc(newName, id_, guiString, viewPointMenu_, flightMenu_, editVPMenu_, this, true);
        //fprintf(stderr,"---- ViewPoints::saveViewPoint() newName=[%s] Id=%d\n" , newName, id_);
    }
    viewDesc->coord = coord;
    //nochmal extra auf false setzen, um "zufaelliges" aktivieren von shiftFlightpath bei der VP-Erstellung zu unterdrücken
    //---------------------------------------------
    viewDesc->shiftFlightpath(false);
    //----------------------------------------------
    viewpoints.push_back(viewDesc);
    flightPathVisualizer->addViewpoint(viewDesc);

    // initially show viewpoint
    viewDesc->showGeometry(showViewpoints);
    viewDesc->showTangent(showViewpoints); //neu
    viewDesc->showMoveInteractors(showInteractors);
    viewDesc->showTangentInteractors(showInteractors);
    flightPathVisualizer->showFlightpath(showFlightpath);

   
    // clipplanes
    stringstream ssplanes_final;
    ssplanes_final << "";

    ref_ptr<ClipNode> clipNode = cover->getObjectsRoot();
    ssplanes_final << clipNode->getNumClipPlanes() << " ";
    if (useClipPlanesCheck_->state())
    {

        for (unsigned int i = 0; i < clipNode->getNumClipPlanes() /*6*/; i++)
        {
            ClipPlane *cp = clipNode->getClipPlane(i);
            Vec4 plane = cp->getClipPlane();
            int number = cp->getClipPlaneNum();

            QDomElement planeElement;
            char planeString[1024];

            snprintf(planeString, 1024, "%d %f %f %f %f ", number, plane[0], plane[1], plane[2], plane[3]);
            // add to the current viewpoint
            viewDesc->addClipPlane(planeString);
            ssplanes_final << planeString << " ";
        }
    }

    //fprintf(stderr, "PLANE: %s\n", ssplanes_final.str().c_str());
    sendNewViewpointMsgToGui(newName, id_, guiString, ssplanes_final.str().c_str());
    //fprintf(stderr, "sendNewViewpointMsgToGui: newName=%s id=%d guiString=%s\n",newName, id_, guiString);
    id_++;

   
    vpnum++;

    // only write VRML viewpoints if VRML_WRITE_VIEWPOINTS is set
    if (coCoviseConfig::isOn("COVER.Plugin.ViewPoint.WriteVrmlViewpoint", false) && coVRMSController::instance()->isMaster())
    {
        Matrix mat = cover->getObjectsXform()->getMatrix();
        Matrix rotMat;
        MAKE_EULER_MAT(rotMat, 0, 90, 0);
        mat.preMult(rotMat);
        MAKE_EULER_MAT(rotMat, 0, -90, 0);
        mat.postMult(rotMat);
        Vec3 Trans;
        Trans = mat.getTrans();
        Quat quat;
        quat = m.getRotate();
        rotMat.makeRotate(quat);
        mat.postMult(rotMat);
        Trans = mat.getTrans();
        mat = cover->getObjectsXform()->getMatrix();
        MAKE_EULER_MAT(rotMat, 0, 90, 0);
        mat.preMult(rotMat);
        MAKE_EULER_MAT(rotMat, 0, -90, 0);
        mat.postMult(rotMat);
        coord = mat;
        quat = mat.getRotate();
        rotMat.makeRotate(quat);
        mat.postMult(rotMat);
        MAKE_EULER_MAT(rotMat, 0, 90, 0);
        mat.postMult(rotMat);
        Trans = mat.getTrans();
        mat.makeRotate(quat);
        mat.postMult(rotMat);
        coord = mat;
        quat = mat.getRotate();
    }
    saveAllViewPoints();
}

#include <util/covise_version.h>

void ViewPoints::saveAllViewPoints()
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::saveAllViewPoints()\n");
    int numVPs=0;
    QDomDocument doc;
    QDomElement rootElement;
    // create a dom
    rootElement = doc.createElement("COVERSTATE");
    // append something
    QDomElement tag = doc.createElement("VERSION");
    rootElement.appendChild(tag);
    QDomText t = doc.createTextNode(covise::CoviseVersion::longVersion());
    tag.appendChild(t);

    doc.appendChild(rootElement);
    // loop over all viewpoints
    for (vector<ViewDesc *>::iterator it = viewpoints.begin() + numberOfDefaultVP; it < viewpoints.end(); it++)
    {
        numVPs++;
        ViewDesc *viewDesc = (*it);
            // add new viewpoint to dom
            // append something
            QDomElement viewpointElement, nameElement, scaleElement, posElement, eulerElement, tangentInElement, tangentOutElement;
        
            //tangentOutElement, tangentInElement, stoptimeElement;
        QDomText nameText, scaleText, posText, eulerText;
        viewpointElement = doc.createElement("VIEWPOINT");

        nameElement = doc.createElement("NAME");
        nameText = doc.createTextNode(viewDesc->getName());
        nameElement.appendChild(nameText);

        char scaleString[1024];
        sprintf(scaleString, "%f", viewDesc->getScale());
        scaleElement = doc.createElement("SCALE");
        scaleText = doc.createTextNode(scaleString);
        scaleElement.appendChild(scaleText);

        char posString[1024];
        sprintf(posString, "%f %f %f ", viewDesc->coord.xyz[0], viewDesc->coord.xyz[1], viewDesc->coord.xyz[2]);
        posElement = doc.createElement("POSITION");
        posText = doc.createTextNode(posString);
        posElement.appendChild(posText);

        char eulerString[1024];
        sprintf(eulerString, "%f %f %f ", viewDesc->coord.hpr[0], viewDesc->coord.hpr[1], viewDesc->coord.hpr[2]);
        eulerElement = doc.createElement("EULER");
        eulerText = doc.createTextNode(eulerString);
        eulerElement.appendChild(eulerText);

        char tangentInString[1024];
        sprintf(tangentInString, "%f %f %f ", viewDesc->getTangentIn()[0], viewDesc->getTangentIn()[1], viewDesc->getTangentIn()[2]);
        tangentInElement = doc.createElement("TANGENT_IN");
        tangentInElement.appendChild(doc.createTextNode(tangentInString));

        char tangentOutString[1024];
        sprintf(tangentOutString, "%f %f %f ", viewDesc->getTangentOut()[0], viewDesc->getTangentOut()[1], viewDesc->getTangentOut()[2]);
        tangentOutElement = doc.createElement("TANGENT_OUT");
        tangentOutElement.appendChild(doc.createTextNode(tangentOutString));

        /*    char stoptimeString[1024];
        sprintf(stoptimeString, "%f ", viewDesc->stoptime);
        stoptimeElement = doc.createElement("STOPTIME");
        stoptimeElement.appendChild(doc.createTextNode(stoptimeString));
        */

        coVRDOMDocument::instance()->getRootElement().appendChild(viewpointElement);
        viewpointElement.appendChild(nameElement);
        viewpointElement.appendChild(scaleElement);
        viewpointElement.appendChild(posElement);
        viewpointElement.appendChild(eulerElement);

        viewpointElement.appendChild(tangentInElement);
        viewpointElement.appendChild(tangentOutElement);
        //viewpointElement.appendChild(stoptimeElement);
        
        QDomElement inFlightPath;
        inFlightPath = doc.createElement("inFlightPath");
        if(viewDesc->getFlightState())
        inFlightPath.appendChild(doc.createTextNode("true"));
        else
        inFlightPath.appendChild(doc.createTextNode("false"));
        viewpointElement.appendChild(inFlightPath);

        
        QDomElement isChangeable;
        isChangeable = doc.createElement("isChangeable");
        if(viewDesc->isChangeable())
        isChangeable.appendChild(doc.createTextNode("true"));
        else
        isChangeable.appendChild(doc.createTextNode("false"));
        viewpointElement.appendChild(isChangeable);
        
        //fprintf(stderr,"ViewPoints::saveViewPoint NAME=%s; SCALE=%s; POSITION=%s; EULER=%s\n", newName, scaleString, posString, eulerString );

        // clipplanes
        stringstream ssplanes_final;
        ssplanes_final << "";
        
    rootElement.appendChild(viewpointElement);

        ref_ptr<ClipNode> clipNode = cover->getObjectsRoot();
        ssplanes_final << clipNode->getNumClipPlanes() << " ";

        for (unsigned int i = 0; i < clipNode->getNumClipPlanes() /*6*/; i++)
        {
            if(viewDesc->isClipPlaneEnabled(i))
            {
                ClipPlane *cp = clipNode->getClipPlane(i);
                Vec4 plane = cp->getClipPlane();


                QDomElement planeElement;
                char planeString[1024];

                snprintf(planeString, 1024, "%d %f %f %f %f ", i, viewDesc->clipPlanes[i].a, viewDesc->clipPlanes[i].b, viewDesc->clipPlanes[i].c, viewDesc->clipPlanes[i].d);
                planeElement = doc.createElement("CLIPPLANE");
                planeElement.appendChild(doc.createTextNode(planeString));

                viewpointElement.appendChild(planeElement);

                // add to the current viewpoint
                viewDesc->addClipPlane(planeString);
                ssplanes_final << planeString << " ";

            }
        }
    }
    
    if (coVRMSController::instance()->isMaster() && numVPs>0) // only write a file if we have more than 0 non default viewpoints
    {
        
        if(vwpPath == "")
        {
            vwpPath = coVRFileManager::instance()->getViewPointFile();
        }
        // save the dom to file
        QFile file(vwpPath.c_str());
        file.open(QIODevice::WriteOnly);
        QTextStream stream(&file); // we will serialize the data into the file
        if (cover->debugLevel(4))
            fprintf(stderr, "%s", doc.toString().toLatin1().data());
        stream << doc.toString();
        file.close();
        if (cover->debugLevel(3))
            fprintf(stderr, "saved dom to file\n");
    }
}

void ViewPoints::changeViewPoint(int id)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::changeViewPoint() id=%d\n", id);

    ViewDesc *foundViewpoint = NULL;

    // find the viewpoint
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        if ((*it)->getId() == id)
        {
            foundViewpoint = (*it);
            break;
        }
    }

    if (foundViewpoint != NULL)
        changeViewDesc(foundViewpoint);
    else if (cover->debugLevel(3))
        fprintf(stderr, "changeViewPoint - no viewpoint with id %d", id);
}

void ViewPoints::changeViewPointName(int id, const char *newName)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::changeViewPointName() id=%d name=%s\n", id, newName);

    ViewDesc *foundViewpoint = NULL;

    // find the viewpoint
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        if ((*it)->getId() == id)
        {
            foundViewpoint = (*it);
            break;
        }
    }

    if (foundViewpoint != NULL)
    {
        foundViewpoint->setName(newName);
    }
    else
    {
        if (cover->debugLevel(3))
            fprintf(stderr, "changeViewPointName - no viewpoint with id %d", id);
    }
}

void ViewPoints::changeViewPoint(const char *name)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::changeViewPoint() name=%s\n", name);

    ViewDesc *foundViewpoint = NULL;

    // find the viewpoint
    if (name && strlen(name) > 0)
    {
        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            if (strcmp(name, (*it)->getName()) == 0)
            {
                foundViewpoint = (*it);
                break;
            }
        }
    }

    if (foundViewpoint != NULL)
        changeViewDesc(foundViewpoint);
    else if (cover->debugLevel(3))
        fprintf(stderr, "changeViewPoint - no viewpoint %s", name);
}

void ViewPoints::changeViewDesc(ViewDesc *viewDesc)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::changeViewDesc");

    if (viewDesc == NULL)
        return;

    Matrix m = cover->getObjectsXform()->getMatrix();
    float scale = cover->getScale();
    char guiString[1000];

    viewDesc->changeViewDesc(scale, m);

    // clipplanes
    stringstream ssplanes_final;
    ssplanes_final << "";

    ref_ptr<ClipNode> clipNode = cover->getObjectsRoot();
    ssplanes_final << clipNode->getNumClipPlanes() << " ";
    if (useClipPlanesCheck_->state())
    {
        for (unsigned int i = 0; i < clipNode->getNumClipPlanes() /*6*/; i++)
        {
            ClipPlane *cp = clipNode->getClipPlane(i);
            Vec4 plane = cp->getClipPlane();
            int number = cp->getClipPlaneNum();

            //QDomElement planeElement;
            char planeString[1024];

            snprintf(planeString, 1024, "%d %f %f %f %f ", number, plane[0], plane[1], plane[2], plane[3]);
            //planeElement = doc.createElement("CLIPPLANE");
            //planeElement.appendChild(doc.createTextNode(planeString));

            //viewpointElement.appendChild(planeElement);

            // add to the current viewpoint
            viewDesc->addClipPlane(planeString);
            ssplanes_final << planeString << " ";
        }
    }
    //fprintf(stderr, "PLANE: %s\n", ssplanes_final.str().c_str());

    sprintf(guiString, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
            scale,
            m(0, 0), m(0, 1), m(0, 2), m(0, 3),
            m(1, 0), m(1, 1), m(1, 2), m(1, 3),
            m(2, 0), m(2, 1), m(2, 2), m(2, 3),
            m(3, 0), m(3, 1), m(3, 2), m(3, 3),
            tangentIn[0], tangentIn[1], tangentIn[2], tangentOut[0], tangentOut[1], tangentOut[2]);

    sendViewpointChangedMsgToGui(viewDesc->getName(), viewDesc->getId(), guiString, ssplanes_final.str().c_str());
    
    saveAllViewPoints();
}

void ViewPoints::changeViewDesc(Matrix newMatrix, float newScale, Vec3 newTanIn, Vec3 newTanOut, int idd, const char *name, ViewDesc *viewDesc)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::changeViewDesc\n");

    Matrix m = newMatrix;
    float scale = newScale;
    double tangentInX = newTanIn[0];
    double tangentInY = newTanIn[1];
    double tangentInZ = newTanIn[2];
    double tangentOutX = newTanOut[0];
    double tangentOutY = newTanOut[1];
    double tangentOutZ = newTanOut[2];
    string n = name;
    int id = idd;
    char guiString[1000];

    //   viewDesc->changeViewDesc( scale, m );

    // clipplanes
    stringstream ssplanes_final;
    ssplanes_final << "";

    ref_ptr<ClipNode> clipNode = cover->getObjectsRoot();
    ssplanes_final << clipNode->getNumClipPlanes() << " ";
    if (useClipPlanesCheck_->state())
    {
        for (unsigned int i = 0; i < clipNode->getNumClipPlanes() /*6*/; i++)
        {
            ClipPlane *cp = clipNode->getClipPlane(i);
            Vec4 plane = cp->getClipPlane();
            int number = cp->getClipPlaneNum();

            //QDomElement planeElement;
            char planeString[1024];

            snprintf(planeString, 1024, "%d %f %f %f %f ", number, plane[0], plane[1], plane[2], plane[3]);
            //planeElement = doc.createElement("CLIPPLANE");
            //planeElement.appendChild(doc.createTextNode(planeString));

            //viewpointElement.appendChild(planeElement);

            // add to the current viewpoint
            viewDesc->addClipPlane(planeString);
            ssplanes_final << planeString << " ";
        }
    }
    //fprintf(stderr, "PLANE: %s\n", ssplanes_final.str().c_str());

    sprintf(guiString, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
            scale,
            m(0, 0), m(0, 1), m(0, 2), m(0, 3),
            m(1, 0), m(1, 1), m(1, 2), m(1, 3),
            m(2, 0), m(2, 1), m(2, 2), m(2, 3),
            m(3, 0), m(3, 1), m(3, 2), m(3, 3),
            tangentInX, tangentInY, tangentInZ, tangentOutX, tangentOutY, tangentOutZ);

    //fprintf(stderr, "\n saveVP: %f \n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f \n %f %f %f \n",
    //scale,
    //m(0,0), m(0,1),m(0,2), m(0,3),
    //m(1,0), m(1,1),m(1,2), m(1,3),
    //m(2,0), m(2,1),m(2,2), m(2,3),
    //m(3,0), m(3,1),m(3,2), m(3,3),
    //tangentInX, tangentInY, tangentInZ,
    //tangentOutX, tangentOutY, tangentOutZ);

    sendViewpointChangedMsgToGui(n.c_str(), id, guiString, ssplanes_final.str().c_str());
    
    saveAllViewPoints();
}

bool ViewPoints::update()
{
    if (dataChanged)
        return true;
    if (record)
        return true;
    if (!activeVP && loopMode)
        return true;
    if (flyingStatus)
        return true;
    if (activeVP && activeVP->isActivated())
        return true;
    if (turnTableAnimation_)
        return true;
    if (turnTableStep_)
        return true;
    if (sendActivatedViewpointMsg)
        return true;

    return false;
}

#define INTERVAL 0.5
#define SAVEINTERVAL 2.0
#define MAXFRAMES 12000
void ViewPoints::preFrame()
{
    if (cover->debugLevel(5))
        fprintf(stderr, "ViewPoints::preFrame() id_=%d\n", id_);
    if(dataChanged)
    {
        
        static double oldTime = 0;
        double time = cover->frameTime();
        if (time - oldTime > SAVEINTERVAL)
        {
            oldTime = time;
            dataChanged=false;
            saveAllViewPoints();
        }
    }
    if (record)
    {
        static double oldTime = 0;
        double time = cover->frameTime();
        if (time - oldTime > INTERVAL)
        {
            oldTime = time;
            Matrix mat = cover->getObjectsXform()->getMatrix(); //getXformMat();
            mat = cover->getInvBaseMat();

            Matrix rotMat;
            maxPos.push_back(mat.getTrans());
            osg::Matrix tmpMat;
            tmpMat.invert(cover->getObjectsXform()->getMatrix());
            //MAKE_EULER_MAT(rotMat, 0,90,0);
            //tmpMat.postMult(rotMat);
            MAKE_EULER_MAT(rotMat, 0, 90, 0);
            tmpMat.preMult(rotMat);
            maxQuat.push_back(tmpMat.getRotate());

            MAKE_EULER_MAT(rotMat, 0, 90, 0); //rotMat.makeEuler(0,90,0);
            mat.preMult(rotMat);
            MAKE_EULER_MAT(rotMat, 0, -90, 0); //rotMat.makeEuler(0,-90,0);
            mat.postMult(rotMat);
            Vec3 Trans;
            Trans = mat.getTrans();
            Quat quat;
            quat = mat.getRotate();
            //float angle, x, y, z;
            //quat.getRot(&angle, &x, &y, &z);
            rotMat.makeRotate(quat);
            mat.postMult(rotMat);
            Trans = mat.getTrans();
            //cerr << "position " << Trans[0]/(-cover->getScale()) << " " <<Trans[1]/(-cover->getScale())<< " " <<Trans[2]/(-cover->getScale()) << endl;
            //cerr << "orientation " << x<< " " <<y << " " <<z << " " << (angle/ -180.0) * M_PI <<endl;
            positions[frameNumber * 3] = Trans[0] / (-cover->getScale());
            positions[frameNumber * 3 + 1] = Trans[1] / (-cover->getScale());
            positions[frameNumber * 3 + 2] = Trans[2] / (-cover->getScale());
            double angle, xr, yr, zr;
            quat.getRotate(angle, xr, yr, zr);
            orientations[frameNumber * 4] = xr;
            orientations[frameNumber * 4 + 1] = yr;
            orientations[frameNumber * 4 + 2] = zr;
            orientations[frameNumber * 4 + 3] = angle;
            frameNumber++;
            if (frameNumber >= MAXFRAMES)
            {
                stopRecord();
            }
        }
    }

    //   if (activeVP != NULL && activeVP->hasMatrix())
    //   {
    //      ViewPoints::curr_translationMode = Interpolator::BEZIER;
    //   }
    //   else
    //   {
    //      ViewPoints::curr_translationMode = Interpolator::LINEAR_TRANSLATION;
    //   }

    if (!activeVP)
    {
        if (loopMode)
            completeFlight(true);
    }

    if (flyingStatus)
    {

        double thisTime = cover->frameTime();

        float lambda = (thisTime - initTime) / flightTime;
        //fprintf(stderr,"flyingStatus thisTime=%f initTime=%f lambda=%f\n", thisTime,initTime,lambda);
        if (lambda < 0)
            lambda = 0;
	    
	float delta = 100.0/(viewpoints.size()-1);
	
        animPositionSlider->setValue((flight_index + lambda)* delta);

        Matrix rotMat, initialMat, destMat;

        if (lambda < 1 && (flyingMode || flight_index >= 0))
        {
            lambda = vpInterpolator->easingFunc(curr_easingFunc, lambda, 0, 1, 1);

            // important! set initial coordinates, if destination does not have them
            Matrix interpolatedTrans;
            float interpolatedScale = curr_scale;
            Matrix interpolatedRot;

            if (lastVP != NULL && lastVP->hasMatrix())
            {
                interpolatedTrans = currentMat_;
                interpolatedRot = currentMat_;
                initialMat = currentMat_;

                if (activeVP->hasMatrix())
                {
                    destMat = activeVP->getMatrix();
                }
                else
                {
                    ViewPoints::curr_translationMode = Interpolator::LINEAR_TRANSLATION;
                    activeVP->coord.makeMat(destMat);
                }
            }
            else
            {
                ViewPoints::curr_translationMode = Interpolator::LINEAR_TRANSLATION;
                curr_coord.makeMat(initialMat);

                if (activeVP->hasMatrix())
                {
                    destMat = activeVP->getMatrix();
                }
                else
                {
                    activeVP->coord.makeMat(destMat);
                }
            }

            //      if(curr_translationMode == Interpolator::LINEAR_TRANSLATION)
            //         fprintf(stderr, "LINEAR\n");
            //      else
            //         fprintf(stderr, "BEZIER\n");

            if (activeVP->hasScale())
            {
                interpolatedScale = vpInterpolator->interpolateScale(curr_scale, activeVP, lambda);
            }
            if (activeVP->hasPosition())
            {
                // add eyepoint to calculation
                if (shiftFlightpathToEyePoint)
                {
                    initialMat(3, 0) -= eyepoint[0];
                    initialMat(3, 1) -= eyepoint[1];
                    initialMat(3, 2) -= eyepoint[2];
                    destMat(3, 0) -= eyepoint[0];
                    destMat(3, 1) -= eyepoint[1];
                    destMat(3, 2) -= eyepoint[2];
                }

                interpolatedTrans = vpInterpolator->interpolateTranslation(initialMat, tangentOut, curr_scale, destMat, activeVP->getScale(), tangentIn,
                                                                           lambda, interpolatedScale, curr_translationMode);
            }
            // for default viewpoints (e.g. left, top, ...)
            else
            {
                activeVP->coord.xyz = Vec3(0, 0, 0);
                activeVP->coord.makeMat(destMat);

                interpolatedTrans = vpInterpolator->interpolateTranslation(initialMat, tangentOut, curr_scale, destMat, activeVP->getScale(), tangentIn,
                                                                           lambda, interpolatedScale, curr_translationMode);
            }
            if (activeVP->hasOrientation())
            {
                interpolatedRot = vpInterpolator->interpolateRotation(initialMat, tangentOut, curr_scale, activeVP->getScale(), destMat, tangentIn, lambda, curr_rotationMode);

                // remove eyepoint from calculation
                if (shiftFlightpathToEyePoint)
                {
                    interpolatedRot.setTrans(eyepoint);
                }
            }

            interpolatedTrans.postMult(interpolatedRot);

            if (showCamera)
            {
                // preScale does not affect translation (we are in camera/world-space!)
                interpolatedTrans.preMultScale(Vec3(interpolatedScale, interpolatedScale, interpolatedScale));

                flightPathVisualizer->updateCamera(interpolatedTrans);
            }
            else
            {
                cover->setScale(interpolatedScale);
                cover->setXformMat(interpolatedTrans);
            }
        }
        else // finished interpolation
        {
            // update curr... if available
            //===================================================
            if (activeVP->hasMatrix())
            {
                currentMat_ = activeVP->getMatrix();
                curr_scale = activeVP->getScale();
            }
            else
            {
                if (activeVP->hasPosition())
                {
                    curr_coord.xyz[0] = activeVP->coord.xyz[0];
                    curr_coord.xyz[1] = activeVP->coord.xyz[1];
                    curr_coord.xyz[2] = activeVP->coord.xyz[2];
                }
                //für Default Viewpoints, zB left, top, ...
                else
                    curr_coord.xyz = Vec3(0, 0, 0);

                if (activeVP->hasOrientation())
                {
                    curr_coord.hpr[0] = activeVP->coord.hpr[0];
                    curr_coord.hpr[1] = activeVP->coord.hpr[1];
                    curr_coord.hpr[2] = activeVP->coord.hpr[2];
                }
                if (activeVP->hasScale() && activeVP->hasPosition())
                    curr_scale = activeVP->getScale();
            }
            //===================================================
            lastVP = activeVP;

            if (showCamera)
            {
                Matrix m;
                if (activeVP->hasMatrix())
                {
                    m = currentMat_;
                }
                else
                {
                    curr_coord.makeMat(m);
                }
                // prescale scales translation
                m.preMultScale(Vec3(curr_scale, curr_scale, curr_scale));
                flightPathVisualizer->updateCamera(m);
                flyingStatus = false;
	        runButton->setState(flyingStatus);
            }
            else
            {
                if (curr_scale > 0.0)
                    cover->setScale(curr_scale);

                if ((curr_coord.hpr[0] < 360.0) && (curr_coord.hpr[0] > -360.0))
                {
                    activeVP->activate(useClipPlanesCheck_->state()); //useClipPlanesCheck_->state() als test gesetzt
                }
            }

            // handle flight stuff
            //===================================================
            if (flight_index > 0)
            {
                flyingStatus = true;
                while (flight_index > 0 && !viewpoints[viewpoints.size() - flight_index]->getFlightState())
                {
                    flight_index--;
                }
                if (flight_index > 0)
                {
                    loadViewpoint(viewpoints[viewpoints.size() - flight_index]);
                    flight_index--;
                }
                else
                    flyingStatus = false;
	        runButton->setState(flyingStatus);
            }
            else //set the destination viewpoint
            {
                updateViewPointIndex();
                flyingStatus = false;
	        runButton->setState(flyingStatus);
                if (loopMode)
                    completeFlight(true);
            }
        } // end else lambda
    } //end flying status

    // call preFrame() on all ViewDesc
    ViewDesc *currentPoint;
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        currentPoint = (*it);
        currentPoint->preFrame(flightPathVisualizer); //parameter: flightPathVisualizer
    }

    // if viewpoint is activated (flight finished) send corresponding msg in next preFrame
    // because the view will change a last time in this preFrame
    if (sendActivatedViewpointMsg)
    {
        sendActivatedViewpointMsg = false;
        if (coVRMSController::instance()->isMaster())
        {
            coGRActivatedViewpointMsg vMsg(activeVP->getId());

            Message grmsg{ COVISE_MESSAGE_UI , DataHandle{(char*)(vMsg.c_str()),strlen(vMsg.c_str()) + 1, false } };
            coVRPluginList::instance()->sendVisMessage(&grmsg);
            if (cover->debugLevel(3))
                fprintf(stderr, "--- sendActivatedViewpointMsg\n");
        }
    }
    if (activeVP && activeVP->isActivated())
    {
        activeVP->setActivated(false);
        sendActivatedViewpointMsg = true;
    }

    if (turnTableAnimation_)
    {
        float step = cover->frameDuration();
        if (videoBeingCaptured)
        {
            step = 1.0f / 25.0f;
        }

        if (turnTableCurrent_ <= 1.0f)
        {
            // wait 1 sec at the beginning
        }
        else if (turnTableCurrent_ <= (1.0f + turnTableAnimationTime_))
        {
            float a = (360 / (turnTableAnimationTime_)) * (turnTableCurrent_ - 1.0f);
            osg::Matrix m1, m2, m;
            MAKE_EULER_MAT(m1, a, 0.0, 0.0);
            MAKE_EULER_MAT(m2, 0.0, turnTableViewingAngle_, 0.0);
            m.mult(m1, m2);
            cover->getObjectsXform()->setMatrix(m);
        }
        else if (turnTableCurrent_ <= (2.0f + turnTableAnimationTime_))
        {
            // wait 1 sec at the end
        }
        else
        {
            turnTableAnimation_ = false;
            turnTableAnimationCheck_->setState(false);
            //stop Video
            cover->sendMessage(NULL, "Video", 0, 13, "stopCapturing");
        }
        turnTableCurrent_ += step;
    }
    if (turnTableStep_)
    {
        double thisTime = cover->frameTime();;

        float lambda = (thisTime - turnTableStepInitTime_) / flightTime;
        if (lambda < 0)
            lambda = 0;

        Matrix rotMat, interpolatedRot;
        if (lambda < 1)
        {
            lambda = vpInterpolator->easingFunc(curr_easingFunc, lambda, 0, 1, 1);
            interpolatedRot = vpInterpolator->interpolateRotation(turnTableStepInitMat_, tangentOut, curr_scale, curr_scale, turnTableStepDestMat_, tangentIn, lambda, curr_rotationMode);
            cover->setXformMat(interpolatedRot);
        }
        else
            turnTableStep_ = false;
    }
}

void ViewPoints::stopRecord()
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::stopRecord()\n");
    if (!record)
        return;

    if (cover->debugLevel(3))
        fprintf(stderr, "Saving Camera %d\n", fileNumber);

    fprintf(fp, "#VRML V2.0 utf8\n\nDEF Camera%d Viewpoint {  \nposition %f %f %f\n  orientation %f %f %f %f\n  fieldOfView 0.6024 description \"Camera%d\"\n}\n", fileNumber, positions[0], positions[1], positions[2], orientations[0], orientations[1], orientations[2], orientations[3], fileNumber);
    fprintf(fp, "DEF Camera%d-TIMER TimeSensor { loop TRUE cycleInterval %f }\n", fileNumber, frameNumber * INTERVAL);
    fprintf(fp, "DEF Camera%d-POS-INTERP PositionInterpolator {\n key [\n", fileNumber);
    int i = 0;
    int z = 0;
    for (i = 0; i < frameNumber; i++)
    {
        fprintf(fp, " %f,", (float)i / (frameNumber - 1));
        z++;
        if (z > 10)
        {
            z = 0;
            fprintf(fp, "\n");
        }
    }
    fprintf(fp, "]\nkeyValue [\n");

    for (i = 0; i < frameNumber; i++)
    {
        fprintf(fp, " %f %f %f,", positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
        z++;
        if (z > 3)
        {
            z = 0;
            fprintf(fp, "\n");
        }
    }

    fprintf(fp, "] }\n");

    fprintf(fp, "DEF Camera%d-ROT-INTERP OrientationInterpolator {\n key [\n", fileNumber);
    for (i = 0; i < frameNumber; i++)
    {
        fprintf(fp, " %f,", (float)i / (frameNumber - 1));
        z++;
        if (z > 10)
        {
            z = 0;
            fprintf(fp, "\n");
        }
    }
    fprintf(fp, "]\nkeyValue [\n");

    for (i = 0; i < frameNumber; i++)
    {
        fprintf(fp, " %f %f %f %f,", orientations[i * 4], orientations[i * 4 + 1], orientations[i * 4 + 2], orientations[i * 4 + 3]);
        z++;
        if (z > 3)
        {
            z = 0;
            fprintf(fp, "\n");
        }
    }

    fprintf(fp, "] }\n");
    fprintf(fp, "ROUTE Camera%d-TIMER.fraction_changed TO Camera%d-POS-INTERP.set_fraction\n", fileNumber, fileNumber);
    fprintf(fp, "ROUTE Camera%d-POS-INTERP.value_changed TO Camera%d.set_position\n", fileNumber, fileNumber);
    fprintf(fp, "ROUTE Camera%d-TIMER.fraction_changed TO Camera%d-ROT-INTERP.set_fraction\n", fileNumber, fileNumber);
    fprintf(fp, "ROUTE Camera%d-ROT-INTERP.value_changed TO Camera%d.set_orientation\n", fileNumber, fileNumber);

    fclose(fp);

    fp = fopen("Animation.maxpos", "a+");
    for (i = 0; i < frameNumber; i++)
    {
        fprintf(fp, "%f %f %f\n", maxPos[i].x(), maxPos[i].y(), maxPos[i].z());
    }

    fclose(fp);
    fp = fopen("Animation.maxori", "a+");
    for (i = 0; i < frameNumber; i++)
    {
        double angle, xr, yr, zr;
        maxQuat[i].getRotate(angle, xr, yr, zr);
        fprintf(fp, "%f %f %f %f\n", (float)xr, (float)yr, (float)zr, (float)((angle / M_PI) * 180.0f));
    }

    fclose(fp);

    record = false;
    delete[] positions;
    delete[] orientations;
}

void ViewPoints::startRecord()
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::startRecord()\n");
    if (record)
        stopRecord();

    char fileName[100];
    sprintf(fileName, "Animation.wrl");
    fileNumber++;
    frameNumber = 0;
    fp = fopen(fileName, "a+");

    positions = new float[3 * MAXFRAMES + 6];
    orientations = new float[4 * MAXFRAMES + 8];

    std::vector<osg::Vec3> maxPos;
    std::vector<osg::Quat> maxQuat;
    maxPos.clear();
    maxQuat.clear();
    maxPos.reserve(1000);
    maxQuat.reserve(1000);
    if (fp)
    {
        record = true;
    }
    else
        perror("Animation.wrl");
}

void ViewPoints::completeFlight(bool state)
{
    if(state)
    {
    tangentIn = tangentOut = Vec3(0, 0, 0);
    int num = viewpoints.size() - 1;
    flight_index = num;

    while (flight_index >= 0 && !viewpoints[num - flight_index]->getFlightState())
    {
        flight_index--;
    }

    if (flight_index >= 0)
    {
        //Unterdrückt "wackeln" zu Flight-Beginn
        tangentIn = tangentOut = Vec3(0, 0, 0);
        //--------------------------------------------------

        Matrix m = cover->getObjectsXform()->getMatrix();
        viewpoints[num - flight_index]->activate(useClipPlanesCheck_->state());
        sendLoadViewpointMsgToGui(num - flight_index);
    }

    while (flight_index >= 0 && !viewpoints[num - flight_index]->getFlightState())
    {
        flight_index--;
    }

    if (flight_index > 0)
    {

        Matrix m = cover->getObjectsXform()->getMatrix();
        currentMat_ = m;
        curr_coord = m;
        curr_scale = cover->getScale();

        initTime = cover->frameTime();

        flyingStatus = true;
        activeVP = viewpoints[num - flight_index];
        updateViewPointIndex();
    }
    }
    else
    {
        flyingStatus = false;
    }
    runButton->setState(flyingStatus);
}

void ViewPoints::pauseFlight(bool state)
{
	static double pauseTime = 0.0;
        flyingStatus = state;
	if(state)
	{
	   pauseTime = cover->frameTime();
	}
	else
	{
	   initTime += cover->frameTime()-pauseTime;
	}
}

// this methods is called also from gui msg to cover here we should not send back a msg
void ViewPoints::loadViewpoint(int id)
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::loadViewpoint %d\n", id);
    activeVP = NULL;
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        if ((*it)->getId() == id)
        {
            activeVP = (*it);
            loadViewpoint(activeVP);
            sendLoadViewpointMsgToGui(id); // TEST fOr Clemens
        }
    }

    if (!activeVP)
    {
        return;
    }

    if (cover->debugLevel(3))
        fprintf(stderr, "activated the viewpoint %s\n", activeVP->getName());
}

// this method is not called from gui here we can send msg to gui
void ViewPoints::loadViewpoint(const char *name)
{
    //fprintf(stderr,"ViewPoints::loadViewpoint %s\n", name);
    ViewDesc *currentPoint;
    int index = 0;
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        currentPoint = (*it);
        if (strcmp(currentPoint->getName(), name) == 0)
        {
            loadViewpoint(currentPoint);
            sendLoadViewpointMsgToGui((*it)->getId());
            flight_index = -1;

            break;
        }
        index++;
    }
}

void ViewPoints::activateViewpoint(ViewDesc *viewDesc)
{
    loadViewpoint(viewDesc);
    sendLoadViewpointMsgToGui(viewDesc->getId());
    activeVP = viewDesc;
    updateViewPointIndex();
    flight_index = -1;
}

void ViewPoints::loadViewpoint(ViewDesc *viewDesc)
{
    Matrix m = cover->getObjectsXform()->getMatrix();

    // do nothing, if matrix of viewpoint to load is the same as current..
    if (viewDesc->equalVP(m))
    {
        //fprintf(stderr, "Der zu ladende Viewpoint stimmt mit der aktuellen Ansicht ueberein -> VP wird ignoriert\n");
        viewDesc->activate(useClipPlanesCheck_->state());
        return;
    }
    if (viewDesc->nearVP(m))
    {
        fprintf(stderr, "Kurze Entfernung zum Viewpoint -> lineare Interpolation\n");
        ViewPoints::curr_translationMode = Interpolator::LINEAR_TRANSLATION;
        //viewDesc->activate(useClipPlanesCheck_->state());
    }
    else
    {
        ViewPoints::curr_translationMode = Interpolator::BEZIER;
    }
    viewDesc->updateToViewAll();
    if (flyingMode)
    {

        // do not update current view position
        // camera-box should start at last vp position
        if (!showCamera)
        {
            currentMat_ = m;
            curr_coord = m;
            curr_scale = cover->getScale();
        }

        if (showCamera && lastVP == NULL)
        {
            curr_scale = viewpoints[numberOfDefaultVP]->getScale();
            currentMat_ = viewpoints[numberOfDefaultVP]->getMatrix();
            curr_coord = viewpoints[numberOfDefaultVP]->getMatrix();
            lastVP = viewpoints[numberOfDefaultVP];
        }

	
        initTime = cover->frameTime();
        // add time offset if we have an active Flight
        // if (lastVP != NULL && lastVP->equalVP(m) && flightmanager->activeFlight)
        // {
        //    initTime += flightmanager->getCurrentStoptime();
        // }
        //
        // if we have an active flight from FlightManager
        // ...change the flighttime
        // if (flightmanager->activeFlight)
        //    flightTime = flightmanager->getCurrentFlighttime();
        // else
        //    flightTime = 3.0;
        // // ...change the easing Function
        // if (flightmanager->activeFlight)
        // {
        //    curr_easingFunc = flightmanager->getCurrentEasingFunction();
        //    curr_translationMode = flightmanager->getCurrentTranslationMode();
        //    curr_rotationMode = flightmanager->getCurrentRotationMode();
        // }
        // else
        // {
        //    curr_easingFunc = Interpolator::LINEAR_EASING;
        //    curr_translationMode = Interpolator::BEZIER;
        //    curr_rotationMode = Interpolator::QUATERNION;
        // }

        flyingStatus = true;
	runButton->setState(flyingStatus);
    }
    else
    {
        viewDesc->activate(useClipPlanesCheck_->state());
    }

    // update tangents:
    // use default tangent, if we don't know last viewpoint
    //---------------------------------------------------------------
    tangentOut = Vec3(0, 0, 0);
    tangentIn = viewDesc->getTangentIn();
    if (lastVP != NULL && ((lastVP->equalVP(m)) || showCamera))
    {
        if (lastVP->getId() < viewDesc->getId())
        {
            tangentOut = lastVP->getTangentOut();
            tangentIn = viewDesc->getTangentIn();
        }
        else
        {
            tangentOut = lastVP->getTangentIn();
            tangentIn = viewDesc->getTangentOut();
        }
    }
    //---------------------------------------------------------------
    activeVP = viewDesc;
    updateViewPointIndex();
}

void ViewPoints::nextViewpoint()
{
    if (viewpoints.size() > 0)
    {
        vp_index = vp_index == ssize_t(viewpoints.size()) - 1 ? 0 : vp_index + 1;
        updateSHMData();
    }
}

void ViewPoints::previousViewpoint()
{
    if (viewpoints.size() > 0)
    {
        vp_index = vp_index == 0 ? viewpoints.size() - 1 : vp_index - 1;
        updateSHMData();
    }
}

void ViewPoints::startStopRec()
{
    if (record)
    {
        startStopRecButton_->setText("Start Record");
        stopRecord();
    }
    else
    {
        startStopRecButton_->setText("Stop Record");
        startRecord();
    }
}

void ViewPoints::updateViewPointIndex()
{
    //fprintf(stderr, "updateViewPointIndex\n");
    vp_index = 0;

    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        if (activeVP == (*it))
        {
            updateSHMData();
            return;
        }
        vp_index++;
    }
}

void ViewPoints::updateSHMData()
{
    if (viewpoints.size() > 0)
    {
        ViewDesc *current_vp = viewpoints[vp_index];

        strncpy(actSharedVPData->name, current_vp->getName(), SHARED_VP_NAME_LENGTH);

        actSharedVPData->totNum = viewpoints.size();
        actSharedVPData->index = vp_index;

        for (int ii = 0; ii < 6; ++ii)
        {
            if (current_vp->isClipPlaneEnabled(ii))
                actSharedVPData->hasClipPlane[ii] = 1;
            else
                actSharedVPData->hasClipPlane[ii] = 0;
        }
    }
}

void ViewPoints::enableHUD()
{
    actSharedVPData->isEnabled = 1;
}

void ViewPoints::disableHUD()
{
    actSharedVPData->isEnabled = 0;
}

bool ViewPoints::isClipPlaneChecked()
{
    return useClipPlanesCheck_->state();
}

bool ViewPoints::isOn(const char *vp)
{
    if ((strncasecmp(vp, "true", 4) == 0) || (strncasecmp(vp, "on", 2) == 0) || (strncasecmp(vp, "1", 1) == 0))
        return true;
    else
        return false;
}

void ViewPoints::sendDefaultViewPoint()
{
    if (cover->debugLevel(3))
        fprintf(stderr, "ViewPoints::sendDefaultViewPoint()\n");

    // get entries from covise.config
    if (coVRMSController::instance()->isMaster())
    {
        coCoviseConfig::ScopeEntries viewpointEntries = coCoviseConfig::getScopeEntries("COVER.Plugin.ViewPoint", "Viewpoints");
        const char **it = viewpointEntries.getValue();
        // save the default viewpoints in those maps
        std::set<const char *> defaultVps;
        while (it && *it)
        {
            const char *name = *it;

            it++;
            const char *vp = *it;
            it++;

            if (name && vp
                && (!strstr(name, "FLYING"))
                && (!strstr(name, "FLIGHT_TIME"))
                && (!strstr(name, "FLY_CONFIG"))
                && (!strstr(name, "LOOP"))
                && (!strstr(name, "QUICK_NAV"))
                && (!strstr(name, "VRML_WRITE_VIEWPOINT")))
            {

                // name text is now e.g. "Viewpoints:top" instead of "top"
                if (strncasecmp(name + strlen(name) - 4, "left", 4) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("left");
                }
                else if (strncasecmp(name + strlen(name) - 5, "right", 5) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("right");
                }
                else if (strncasecmp(name + strlen(name) - 5, "front", 5) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("front");
                }
                else if (strncasecmp(name + strlen(name) - 4, "back", 4) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("back");
                }
                else if (strncasecmp(name + strlen(name) - 3, "top", 3) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("top");
                }
                else if (strncasecmp(name + strlen(name) - 6, "bottom", 6) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("bottom");
                }
                else if (strncasecmp(name + strlen(name) - 7, "halftop", 7) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("halftop");
                }
                else if (strncasecmp(name + strlen(name) - 6, "center", 6) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("center");
                }
                else if (strncasecmp(name + strlen(name) - 6, "behind", 6) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("behind");
                }
                else if (strncasecmp(name + strlen(name) - 6, "onfloor", 6) == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("onfloor");
                }
                else if (strcmp(name + strlen(name) - 5, "0.001") == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("0.001");
                }
                else if (strcmp(name + strlen(name) - 4, "0.01") == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("0.01");
                }
                else if (strcmp(name + strlen(name) - 3, "0.1") == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("0.1");
                }
                else if (strcmp(name + strlen(name) - 1, "1") == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("1");
                }
                else if (strcmp(name + strlen(name) - 2, "10") == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("10");
                }
                else if (strcmp(name + strlen(name) - 3, "100") == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("100");
                }
                else if (strcmp(name + strlen(name) - 4, "1000") == 0)
                {
                    if (isOn(vp))
                        defaultVps.insert("1000");
                }
                else
                {
                    defaultVps.insert(name);
                }
            }
        }

        // go through all viewpoints and send the default viewpoints to gui
        // delete all other than default viewpoints
        for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
        {
            // do not use find because const char has to be compared with strcmp
            bool found = false;
            for (std::set<const char *>::iterator iter = defaultVps.begin(); iter != defaultVps.end(); iter++)
            {
                if (strcmp((*iter), (*it)->getName()) == 0)
                {
                    found = true;
                    break;
                }
            }

            if (found)
            {
                sendNewDefaultViewpointMsgToGui((*it)->getName(), (*it)->getId());
            }
            else
            {
                viewpoints.erase(remove(viewpoints.begin(), viewpoints.end(), *it), viewpoints.end());
            }
        }
    }
}

ui::Action *ViewPoints::getMenuButton(const std::string &buttonName)
{
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        if (strcmp(buttonName.c_str(), (*it)->getName()) == 0)
        {
            return (*it)->getButton();
        }
    }
    return NULL;
}

void ViewPoints::sendNewViewpointMsgToGui(const char *name, int index, const char *str, const char *plane)
{

    if (coVRMSController::instance()->isMaster())
    {
        // send to GUI
        //fprintf(stderr,"--- sendNewViewpointMsgToGui \n");
        coGRCreateViewpointMsg vMsg(name, index, str, plane);

        Message grmsg{ COVISE_MESSAGE_UI , DataHandle{(char*)(vMsg.c_str()),strlen(vMsg.c_str()) + 1, false } };
        coVRPluginList::instance()->sendVisMessage(&grmsg);
        if (cover->debugLevel(3))
            fprintf(stderr, "--- sendNewViewpointMsgToGui \n");
    }
}

void ViewPoints::sendNewDefaultViewpointMsgToGui(const char *name, int index)
{

    if (coVRMSController::instance()->isMaster())
    {
        // send to GUI
        coGRCreateDefaultViewpointMsg vMsg(name, index);

        Message grmsg{ COVISE_MESSAGE_UI , DataHandle{(char*)(vMsg.c_str()),strlen(vMsg.c_str()) + 1, false } };
        coVRPluginList::instance()->sendVisMessage(&grmsg);
        if (cover->debugLevel(3))
            fprintf(stderr, "--- sendNewDefaultViewpointMsgToGui\n");
    }
}

void ViewPoints::sendViewpointChangedMsgToGui(const char *name, int index, const char *str, const char *plane)
{

    (void)plane;

    if (coVRMSController::instance()->isMaster())
    {
        // send to GUI
        //fprintf(stderr,"--- sendViewpointChangedMsgToGui_ \n");
        coGRViewpointChangedMsg vMsg(index, name, str /*, plane*/);

        Message grmsg{ COVISE_MESSAGE_UI , DataHandle{(char*)(vMsg.c_str()),strlen(vMsg.c_str()) + 1, false } };
        coVRPluginList::instance()->sendVisMessage(&grmsg);
        if (cover->debugLevel(3))
            fprintf(stderr, "--- sendViewpointChangedMsgToGui \n");
    }
}

void ViewPoints::sendLoadViewpointMsgToGui(int index)
{

    //fprintf(stderr,"---- ViewPoints::sendLoadViewpointMsgToGui %d\n", index);
    if (coVRMSController::instance()->isMaster())
    {
        // send to GUI
        //fprintf(stderr,"ViewPoints::sendLoadViewpointMasgToGui(%d)\n", index);
        coGRShowViewpointMsg vMsg(index);

        Message grmsg{ COVISE_MESSAGE_UI , DataHandle{(char*)(vMsg.c_str()),strlen(vMsg.c_str()) + 1, false } };
        coVRPluginList::instance()->sendVisMessage(&grmsg);
        if (cover->debugLevel(3))
            fprintf(stderr, "--- sendLoadViewpointMsgToGui\n");
    }
}

void ViewPoints::sendFlyingModeToGui()
{
    if (coVRMSController::instance()->isMaster())
    {
        // send to GUI
        //fprintf(stderr,"ViewPoints::sendFlyingModeToGui(%d)\n", flyingMode);
        coGRToggleFlymodeMsg vMsg((int)flyingMode);

        Message grmsg{ COVISE_MESSAGE_UI , DataHandle{(char*)(vMsg.c_str()),strlen(vMsg.c_str()) + 1, false } };
        coVRPluginList::instance()->sendVisMessage(&grmsg);
        if (cover->debugLevel(3))
            fprintf(stderr, "--- sendFlyingModeToGui\n");
    }
}

void ViewPoints::sendChangeIdMsgToGui(int guiId, int newId)
{
    //fprintf(stderr,"----ViewPoints::sendChangeIdMsgToGui(%d, %d)\n", guiId, newId);
    if (coVRMSController::instance()->isMaster())
    {
        coGRChangeViewpointIdMsg cidMsg(guiId, newId);

        Message grmsg{ COVISE_MESSAGE_UI , DataHandle{(char*)(cidMsg.c_str()),strlen(cidMsg.c_str()) + 1, false } };
        coVRPluginList::instance()->sendVisMessage(&grmsg);
        if (cover->debugLevel(3))
            fprintf(stderr, "--- sendChangeIdMsgToGui\n");
    }
}

void ViewPoints::alignViewpoints(char alignment)
{
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        ViewDesc *currentPoint = (*it);
        currentPoint->alignViewpoint(alignment);
    }
}

/*
 * This method changes the tangents of all viewpoints
 * due to the catmull-rom schema
 */
void ViewPoints::setCatmullRomTangents()
{
    // catmull-rom only possible with 3 or more viewpoints
    if (viewpoints.size() < 3)
        return;

    ViewDesc *Pprev = viewpoints.at(0);
    ViewDesc *Pcurr = viewpoints.at(1);
    ViewDesc *Pnext;

    for (size_t i = 2; i < viewpoints.size(); i++)
    {
        ViewDesc *currentPoint = viewpoints.at(i);
        Pnext = currentPoint;
        if (Pprev->hasTangent() && Pcurr->hasTangent() && Pnext->hasTangent())
        {
            //fprintf(stderr, " Viewpoint.at(%i)", i);
            Matrix mNext;
            mNext = Pnext->localDCS->getMatrix();
            Matrix mPrev;
            mPrev = Pprev->localDCS->getMatrix();
            Matrix mCurr;
            mCurr = Pcurr->localDCS->getMatrix();
            Vec3 next;
            next = mNext.getTrans();

            Vec3 prev;
            prev = mPrev.getTrans();

            Vec3 curr = (next - prev);

            mCurr.invert(mCurr);
            curr = Matrix::transform3x3(curr, mCurr);
            curr *= Pcurr->getScale();

            curr /= 4;
            Pcurr->setTangentOut(curr);
            Pcurr->setTangentIn(-curr);
        }
        Pprev = Pcurr;
        Pcurr = Pnext;
    }

    flightPathVisualizer->updateDrawnCurve();
    flightPathVisualizer->showFlightpath(showFlightpath);
}

void ViewPoints::setStraightTangents()
{
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        ViewDesc *currentPoint = (*it);
        if (currentPoint->hasTangent())
        {
            Vec3 tanIn = currentPoint->getTangentIn();
            Vec3 tanOut = currentPoint->getTangentOut();

            currentPoint->setTangentIn(Vec3(0, -tanIn.length(), 0));
            currentPoint->setTangentOut(Vec3(0, tanOut.length(), 0));
        }
    }
    flightPathVisualizer->updateDrawnCurve();
    flightPathVisualizer->showFlightpath(showFlightpath);
}

void ViewPoints::setEqualTangents()
{
    for (vector<ViewDesc *>::iterator it = viewpoints.begin(); it < viewpoints.end(); it++)
    {
        ViewDesc *currentPoint = (*it);
        if (currentPoint->hasTangent())
        {
            Vec3 tanIn = currentPoint->getTangentIn();
            Vec3 tanOut = currentPoint->getTangentOut();

            if (tanIn.length() > tanOut.length())
            {
                currentPoint->setTangentOut(-tanIn);
            }
            else
                currentPoint->setTangentIn(-tanOut);
        }
    }
    flightPathVisualizer->updateDrawnCurve();
    flightPathVisualizer->showFlightpath(showFlightpath);
}

void ViewPoints::startTurnTableAnimation(float time)
{
    if (!turnTableAnimation_)
    {
        turnTableCurrent_ = 0.0f;
        turnTableAnimationTime_ = time;
        // jump to start position
        VRSceneGraph::instance()->scaleAllObjects();
        osg::Matrix m1, m2, m;
        MAKE_EULER_MAT(m1, 0.0, 0.0, 0.0);
        MAKE_EULER_MAT(m2, 0.0, turnTableViewingAngle_, 0.0);
        m.mult(m1, m2);
        cover->getObjectsXform()->setMatrix(m);
        // go
        turnTableAnimation_ = true;
    }
    else
    {
        turnTableAnimation_ = false;
    }
}
void ViewPoints::turnTableStep()
{
    if (turnTableAnimation_)
    {
        turnTableAnimation_ = false;
    }

        double turnTableStepInitTime_ = cover->frameTime();
    osg::Matrix m1, m2, m;
    turnTableStep_ = true;
    VRSceneGraph::instance()->scaleAllObjects();

    turnTableStepInitMat_ = cover->getObjectsXform()->getMatrix();

    turnTableStepAlpha_ += 45.0;
    if (turnTableStepAlpha_ >= 360.0)
        turnTableStepAlpha_ = 0;
    MAKE_EULER_MAT(m1, turnTableStepAlpha_, 0.0, 0.0);
    MAKE_EULER_MAT(m2, 0.0, turnTableViewingAngle_, 0.0);
    turnTableStepDestMat_.mult(m1, m2);
}

COVERPLUGIN(ViewPoints)
