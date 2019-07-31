/* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */

 /****************************************************************************\
  **                                                            (C)2017 HLRS  **
  **                                                                          **
  ** Description: SumoTraCI - Traffic Control Interface client                **
  ** for traffic simulations with Sumo software - http://sumo.dlr.de          **
  **                                                                          **
 \****************************************************************************/

#ifdef _MSC_VER
#include "windows_config.h"
#else
#include "config.h"
#endif

#include "SumoTraCI.h"

#include <utils/common/SUMOTime.h>

#include <iostream>
#include <algorithm>
#include <cmath>
#include <random>

#include <cover/coVRPluginSupport.h>
#include <cover/coVRMSController.h>
#include <config/CoviseConfig.h>
#include <TrafficSimulation/Vehicle.h>
#include <TrafficSimulation/CarGeometry.h>
#include <net/tokenbuffer.h>
#include <util/unixcompat.h>

#include <cover/ui/Menu.h>
#include <cover/ui/Button.h>
#include <cover/ui/Slider.h>


int gPrecision;

using namespace opencover;

SumoTraCI::SumoTraCI() : ui::Owner("SumoTraCI", cover->ui)
{
    fprintf(stderr, "SumoTraCI::SumoTraCI\n");
    initUI();
    const char *coviseDir = getenv("COVISEDIR");
    std::string defaultDir = std::string(coviseDir) + "/share/covise/vehicles";
    vehicleDirectory = covise::coCoviseConfig::getEntry("value","COVER.Plugin.SumoTraCI.VehicleDirectory", defaultDir.c_str());
    //AgentVehicle *av = getAgentVehicle("veh_passenger","passenger","veh_passenger");
    //av = getAgentVehicle("truck","truck","truck_truck");
    pf = PedestrianFactory::Instance();
    pedestrianGroup =  new osg::Group;
    pedestrianGroup->setName("pedestrianGroup");
    cover->getObjectsRoot()->addChild(pedestrianGroup.get());
    pedestrianGroup->setNodeMask(pedestrianGroup->getNodeMask() & ~Isect::Update); // don't use the update traversal, tey are updated manually when in range
    getPedestriansFromConfig();
    getVehiclesFromConfig();
    loadAllVehicles();
}

AgentVehicle *SumoTraCI::getAgentVehicle(const std::string &vehicleID, const std::string &vehicleClass, const std::string &vehicleType)
{

    auto vehiclesPair = vehicleModelMap.find(vehicleClass);
    auto vehicles = vehiclesPair->second;
    static std::mt19937 gen2(0);
    std::uniform_int_distribution<> dis(0, vehicles->size()-1);
    int vehicleIndex = dis(gen2);

    AgentVehicle *av;
    auto avIt = vehicleMap.find(vehicles->at(vehicleIndex).vehicleName);
    if(avIt != vehicleMap.end())
    {
        av = avIt->second;
    }
    else
    {
        av= new AgentVehicle(vehicleID, new CarGeometry(vehicleID, vehicles->at(vehicleIndex).fileName, false), 0, NULL, 0, 1, 0.0, 1);
        vehicleMap.insert(std::pair<std::string, AgentVehicle *>(vehicles->at(vehicleIndex).vehicleName,av));
    }
    return av;
}



SumoTraCI::~SumoTraCI()
{
    fprintf(stderr, "SumoTraCI::~SumoTraCI\n");
    if(coVRMSController::instance()->isMaster())
    {
        client.close();
    }
    //cover->getScene()->removeChild(vehicleGroup);
}

bool SumoTraCI::init()
{
    fprintf(stderr, "SumoTraCI::init\n");
    if(coVRMSController::instance()->isMaster())
    {
        bool connected = false;
        do{

        try {
            client.connect("localhost", 1337);
            connected = true;
        }
        catch (tcpip::SocketException&) {
            fprintf(stderr, "could not connect to localhost port 1337\n");
            usleep(10000);
        }
        } while (!connected);
    }

    // identifiers: 57, 64, 67, 73, 79
    variables = { VAR_POSITION3D, VAR_SPEED, VAR_ANGLE, VAR_VEHICLECLASS, VAR_TYPE };
    subscribeToSimulation();

    if(coVRMSController::instance()->isMaster())
    {
        client.simulationStep();
        simResults = client.vehicle.getAllSubscriptionResults();
        pedestrianSimResults = client.person.getAllSubscriptionResults();
        sendSimResults();
    }
    else
    {
        readSimResults();
    }
    previousResults = currentResults;
    simTime = cover->frameTime();
    currentTime = cover->frameTime();

    if(coVRMSController::instance()->isMaster())
    {
        client.simulationStep();
        simResults = client.vehicle.getAllSubscriptionResults();
        pedestrianSimResults = client.person.getAllSubscriptionResults();
        sendSimResults();
    }
    else
    {
        readSimResults();
    }
    nextSimTime = cover->frameTime();

    updateVehiclePosition();

    //find TAZs
    if (coVRMSController::instance()->isMaster())
    {
        std::vector<std::string> edges = client.edge.getIDList();
        for (auto iter = edges.begin(); iter != edges.end(); iter++)
        {
            if (!(iter->compare(0, 3, "taz")))
            {
                fprintf(stderr, "Found TAZ %s \n", iter->c_str());
                if (compareTAZending(*iter, "source"))
                    sourceTAZs.push_back(*iter);
                else if (compareTAZending(*iter, "sink"))
                    sinkTAZs.push_back(*iter);
            }
        }
    }
    //AgentVehicle* tmpVehicle = createVehicle("passenger", "audi", "12");
    //tmpVehicle->setTransform(osg::Matrix::translate(5,0,0));
    lastParticipantStartedTime = 0.0;
    return true;
}

bool SumoTraCI::initUI()
{
traciMenu = new ui::Menu("TraCI", this);
pedestriansVisible = new ui::Button(traciMenu,"Pedestrians");
    pedestriansVisible->setCallback([this](bool state){
        if (state)
        {
        setPedestriansVisible(true);
        }
        else
        {
        setPedestriansVisible(false);
        }
    });
    pauseUI = new ui::Button(traciMenu, "Pause");

    addTrafficUI = new ui::Button(traciMenu, "AddNewPersons");
    addTrafficUI->setText("Add new Persons");
    addTrafficUI->setState(false);
    
    trafficRateUI = new ui::Slider(traciMenu, "timeBetweenNew");
    trafficRateUI->setText("seconds until new traffic participant starts route");
    trafficRateUI->setBounds(0.0, 60.0);
    trafficRateUI->setPresentation(ui::Slider::AsDial);
    trafficRateUI->setValue(1.0);

    //Modal Split
    modalSplit bus;
    bus.modalSplitButton = new ui::Button(traciMenu, "fixModalSplitBus");
    bus.modalSplitSlider = new  ui::Slider(traciMenu, "modalSplitBus");
    bus.modalSplitString = "bus";
    bus.modalSplitSlider->setBounds(0.0, 100.0);
    bus.modalSplitSlider->setValue(25.0);
    bus.modalSplitSlider->setCallback([this](double value, bool released) {
        balanceModalSplits();
    });
    modalSplits.push_back(bus);

    /*modalSplitCyclingUI = new  ui::Slider(traciMenu, "modalSplitCycling");
    modalSplits.insert(std::pair<std::string, ui::Slider *>("bicycle", modalSplitCyclingUI));
    modalSplitCyclingUI->setBounds(0.0, 100.0);
    modalSplitCyclingUI->setValue(25.0);
    fixModalSplitCyclingUI = new ui::Button(traciMenu, "fixModalSplitCycling");*/
    
    modalSplit passenger;
    passenger.modalSplitButton = new ui::Button(traciMenu, "fixModalSplitpassenger");
    passenger.modalSplitSlider = new  ui::Slider(traciMenu, "modalSplitpassenger");
    passenger.modalSplitString = "passenger";
    passenger.modalSplitSlider->setBounds(0.0, 100.0);
    passenger.modalSplitSlider->setValue(25.0);
    passenger.modalSplitSlider->setCallback([this](double value, bool released) {
        balanceModalSplits();
    });
    modalSplits.push_back(passenger);

    modalSplit pedestrian;
    pedestrian.modalSplitButton = new ui::Button(traciMenu, "fixModalSplitpedestrian");
    pedestrian.modalSplitSlider = new  ui::Slider(traciMenu, "modalSplitpedestrian");
    pedestrian.modalSplitString = "pedestrian";
    pedestrian.modalSplitSlider->setBounds(0.0, 100.0);
    pedestrian.modalSplitSlider->setValue(25.0);
    pedestrian.modalSplitSlider->setCallback([this](double value, bool released) {
        balanceModalSplits();
    });
    modalSplits.push_back(pedestrian);

    
    balanceModalSplits();
    
    return true;
}

bool SumoTraCI::compareTAZending(std::string& TAZ, std::string ending)
{
    if (TAZ.length() >= ending.length())
    {
        return (0 == TAZ.compare(TAZ.length() - ending.length(), ending.length(), ending));
    }
    else 
    {
        return false;
    }
}

void SumoTraCI::preFrame()
{
    //cover->watchFileDescriptor();
    previousTime = currentTime;
    currentTime = cover->frameTime();
    framedt = currentTime - previousTime;
    if (coVRMSController::instance()->isMaster())
    {
        if (addTrafficUI->state() && (trafficRateUI->value() < (currentTime - lastParticipantStartedTime)))
        {
            static std::mt19937 gen(0);
            std::uniform_real_distribution<> dis(0.0, 100.0);
            double mode = dis(gen);
            std::string newPerson;
            for (auto iter = modalSplits.begin(); iter != modalSplits.end(); iter++)
            {
                mode -= iter->modalSplitSlider->value();
                newPerson = iter->modalSplitString.c_str();
                if (mode <= 0)
                    break;
            }
            fprintf(stderr, "Random Vehicle will be %s \n", newPerson.c_str());

            std::vector<std::string> routes = client.route.getIDList();
            std::string uniqueID = "unique" + std::to_string(uniqueIDValue);
            std::string uniquePersonID = "uniquePerson" + std::to_string(uniqueIDValue);
            std::string uniqueRouteID = "uniqueRoute" + std::to_string(uniqueIDValue);
            uniqueIDValue++;
            std::vector<std::string> edges = client.edge.getIDList();

            //client.simulation.getDistanceRoad(edges[1], 0.0, edges[2], 0.0);
            if ((sourceTAZs.size() > 0) && (sinkTAZs.size() > 0))
            {
                static std::mt19937 gen2(0);
                static std::mt19937 gen3(0);
                std::uniform_int_distribution<> dis2(0, sourceTAZs.size() - 1);
                std::uniform_int_distribution<> dis3(0, sinkTAZs.size() - 1);
                int sourceTAZ = dis2(gen2);
                int sinkTAZ = dis3(gen3);
                while (sinkTAZ == sourceTAZ)
                {
                    sinkTAZ = dis3(gen3);
                }
                libsumo::TraCIStage newStage = client.simulation.findRoute(sourceTAZs[sourceTAZ], sinkTAZs[1], "ped_pedestrian");
                //fprintf(stderr, "Starting intermodal route at %s \n", newStage.edges.begin()->c_str());
                std::string fromEdge = *(newStage.edges.begin() + 1);
                std::string toEdge = *(newStage.edges.end() - 2);
                std::vector<libsumo::TraCIStage> newIntermodalRoute = client.simulation.findIntermodalRoute(fromEdge, toEdge, "public");
                if (newIntermodalRoute.size() > 1)
                    fprintf(stderr, "Somebody used the bus!");
                client.route.add(uniqueRouteID, newStage.edges);

                if (newPerson.compare("passenger"))
                {
                    client.vehicle.add(uniqueID, uniqueRouteID);
                }
                else if ((newPerson.compare("pedestrian")) || (newPerson.compare("bus")))
                {
                    client.person.add(uniquePersonID, *newIntermodalRoute.begin()->edges.begin(), 0.0, -1.0);// , "ped_pedestrian");
                    client.person.appendWalkingStage(uniquePersonID, newIntermodalRoute.begin()->edges, 0.0);
                }
                //client.vehicle.add(uniqueID, *routes.begin(), "DEFAULT_VEHTYPE","-1","first","base","0","current","max","current", TAZs[1],TAZs[2]);
                //client.vehicle.setRoute(uniqueID,newStage.edges);
                //std::vector<std::string> route1Edges = client.route.getEdges(*routes.begin());
                //client.person.add(uniquePersonID, *edges.begin(),0.0,-1.0,"ped_pedestrian");

                //std::vector<std::string> ped1Edges = client.person.getEdges(pedestrianSimResults.begin()->first);
                //std::vector<std::string> lanes = client.lane.getIDList();
                //std::vector<std::string> allowedOnLane = client.lane.getAllowed(*lanes.begin());
                //client.person.appendWaitingStage(uniquePersonID,1000.0);
                lastParticipantStartedTime = currentTime;
            }
            else
            {
                fprintf(stderr, "Adding persons only works with TAZs");
            }

        }
    }
    if ((currentTime - nextSimTime) > 1)
    {
        subscribeToSimulation();
        simTime = nextSimTime;
        nextSimTime = cover->frameTime();
        previousResults = currentResults;
        
        if(coVRMSController::instance()->isMaster())
        {
            if(!pauseUI->state())
            { 
            client.simulationStep();
            simResults = client.vehicle.getAllSubscriptionResults();
            pedestrianSimResults = client.person.getAllSubscriptionResults();
            }
            else
            {
                simResults.clear();
                pedestrianSimResults.clear();
            }
            sendSimResults();
        }
        else
        {
            readSimResults();
        }
        
        updateVehiclePosition();
    }
    else
    {
        interpolateVehiclePosition();
    }
}

void SumoTraCI::sendSimResults()
{
    if(currentResults.size() != simResults.size()+pedestrianSimResults.size())
    {
        currentResults.resize(simResults.size()+pedestrianSimResults.size());
    }
    size_t i = 0;

    for (std::map<std::string, libsumo::TraCIResults>::iterator it = simResults.begin(); it != simResults.end(); ++it)
    {
        std::shared_ptr<libsumo::TraCIPosition> position = std::dynamic_pointer_cast<libsumo::TraCIPosition> (it->second[VAR_POSITION3D]);
        currentResults[i].position = osg::Vec3d(position->x, position->y, position->z);
        std::shared_ptr<libsumo::TraCIDouble> angle = std::dynamic_pointer_cast<libsumo::TraCIDouble> (it->second[VAR_ANGLE]);
        currentResults[i].angle = angle->value;
        std::shared_ptr<libsumo::TraCIString> vehicleClass = std::dynamic_pointer_cast<libsumo::TraCIString> (it->second[VAR_VEHICLECLASS]);
        currentResults[i].vehicleClass = vehicleClass->value;
        std::shared_ptr<libsumo::TraCIString> vehicleType = std::dynamic_pointer_cast<libsumo::TraCIString> (it->second[VAR_TYPE]);
        currentResults[i].vehicleType = vehicleType->value;
        currentResults[i].vehicleID = it->first;
        i++;
        //fprintf(stderr,"VehicleClass is %s \n", vehicleClass->getString().c_str());
        //fprintf(stderr,"VehicleID is %s \n", it->first.c_str());
    }
    for (std::map<std::string, libsumo::TraCIResults>::iterator it = pedestrianSimResults.begin(); it != pedestrianSimResults.end(); ++it)
    {
        std::shared_ptr<libsumo::TraCIPosition> position = std::dynamic_pointer_cast<libsumo::TraCIPosition> (it->second[VAR_POSITION3D]);
        currentResults[i].position = osg::Vec3d(position->x, position->y, position->z);
        std::shared_ptr<libsumo::TraCIDouble> angle = std::dynamic_pointer_cast<libsumo::TraCIDouble> (it->second[VAR_ANGLE]);
        currentResults[i].angle = angle->value;
        std::shared_ptr<libsumo::TraCIString> vehicleClass = std::dynamic_pointer_cast<libsumo::TraCIString> (it->second[VAR_VEHICLECLASS]);
        currentResults[i].vehicleClass = vehicleClass->value;
        std::shared_ptr<libsumo::TraCIString> vehicleType = std::dynamic_pointer_cast<libsumo::TraCIString> (it->second[VAR_TYPE]);
        currentResults[i].vehicleType = vehicleType->value;
        currentResults[i].vehicleID = it->first;
        i++;
        //fprintf(stderr,"VehicleType is %s \n", vehicleType->getString().c_str());
        //fprintf(stderr,"VehicleClass is %s \n", vehicleClass->getString().c_str());
        //fprintf(stderr,"VehicleID is %s \n", it->first.c_str());
    }

    covise::TokenBuffer stb;
    unsigned int currentSize = currentResults.size();
    stb << currentSize;
    for(size_t i=0;i < currentResults.size(); i++)
    {
        double x = currentResults[i].position[0];
        double y = currentResults[i].position[1];
        double z = currentResults[i].position[2];
        stb << x;
        stb << y;
        stb << z;
        stb << currentResults[i].angle;
        stb << currentResults[i].vehicleClass;
        stb << currentResults[i].vehicleType;
        stb << currentResults[i].vehicleID;
    }
    unsigned int sizeInBytes=stb.getData().length();
    coVRMSController::instance()->sendSlaves(&sizeInBytes,sizeof(sizeInBytes));
    coVRMSController::instance()->sendSlaves(stb.getData().data(),sizeInBytes);
}
void SumoTraCI::readSimResults()
{
    unsigned int sizeInBytes=0;
    coVRMSController::instance()->readMaster(&sizeInBytes,sizeof(sizeInBytes));
    char *buf = new char[sizeInBytes];
    coVRMSController::instance()->readMaster(buf,sizeInBytes);
    covise::TokenBuffer rtb((const char *)buf,sizeInBytes);
    unsigned int currentSize;
    rtb >> currentSize;
    if(currentSize !=currentResults.size() )
    {
        currentResults.resize(currentSize);
    }
    
    for(int i=0;i < currentResults.size(); i++)
    {
        double x;
        double y;
        double z;
        rtb >> x;
        rtb >> y;
        rtb >> z;
        currentResults[i].position[0]=x;
        currentResults[i].position[1]=y;
        currentResults[i].position[2]=z;
        rtb >>  currentResults[i].angle;
        rtb >>  currentResults[i].vehicleClass;
        rtb >>  currentResults[i].vehicleType;
        rtb >>  currentResults[i].vehicleID;
    }
}

void SumoTraCI::subscribeToSimulation()
{
    if(coVRMSController::instance()->isMaster())
    {
        if (client.simulation.getMinExpectedNumber() > 0)
        {
            std::vector<std::string> departedIDList = client.simulation.getDepartedIDList();
            for (std::vector<std::string>::iterator it = departedIDList.begin(); it != departedIDList.end(); ++it)
            {
                client.vehicle.subscribe(*it, variables, 0, TIME2STEPS(1000));
            }
            std::vector<std::string> personIDList = client.person.getIDList();
            for (std::vector<std::string>::iterator it = personIDList.begin(); it != personIDList.end();it++)
            {
                client.person.subscribe(*it, variables, 0, TIME2STEPS(1000));
            }
            //fprintf(stderr, "There are currently %lu persons in the simulation \n", (unsigned long)personIDList.size());
        }
        else
        {
            fprintf(stderr, "no expected vehicles in simulation\n");
        }
    }
}

void SumoTraCI::updateVehiclePosition()
{
    osg::Matrix rotOffset;
    rotOffset.makeRotate(M_PI_2, 0, 0, 1);
    for(int i=0;i < currentResults.size(); i++)
    {
        osg::Quat orientation(osg::DegreesToRadians(currentResults[i].angle), osg::Vec3d(0, 0, -1));
        if (!currentResults[i].vehicleClass.compare("pedestrian"))
        {
            if (pedestrianModels.size() > 0)
            {
                if (loadedPedestrians.find(currentResults[i].vehicleID) == loadedPedestrians.end())
                {
                    loadedPedestrians.insert(std::pair<const std::string, PedestrianGeometry *>((currentResults[i].vehicleID), createPedestrian(currentResults[i].vehicleClass, currentResults[i].vehicleType, currentResults[i].vehicleID)));
                }
            }
        }
        else
        {
            auto matchingType = vehicleModelMap.find(currentResults[i].vehicleClass);
            if ((matchingType != vehicleModelMap.end()) && (matchingType->second->size() > 0))
            {
                // new vehicle appeared
                if (loadedVehicles.find(currentResults[i].vehicleID) == loadedVehicles.end())
                {
                    loadedVehicles.insert(std::pair<const std::string, AgentVehicle *>((currentResults[i].vehicleID), createVehicle(currentResults[i].vehicleClass, currentResults[i].vehicleType, currentResults[i].vehicleID)));
                }
                else
                {
                    /*osg::Matrix rmat,tmat;
                rmat.makeRotate(orientation);
                tmat.makeTranslate(currentResults[i].position);
                loadedVehicles.find(currentResults[i].vehicleID)->second->setTransform(rotOffset*rmat*tmat);*/
                }
            }
        }
    }
}

void SumoTraCI::interpolateVehiclePosition()
{
    osg::Matrix rotOffset;
    rotOffset.makeRotate(M_PI_2, 0, 0, 1);
    double simdt = nextSimTime - simTime;
    for(int i=0;i < previousResults.size(); i++)
    {
        int currentIndex =-1;
        // delete vehicle that will vanish in next step
        for(int n=0;n < currentResults.size(); n++)
        {
            if(previousResults[i].vehicleID == currentResults[n].vehicleID)
            {
                currentIndex = n;
                break;
            }
        }

        if (!previousResults[i].vehicleClass.compare("pedestrian"))
        {
            PedestrianMap::iterator itr = loadedPedestrians.find(previousResults[i].vehicleID);

            if (itr != loadedPedestrians.end())
            {
                if (currentIndex == -1)
                {
                    delete itr->second;
                    loadedPedestrians.erase(itr);
                }
                else
                {
                    PedestrianGeometry * p = itr->second;

                    double weight = currentTime - nextSimTime;
                    if (isnan(currentResults[currentIndex].position.x()) || isnan(currentResults[currentIndex].position.x()) || isnan(currentResults[currentIndex].position.x()))
                    {
                        currentResults[currentIndex].position = previousResults[i].position;
                    }
                    osg::Vec3d position = interpolatePositions(weight, previousResults[i].position, currentResults[currentIndex].position);
                    osg::Quat pastOrientation(osg::DegreesToRadians(previousResults[i].angle), osg::Vec3d(0, 0, -1));
                    osg::Quat futureOrientation(osg::DegreesToRadians(currentResults[currentIndex].angle), osg::Vec3d(0, 0, -1));
                    osg::Quat orientation;
                    orientation.slerp(weight, pastOrientation, futureOrientation);

                    Transform trans = Transform(Vector3D(position.x(),position.y(),position.z()),Quaternion(orientation.w(),orientation.x(),orientation.y(),orientation.z()));
                    p->setTransform(trans,M_PI);


                    if (simdt>0.0)
                    {
                        double walkingSpeed = (currentResults[currentIndex].position - previousResults[i].position).length()/simdt;
                        std::string person = itr->first;
                        /*if (!person.compare("ped51"))
                        {
                        fprintf(stderr, "Persons %s speed is %f \n", person.c_str(), walkingSpeed);
                        fprintf(stderr, "Persons %s previous position is %f %f %f \n", person.c_str(), previousResults[i].position.x(), previousResults[i].position.y(), previousResults[i].position.z());
                        fprintf(stderr, "Persons %s current position is %f %f %f \n", person.c_str(), currentResults[currentIndex].position.x(), currentResults[currentIndex].position.y(), currentResults[currentIndex].position.z());
                        }*/
                        if (!(walkingSpeed==walkingSpeed))
                        {
                            walkingSpeed = 0.0;
                        }
                        p->setWalkingSpeed(walkingSpeed);
                    }
                    else
                    {
                        p->setWalkingSpeed(0.0);
                    }

                    if(p->isGeometryWithinLOD())
                    {
                        p->update(cover->frameDuration());
                    }
                }
            }
        }
        else
        {
            //osg::Quat orientation(osg::DegreesToRadians(previousResults[i].angle), osg::Vec3d(0, 0, -1));
            std::map<const std::string, AgentVehicle *>::iterator itr = loadedVehicles.find(previousResults[i].vehicleID);

            if (itr != loadedVehicles.end())
            {
                if (currentIndex == -1)
                {
                    //delete itr->second;
                    loadedVehicles.erase(itr);
                }
                else
                {
                    double weight = currentTime - nextSimTime;

                    osg::Vec3d position = interpolatePositions(weight, previousResults[i].position, currentResults[currentIndex].position);
                    double drivingSpeed = (currentResults[currentIndex].position - previousResults[i].position).length()/simdt;

                    osg::Quat pastOrientation(osg::DegreesToRadians(previousResults[i].angle), osg::Vec3d(0, 0, -1));
                    osg::Quat futureOrientation(osg::DegreesToRadians(currentResults[currentIndex].angle), osg::Vec3d(0, 0, -1));
                    osg::Quat orientation;
                    orientation.slerp(weight, pastOrientation, futureOrientation);

                    osg::Matrix rmat, tmat;
                    rmat.makeRotate(orientation);
                    tmat.makeTranslate(position);
                    AgentVehicle * av = itr->second;
                    av->setTransform(rotOffset*rmat*tmat);
                    VehicleState vs;
                    vs.du = drivingSpeed;
                    av->getCarGeometry()->updateCarParts(1, framedt, vs);
                }
            }
        }
    }
}

osg::Vec3d SumoTraCI::interpolatePositions(double lambda, osg::Vec3d pastPosition, osg::Vec3d futurePosition)
{
    osg::Vec3d interpolatedPosition;
    for (int i = 0; i < 3; ++i)
    {
        double interpolatedPoint = futurePosition[i] + (1.0 - lambda) * (pastPosition[i] - futurePosition[i]);
        interpolatedPosition[i] = interpolatedPoint;
    }
    return interpolatedPosition;
}

double SumoTraCI::interpolateAngles(double lambda, double pastAngle, double futureAngle)
{
    double interpolatedPosition = futureAngle + (1.0 - lambda) * (pastAngle - futureAngle);
    return interpolatedPosition;
}

AgentVehicle* SumoTraCI::createVehicle(const std::string &vehicleClass, const std::string &vehicleType, const std::string &vehicleID)
{
    AgentVehicle *av = getAgentVehicle(vehicleID,vehicleClass,vehicleType);

    VehicleParameters vp;
    vp.rangeLOD = 400;
    return new AgentVehicle(av, vehicleID,vp,NULL,0.0,0);
}

PedestrianGeometry* SumoTraCI::createPedestrian(const std::string &vehicleClass, const std::string &vehicleType, const std::string &vehicleID)
{
    std::string ID = vehicleID;
    PedestrianAnimations a = PedestrianAnimations();
    std::string modelFile;

    static std::mt19937 gen(0);
    std::uniform_int_distribution<> dis(0, pedestrianModels.size()-1);
    int pedestrianIndex = dis(gen);

    pedestrianModel p = pedestrianModels[pedestrianIndex];
    return new PedestrianGeometry(ID, p.fileName,p.scale, 40.0, a, pedestrianGroup);
}

void SumoTraCI::getPedestriansFromConfig()
{
    covise::coCoviseConfig::ScopeEntries e = covise::coCoviseConfig::getScopeEntries("COVER.Plugin.SumoTraCI.Pedestrians");
    const char **entries = e.getValue();
    if (entries)
    {
        while (*entries)
        {
            const char *fileName = *entries;
            entries++;
            const char *scaleChar = *entries;
            entries++;
            double scale = atof(scaleChar);
            pedestrianModels.push_back(pedestrianModel(fileName, scale));
        }
    }
}

void SumoTraCI::getVehiclesFromConfig()
{
    for (std::vector<std::string>::iterator itr=vehicleClasses.begin(); itr!=vehicleClasses.end(); itr++)
    {
        std::vector<vehicleModel> * vehicles = new std::vector<vehicleModel>;
        vehicleModelMap[*itr]=vehicles;
        covise::coCoviseConfig::ScopeEntries e = covise::coCoviseConfig::getScopeEntries("COVER.Plugin.SumoTraCI.Vehicles."+*itr);
        const char **entries = e.getValue();
        if (entries)
        {
            while (*entries)
            {
                const char *vehicleName = *entries;
                entries++;
                const char *fileName = *entries;
                entries++;
                vehicleModel m = vehicleModel(vehicleName, fileName);
                vehicles->push_back(m);
            }
        }
        if (vehicles->size() == 0)
        {
            fprintf(stderr, "please add vehicle config %s\n", ("COVER.Plugin.SumoTraCI.Vehicles." + *itr).c_str());
        }
    }
}

void SumoTraCI::loadAllVehicles()
{
    for (auto itr = vehicleModelMap.begin(); itr!=vehicleModelMap.end(); itr++)
    {
        auto vehicles = itr->second;
        for (auto itr2 =vehicles->begin(); itr2!= vehicles->end(); itr2++)
        {
            AgentVehicle * av;
            av= new AgentVehicle("test1", new CarGeometry("test2", itr2->fileName, false), 0, NULL, 0, 1, 0.0, 1);
            vehicleMap.insert(std::pair<std::string, AgentVehicle *>(itr2->vehicleName,av));
        }
    }
}

void SumoTraCI::setPedestriansVisible(bool pedestrianVisibility)
{
    if (m_pedestrianVisible != pedestrianVisibility)
    {
        if (pedestrianVisibility)
            cover->getObjectsRoot()->addChild(pedestrianGroup.get());
        else
            cover->getObjectsRoot()->removeChild(pedestrianGroup.get());
        m_pedestrianVisible = pedestrianVisibility;
    }
}

bool SumoTraCI::balanceModalSplits()
{
    double fixedSplits = 0.0;
    double nonFixedSplits = 0.0;
    int numFixed = 0;
    for (auto iter = modalSplits.begin(); iter != modalSplits.end(); iter++)
    {
        if (iter->modalSplitButton->state())
        {
            fixedSplits += iter->modalSplitSlider->value();
            numFixed++;
        }
        else 
            nonFixedSplits += iter->modalSplitSlider->value();
    }
    if (fixedSplits > 100.0)
    {
        for (auto iter = modalSplits.begin(); iter != modalSplits.end(); iter++)
        {
            if (iter->modalSplitButton->state())
                iter->modalSplitSlider->setValue(iter->modalSplitSlider->value() / fixedSplits * 100.0);
            else
                iter->modalSplitSlider->setValue(0.0);
        }
        fixedSplits = 100.0;
    }
    if (numFixed < modalSplits.size() && (nonFixedSplits>0.0))
    {
        double factor = (100.0 - fixedSplits)/nonFixedSplits;
        for (auto iter = modalSplits.begin(); iter != modalSplits.end(); iter++)
        {
            if (!iter->modalSplitButton->state())
            {
                iter->modalSplitSlider->setValue(iter->modalSplitSlider->value() * factor);
            }
        }
    }
    else if ((numFixed == modalSplits.size()) && (fixedSplits < 100.0))
    {
        double factor = (100.0 / fixedSplits);
        for (auto iter = modalSplits.begin(); iter != modalSplits.end(); iter++)
        {
            iter->modalSplitSlider->setValue(iter->modalSplitSlider->value() * factor);
        }
    }
    return 0;
}
COVERPLUGIN(SumoTraCI)
