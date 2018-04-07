/*
 * SmartConeAppl.cpp
 * Author: Jia Guo
 * Created on: 03/31/2018
 */

#include "SmartConeAppl.h"

#include <stdlib.h>

using Veins::TraCIMobilityAccess;
using Veins::AnnotationManagerAccess;

Define_Module(SmartConeAppl);

void SmartConeAppl::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);
    if (stage == 0) {
        //setup veins pointers
        mobility = TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        lastSent = simTime();
        traciVehicle->setLaneChangeMode(512);
        /*
         * The default lane change mode is 0b011001010101 = 1621
         * which means that the laneChangeModel may execute all changes unless in conflict with TraCI.
         * Requests from TraCI are handled urgently but with full consideration for safety constraints.
         * To disable all autonomous changing but still handle safety checks in the simulation, either one of the modes 256 (collision avoidance)
         * or 512 (collision avoidance and safety-gap enforcement) may be used.
         */
    }
}

void SmartConeAppl::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
}

void SmartConeAppl::onData(WaveShortMessage* wsm) {
    // Receive a message with the target laneId and target speed, slow down to that speed and change lane
    std::string message = wsm->getWsmData();
    std::targetLaneId = strtok(message, " ");
    float speed = atof(strtok(NULL, " "));

    // check if the vehicle is on the target lane
    if (traciVehicle->getLaneId() == targetLaneId) {
        traciVehicle->slowDown(message_speed, 5000); //slow down over 1s
        //TODO: the duration should be at least the time estimated to pass the construction zone
        traciVehicle->changeLane(1, 5000); // merge to the left lane
        /*
         * The enumeration index of the lane (0 is the rightmost lane, <NUMBER_LANES>-1 is the leftmost one)
         */
    } else {
        // ignore the message
    }
}

void SmartConeAppl::onBeacon(WaveShortMessage* wsm) {
    //do something on receiving a message from a beacon
}

void SmartConeAppl::handlePositionUpdate(cObject* obj) {
    BaseWaveApplLayer::handlePositionUpdate(obj);
    double suggestedSpeed = 40;
    double minSpeed = 53.3;

    // TODO: update every 30s
    while (true) {
        for (int i = 2; i <= 10; i++) {
            std::string laneId = "1_" << i;
            double meanSpeed = traci->lane(laneId).getMeanSpeed();
            if (meanSpeed < minSpeed) {
                std::string message = laneId << " " << suggestedSpeed;
                sendMessage(message);
                lastSent = simTime();
                break;
            } else {
                continue;
            }
        }

    }

    //sends message every 5 seconds
//    if (simTime() - lastSent >= 5) {
//        std::string message = std::to_string(suggestedSpeed);
//        sendMessage(message);
//        lastSent = simTime();
//    }
}

void SmartConeAppl::sendWSM(WaveShortMessage* wsm) {
    sendDelayedDown(wsm, individualOffset);
}

void SmartConeAppl::sendMessage(std::string msg) {
    t_channel channel = dataOnSch ? type_SCH : type_CCH;
    WaveShortMessage* wsm = prepareWSM("data", dataLengthBits, channel, dataPriority, -1,2);
    wsm->setWsmData(msg.c_str());
    sendWSM(wsm);
}

/**/
