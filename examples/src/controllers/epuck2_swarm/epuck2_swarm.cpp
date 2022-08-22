/**
 * @file <epuck2_swarm.cpp>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#include "epuck2_swarm.h"

#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/simulator/simulator.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>
#include "math.h"

/****************************************/
/****************************************/

CEPuck2Swarm::CEPuck2Swarm() :
        m_pcWheels(NULL), m_pcProximity(NULL), m_pcLedAct(NULL), m_pcTOFSensor(NULL),
        m_pcRABAct(NULL), m_pcRABSens(NULL),
        m_cState(MOVING), m_cNext(NONE), m_rRotationTime(0.0),
        m_rDistance(0.0), m_uEnd_Rotation(0), m_iDebug(0) {
}

/****************************************/
/****************************************/

void CEPuck2Swarm::Init(TConfigurationNode &t_node) {
    m_pcWheels    = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
    m_pcProximity = GetSensor  <CCI_EPuck2ProximitySensor       >("epuck2_proximity"     );
    m_pcLedAct    = GetActuator<CCI_EPuck2LEDsActuator          >("epuck2_leds"          );
    m_pcTOFSensor = GetSensor  <CCI_EPuck2TOFSensor             >("epuck2_tof"           );
    m_pcRABAct    = GetActuator<CCI_RangeAndBearingActuator     >("range_and_bearing"    );
    m_pcRABSens   = GetSensor  <CCI_RangeAndBearingSensor       >("range_and_bearing"    );

    GetNodeAttributeOrDefault(t_node, "debug", m_iDebug, m_iDebug);
    GetNodeAttributeOrDefault(t_node, "distance", m_rDistance, m_rDistance);

}

/****************************************/
/****************************************/

void CEPuck2Swarm::ControlStep() {

    CRadians cZAngle, cYAngle, cXAngle;
    CEPuck2Entity cEPuck = *dynamic_cast<CEPuck2Entity*>(&(&CSimulator::GetInstance())->GetSpace().GetEntity(CCI_Controller::GetId()));
    cEPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
    CDegrees cAngle = ToDegrees(cZAngle);

    std::string sId = CCI_Controller::GetId();
    unsigned uTick = (&CSimulator::GetInstance())->GetSpace().GetSimulationClock();
    CByteArray cTxData = CByteArray(3, (UInt8) 0);
    cTxData[0] = (UInt8) atoi(sId.c_str());

    if (sId != "-1") {
        float angle = 0.0;
        Real x = 0.0;
        Real y = 0.0;

        /* Get RAB messages */
        const CCI_RangeAndBearingSensor::TReadings& tMsgs = m_pcRABSens->GetReadings();
        if(!tMsgs.empty()) {

            for(size_t i = 0; i < tMsgs.size(); ++i) {
                CByteArray cRxData = tMsgs[i].Data;
                Real range = tMsgs[i].Range;
                CRadians bearing = tMsgs[i].HorizontalBearing;
                // LOG << sId << "> Rx: " << std::fixed << std::setprecision(3) << range << " " << ToDegrees(bearing).GetValue() << std::endl;
                if (m_iDebug >= 3) {
                    LOG << sId << "> Rx: " << range << " " << ToDegrees(bearing).GetValue() << ": " << cRxData;
                }
//                 int iSource = cRxData[0];
//                 int iDest   = cRxData[1];
//                 char chMsg  = cRxData[2];

                if (range < m_rDistance) {
                    x -= (m_rDistance - range) * sin(bearing.GetValue());
                    y -= (m_rDistance - range) * cos(bearing.GetValue());
                } else {
                    x += (range - m_rDistance) * sin(bearing.GetValue());
                    y += (range - m_rDistance) * cos(bearing.GetValue());
                }
            }
            if (x != 0.0 || y != 0.0) {
                angle = atan2(x, y) * 180 / CRadians::PI.GetValue();
                if (m_iDebug >= 2) {
                    LOG << sId << "> x,y: " << x << "," << y << " angle: " << angle << std::endl;
                }
            }
        }

        /* Get readings from proximity sensor */
        if (m_iDebug >= 4) {
            const CCI_EPuck2ProximitySensor::TReadings &tProxReads = m_pcProximity->GetReadings();
            LOG << sId << "> Proximity: " << std::fixed << std::setprecision(3);
            for (size_t i = 0; i < tProxReads.size()-1; ++i) {
                LOG << ToDegrees(tProxReads[i].Angle).GetValue() << ": " << tProxReads[i].Value << " # ";
            }
            LOG << ToDegrees(tProxReads[tProxReads.size()-1].Angle).GetValue() << ": " << tProxReads[tProxReads.size()-1].Value << std::endl;
        }

        switch (m_cState) {
            /***************************************************************/
            case MOVING:
                {
                    m_pcLedAct->SetBodyLed(false);
                    float collision = Collision(COLLISION_DISTANCE);
                    if (collision != 0.0) {
                        if (m_iDebug >= 2) {
                            LOG << sId << "> Collision: " << collision << std::endl;
                        }
                        m_rRotationTime = uTick + Rotation_Time(ROTATION_SPEED, collision);
                        m_cState = ROTATING;
                        m_cNext = MOVING;
                        if (collision < 0.0) {
                            m_pcWheels->SetLinearVelocity(ROTATION_SPEED, -ROTATION_SPEED);
                        } else {
                            m_pcWheels->SetLinearVelocity(-ROTATION_SPEED, ROTATION_SPEED);
                        }
                    } else {
                        Real target =  (cAngle + CDegrees(angle)).GetValue();
                        if (m_iDebug >= 2) {
                            LOG << sId << "> Angle: " << angle << " Current: " << cAngle.GetValue() << " Target: " << target << std::endl;
                        }
                        // if ( (angle > -15.0 && angle < 15.0) || angle < -165.0 || angle > 165.0 ) {
                        Real error = target - cAngle.GetValue();
                        if (std::abs(error) < 15.0) {
                            float offset = sin(CRadians(error * 180.0 / 15.0).GetValue()) / 5.0;
                            if (m_iDebug >= 2) {
                                LOG << sId << "> Error: " << error << " Offset: " << offset << ": " << SPEED * (1.0 + offset) << "," << SPEED * (1.0 - offset) << std::endl;
                            }
                            m_pcWheels->SetLinearVelocity(SPEED * (1.0 - offset), SPEED * (1.0 + offset));
                        } else {
                            if (m_iDebug >= 2) {
                                LOG << sId << "> Will Rotate: " << angle << std::endl;
                            }
                            m_rRotationTime = uTick + Rotation_Time(ROTATION_SPEED, angle);
                            m_cState = ROTATING;
                            m_cNext = MOVING;
                            if (angle < 0.0) {
                                m_pcWheels->SetLinearVelocity(ROTATION_SPEED, -ROTATION_SPEED);
                            } else {
                                m_pcWheels->SetLinearVelocity(-ROTATION_SPEED, ROTATION_SPEED);
                            }
                        }
                    }
                }
                break;
            /***************************************************************/
            case ROTATING:
                if (uTick > m_rRotationTime) {
                    m_cState = m_cNext;
                    m_uEnd_Rotation = uTick;
                    if (m_iDebug >= 2) {
                        LOG << sId << "> End Rotation: " << cAngle.GetValue() << std::endl;
                    }
                }
                m_pcLedAct->SetBodyLed(true);
                break;
            /***************************************************************/
            default:
                LOGERR << sId << "> " << "ERROR" << std::endl;
        }
        if (m_iDebug >= 1) {
            LOG << sId << "> " << State(m_cState) << " -> " << State(m_cNext) << std::endl;
        }
    }
    cTxData[1] = (UInt8) 0;
    cTxData[2] = (UInt8) '-';
    m_pcRABAct->SetData(cTxData);
    if (m_iDebug >= 3) {
       LOG << sId << "> Tx:" << cTxData;
    }
    m_pcRABAct->SetData(cTxData);

}

/****************************************/
/****************************************/

std::string CEPuck2Swarm::State(const CState state) {
    switch (state) {
        case NONE: return "NONE";
        case MOVING: return "MOVING";
        case ROTATING: return "ROTATING";
        default: return "";
    }
}

void CEPuck2Swarm::Reset() {
    m_cState = MOVING;
    m_cNext = NONE;
    m_uEnd_Rotation = 0;
    m_rRotationTime = 0.0;
}

float CEPuck2Swarm::Collision(const float distance) {
    float x = 0.0;
    float y = 0.0;
    for (unsigned int i = 0; i < m_pcProximity->GetReadings().size(); i++) {
      if (m_pcProximity->GetReadings()[i].Value > distance) {
            x -= m_pcProximity->GetReadings()[i].Value * cos(m_pcProximity->GetReadings()[i].Angle.GetValue());
            y -= m_pcProximity->GetReadings()[i].Value * sin(m_pcProximity->GetReadings()[i].Angle.GetValue());
      }
    }
    if (x != 0.0 || y != 0.0) {
        return atan2(y, x) * 180 / CRadians::PI.GetValue();
    } else {
        return 0.0;
    }
}

float CEPuck2Swarm::Rotation_Time(const float speed, const int degrees) {
    return 162 / speed * std::abs(degrees) / 360;
}

/****************************************/
/****************************************/

REGISTER_CONTROLLER(CEPuck2Swarm, "epuck2_swarm_controller")
