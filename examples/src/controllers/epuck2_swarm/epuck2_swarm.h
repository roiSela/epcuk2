/**
 * @file <epuck2_swarm.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_SWARM_H
#define EPUCK2_SWARM_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_tof_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
#include <argos3/core/simulator/entity/entity.h>

using namespace argos;

const Real SPEED = 1.0;
const Real ROTATION_SPEED = 2.0;
const Real ALIGN_SPEED = 0.75;
const Real COLLISION_DISTANCE = 500.0; // 0.975;

class CEPuck2Swarm: public CCI_Controller {

    public:

        CEPuck2Swarm();

        virtual ~CEPuck2Swarm() {
        }

        virtual void Init(TConfigurationNode &t_node);

        virtual void ControlStep();

        virtual void Reset();

        virtual void Destroy() {
        }

    private:

        enum CState {
            NONE,
            MOVING,
            ROTATING
        };

        std::string State(const CState state);
        void Register();
        float Collision(const float distance);
        float Rotation_Time(const float speed, const int degrees);

        CCI_DifferentialSteeringActuator *m_pcWheels;
        CCI_EPuck2ProximitySensor *m_pcProximity;
        CCI_EPuck2LEDsActuator *m_pcLedAct;
        CCI_EPuck2TOFSensor *m_pcTOFSensor;
        CCI_RangeAndBearingActuator* m_pcRABAct;
        CCI_RangeAndBearingSensor* m_pcRABSens;

        CState m_cState, m_cNext;
        Real m_rRotationTime, m_rDistance;
        unsigned m_uEnd_Rotation;
        int m_iDebug;

};

#endif
