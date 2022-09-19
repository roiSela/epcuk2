/**
 * @file <epuck2_leds.h>
 *
 * @author Daniel H. Stolfi
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_LEDS_H
#define EPUCK2_LEDS_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/plugins/robots/e-puck2/control_interface/ci_epuck2_leds_actuator.h>

using namespace argos;

class CEPuck2Leds : public CCI_Controller {

public:

   CEPuck2Leds();

   virtual ~CEPuck2Leds() {}

   virtual void Init(TConfigurationNode& t_node);

   virtual void ControlStep();

   virtual void Reset() {}

   virtual void Destroy() {}

private:

   CCI_EPuck2LEDsActuator* m_pcLedAct;

};

#endif
