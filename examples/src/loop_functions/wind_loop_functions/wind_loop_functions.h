#ifndef WIND_LOOP_FUNCTIONS_H
#define WIND_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

/* wind applied as velocity in PreStep() */
class CWindLoopFunctions : public CLoopFunctions {

public:
   void Init(TConfigurationNode& t_node) override;
   void PreStep() override;                 /* v = v_wheel + wind */

private:
   CVector2 m_cWindCms;                     /* cm s⁻¹ (world) */
};

#endif
