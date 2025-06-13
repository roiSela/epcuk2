#ifndef WIND_LOOP_FUNCTIONS_H
#define WIND_LOOP_FUNCTIONS_H

#include <argos3/core/simulator/loop_functions.h>
#include <argos3/core/utility/math/vector2.h>

using namespace argos;

class CWindLoopFunctions : public CLoopFunctions {
public:
   void Init(TConfigurationNode&) override;
   void PreStep()               override;
private:
   CVector2 m_cWindCms;
   Real     m_fDt{0.1f};
};
#endif
