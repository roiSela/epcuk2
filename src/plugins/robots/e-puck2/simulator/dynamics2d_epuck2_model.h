/**
 * @file <argos3/plugins/robots/e-puck2/simulator/dynamics2d_epuck2_model.h>
 *
 * @author Daniel H. Stolfi based on the Carlo Pinciroli's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef DYNAMICS2D_EPUCK2_MODEL_H
#define DYNAMICS2D_EPUCK2_MODEL_H

namespace argos {
   class CDynamics2DDifferentialSteeringControl;
   class CDynamics2DGripper;
   class CDynamics2DGrippable;
   class CDynamics2DEPuckModel;
}

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_differentialsteering_control.h>
#include <argos3/plugins/robots/e-puck2/simulator/epuck2_entity.h>

namespace argos {

   class CDynamics2DEPuck2Model : public CDynamics2DSingleBodyObjectModel {

   public:

      CDynamics2DEPuck2Model(CDynamics2DEngine& c_engine,
                              CEPuck2Entity& c_entity);
      virtual ~CDynamics2DEPuck2Model();

      virtual void Reset();

      virtual void UpdateFromEntityStatus();

   private:

      CEPuck2Entity& m_cEPuckEntity;
      CWheeledEntity& m_cWheeledEntity;

      CDynamics2DDifferentialSteeringControl m_cDiffSteering;

      const Real* m_fCurrentWheelVelocity;

   };

}

#endif
