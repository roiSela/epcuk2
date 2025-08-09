#include "air_resistance.h"

#include <argos3/core/simulator/simulator.h>
#include <argos3/core/simulator/space/space.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/math/angles.h>

/* needed to downcast the top-level entity to access components */
#include <argos3/core/simulator/entity/composable_entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>

/* dyn2d model + chipmunk body */
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/chipmunk-physics/include/chipmunk.h>

#include <cmath>

using namespace argos;

/* 1 cm s⁻¹ of wind ⇒ this fraction of m·v added each tick */
static constexpr Real WIND_IMPULSE_SCALE = 3.5;

/* ------------------------------------------------------------------ */
void CAirResistance::Init(TConfigurationNode& t_node) {

   /* generic device handles */
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
   m_pcPos    = GetSensor  <CCI_PositioningSensor>           ("positioning");

   GetNodeAttributeOrDefault(t_node, "velocity", m_fBaseCms, m_fBaseCms);

   /* fetch <configuration><air_resistance …/> ---------------------- */
   TConfigurationNode& tRoot = CSimulator::GetInstance().GetConfigurationRoot();
   TConfigurationNode  tConf = GetNode(tRoot, "configuration");
   TConfigurationNode  tAir  = GetNode(tConf, "air_resistance");

   Real deg = 0.0, mag = 0.0;
   GetNodeAttribute(tAir, "angle_deg", deg);
   GetNodeAttribute(tAir, "magnitude", mag);

   const Real rad = deg * ARGOS_PI / 180.0;
   m_cWindCms.Set(mag * std::cos(rad),
                  mag * std::sin(rad));
}

/* ------------------------------------------------------------------ */
void CAirResistance::LazyInitBody() {
   if(m_bBodyReady) return;

   /* get this controller's entity and required components */
   CEntity& cEntity = CSimulator::GetInstance().GetSpace().GetEntity(GetId());

   auto& cComposable = dynamic_cast<CComposableEntity&>(cEntity);
   auto& cEmbodied   = *dynamic_cast<CEmbodiedEntity*>(
                          &cComposable.GetComponent("body"));

   auto* pcModel = dynamic_cast<CDynamics2DSingleBodyObjectModel*>(
                     &cEmbodied.GetPhysicsModel("dyn2d"));
   if(!pcModel)
      THROW_ARGOSEXCEPTION("No dyn2d model for " << GetId());

   m_ptBody     = pcModel->GetBody();
   m_bBodyReady = true;
}

/* ------------------------------------------------------------------ */
void CAirResistance::ControlStep() {

   LazyInitBody();

   /* 2. apply wind impulse once per tick ---------------------------- */
   Real mass = cpBodyGetMass(m_ptBody);            // robot mass (kg)
   CVector2 J = (m_cWindCms / 100.0) * mass * WIND_IMPULSE_SCALE;

   cpBodyApplyImpulse(m_ptBody,
                      cpv(J.GetX(), J.GetY()),
                      cpvzero);

   /* 1. drive forward ---------------------------------------------- */
   m_pcWheels->SetLinearVelocity(m_fBaseCms, m_fBaseCms);

   // TODO: addResistanceControlStep() to hold wind handling + lazy init
}

/* ------------------------------------------------------------------ */
REGISTER_CONTROLLER(CAirResistance,
                    "air_resistance_controller")
