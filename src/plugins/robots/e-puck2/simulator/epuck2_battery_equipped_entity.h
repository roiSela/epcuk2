/**
 * @file <argos3/plugins/robots/e-puck2/battery_equipped_entity.h>
 *
 * @author  Daniel H. Stolfi based on the Adhavan Jayabalan's work
 *
 * ADARS project -- PCOG / SnT / University of Luxembourg
 */

#ifndef EPUCK2_BATTERY_EQUIPPED_ENTITY_H
#define EPUCK2_BATTERY_EQUIPPED_ENTITY_H

namespace argos {
   class CEPuck2BatteryEquippedEntity;
   class CEPuck2BatteryDischargeModel;
}

#include <argos3/core/utility/math/vector3.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <map>

namespace argos {

   /****************************************/
   /****************************************/
   
   /**
    * The battery entity.
    *
    * This entity is used to model the fact that robots have
    * batteries. This entity allows the user to specify the initial
    * level of the battery, its maximum charge, and the discharge
    * model. A few discharge models are offered by default.
    */
   class CEPuck2BatteryEquippedEntity : public CEntity {

   public:

      ENABLE_VTABLE();
      Real FULL_CHARGE = 1.0;

   public:

      CEPuck2BatteryEquippedEntity(CComposableEntity* pc_parent);

      CEPuck2BatteryEquippedEntity(CComposableEntity *pc_parent,
                             const std::string& str_id,
                             CEPuck2BatteryDischargeModel* pc_discharge_model = NULL,
                             Real f_start_charge = 1.0,
                             Real f_full_charge = 1.0);

      CEPuck2BatteryEquippedEntity(CComposableEntity *pc_parent,
                             const std::string& str_id,
                             const std::string& str_discharge_model,
                             Real f_start_charge = 1.0,
                             Real f_full_charge = 1.0);

      virtual ~CEPuck2BatteryEquippedEntity();

      virtual void Init(TConfigurationNode& t_tree);

      virtual std::string GetTypeDescription() const {
         return "epuck2_battery";
      }

      virtual void Update();

      Real GetFullCharge() const {
         return m_fFullCharge;
      }

      void SetFullCharge(Real f_full_charge) {
         m_fFullCharge = f_full_charge;
      }

      Real GetAvailableCharge() const {
         return m_fAvailableCharge;
      }

      void SetAvailableCharge(Real f_available_charge) {
         m_fAvailableCharge = f_available_charge;
      }

      void SetDischargeModel(CEPuck2BatteryDischargeModel* pc_model);

      void SetDischargeModel(const std::string& str_model);

   protected:

      /** Full charge */
      Real m_fFullCharge;
      /** Available charge */
      Real m_fAvailableCharge;
      /** Discharge model */
      CEPuck2BatteryDischargeModel* m_pcDischargeModel;
   };

   /****************************************/
   /****************************************/
   
   /**
    * The discharge model dictates how the battery discharges over
    * time.
    */
   class CEPuck2BatteryDischargeModel : public CBaseConfigurableResource {
         
   public:

      CEPuck2BatteryDischargeModel();

      virtual ~CEPuck2BatteryDischargeModel();

      virtual void Init(TConfigurationNode& t_tree) {}

      virtual void UpdatePos(const Real f_charge) {}

      virtual void Reset() {}

      virtual void Destroy() {}

      virtual void SetBattery(CEPuck2BatteryEquippedEntity* pc_battery);
         
      virtual void operator()() = 0;
         
   protected:
      Real MAX_SPEED = 0.15;  // 15 cm/s
      CEPuck2BatteryEquippedEntity* m_pcBattery;
   };

   /****************************************/
   /****************************************/
   
   /**
    * For dynamic loading of battery discharge models.
    */
   typedef CFactory<CEPuck2BatteryDischargeModel> TFactoryBatteryDischargeModel;
   
#define REGISTER_BATTERY_DISCHARGE_MODEL(CLASSNAME, LABEL)  \
   REGISTER_SYMBOL(CEPuck2BatteryDischargeModel,                  \
                   CLASSNAME,                               \
                   LABEL,                                   \
                   "undefined",                             \
                   "undefined",                             \
                   "undefined",                             \
                   "undefined",                             \
                   "undefined")
   
   /**
    * A cubic battery discharge model
    */
   class CEPuck2BatteryDischargeModelCubic : public CEPuck2BatteryDischargeModel {
      
   public:
      
      CEPuck2BatteryDischargeModelCubic() :
         m_psAnchor(NULL) {}

      virtual void Init(TConfigurationNode& t_tree);
      
      virtual void SetBattery(CEPuck2BatteryEquippedEntity* pc_battery);
      
      virtual void operator()();

   protected:
      
      const SAnchor* m_psAnchor;
      CVector3 m_cOldPosition;
      CQuaternion m_cOldOrientation;
   private:
      Real CubicRoot(const Real a, const Real b, const Real c, const Real d, const Real y);
      // d, c, b, a, end
      Real Q[16][5] =
        {{   0.9898443199, -8.9712401631e-05, 7.8335314784e-09, -2.7403123453e-13, 19743.4806580544},
         {   0.9921311498, -1.1522707885e-04, 1.2495664269e-08, -5.4530754116e-13, 16021.8546357155},
         {   0.9566683695, -1.3697451496e-04, 1.8159817335e-08, -9.7844050612e-13, 12856.6157214642},
         {   0.9710580344, -1.6441170623e-04, 2.5064013018e-08, -1.5578008608e-12, 11142.8265268803},
         {   0.9373195222, -1.6723166189e-04, 2.7801039556e-08, -2.0162222851e-12,  9166.2482731342},
         {   0.9603929712, -1.9429015609e-04, 3.4602913169e-08, -2.5372952124e-12,  9355.3361966610},
         {   0.9371464445, -1.7462989849e-04, 3.1318946124e-08, -2.3915944571e-12,  9202.5480720997},
         {   0.9363534370, -1.6885512266e-04, 2.8063544748e-08, -2.0145865693e-12,  9729.1094522476},
         {   0.9329027675, -1.6575267405e-04, 2.6663267836e-08, -1.8147243149e-12,  9897.8873100281},
         {   0.9394276839, -1.5856934248e-04, 2.5039026198e-08, -1.6480947395e-12, 10356.6555407047},
         {   0.9625100937, -1.5231399936e-04, 2.2831535432e-08, -1.4209637836e-12, 11057.3435299397},
         {   0.9804099334, -1.5351601481e-04, 2.2316751643e-08, -1.3235128960e-12, 11638.0295445919},
         {   0.9830482173, -1.4493482562e-04, 1.9656989630e-08, -1.0667789832e-12, 12765.5281226635},
         {   0.9685180835, -1.2884120218e-04, 1.6680760997e-08, -8.6820889493e-13, 13055.4147162437},
         {   0.9849589465, -1.2507135152e-04, 1.5623460923e-08, -7.9006866533e-13, 13228.2937741280},
         {   0.9938042146, -1.2958254980e-04, 1.6227247309e-08, -8.1846159728e-13, 12957.4709944725}};
   };

   /****************************************/
   /****************************************/

   /**
    * An approximated battery discharge model
    */
   class CEPuck2BatteryDischargeModelApprox : public CEPuck2BatteryDischargeModel {

   public:

      CEPuck2BatteryDischargeModelApprox() :
         m_psAnchor(NULL),
         m_fDelta(1e-5),
         m_fPosFactor(1e-3) {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void SetBattery(CEPuck2BatteryEquippedEntity* pc_battery);

      virtual void operator()();

   protected:

      const SAnchor* m_psAnchor;
      CVector3 m_cOldPosition;
      CQuaternion m_cOldOrientation;
      Real m_fDelta;
      Real m_fPosFactor;
   private:
      // h, m, x1, x2, y1, y2
      Real M1[16][6] =
        {{   1.0000000000, -5.9393113079e-05,     0.0000000000,  5006.1026633397, 1.0000000000e+00, 7.0267197843e-01},
         {   1.0000000000, -7.5853230124e-05,     0.0000000000,  4009.6680848299, 1.0000000000e+00, 6.9585372404e-01},
         {   1.0000000000, -1.0127281327e-04,     0.0000000000,  3272.0703072119, 1.0000000000e+00, 6.6862823479e-01},
         {   1.0000000000, -1.1589613151e-04,     0.0000000000,  2844.5074985123, 1.0000000000e+00, 6.7033258488e-01},
         {   1.0000000000, -1.3860761294e-04,     0.0000000000,  2387.3441028849, 1.0000000000e+00, 6.6909593264e-01},
         {   1.0000000000, -1.4181124163e-04,     0.0000000000,  2418.8251315380, 1.0000000000e+00, 6.5698340481e-01},
         {   1.0000000000, -1.4482053885e-04,     0.0000000000,  2231.4038540513, 1.0000000000e+00, 6.7684689146e-01},
         {   1.0000000000, -1.3893825244e-04,     0.0000000000,  2423.4827063050, 1.0000000000e+00, 6.6328554796e-01},
         {   1.0000000000, -1.3523173726e-04,     0.0000000000,  2573.2859868901, 1.0000000000e+00, 6.5201006553e-01},
         {   1.0000000000, -1.2685319714e-04,     0.0000000000,  2641.7481001750, 1.0000000000e+00, 6.6488580745e-01},
         {   1.0000000000, -1.1319940586e-04,     0.0000000000,  2785.5797337205, 1.0000000000e+00, 6.8467402917e-01},
         {   1.0000000000, -1.0560702417e-04,     0.0000000000,  2963.9356636710, 1.0000000000e+00, 6.8698757472e-01},
         {   1.0000000000, -9.8032888621e-05,     0.0000000000,  3215.2700773228, 1.0000000000e+00, 6.8479778662e-01},
         {   1.0000000000, -9.2133681176e-05,     0.0000000000,  3347.6620350584, 1.0000000000e+00, 6.9156757338e-01},
         {   1.0000000000, -8.3355245508e-05,     0.0000000000,  3590.0066330425, 1.0000000000e+00, 7.0075411573e-01},
         {   1.0000000000, -8.2780973806e-05,     0.0000000000,  3666.2165957370, 1.0000000000e+00, 6.9650702002e-01}};

      Real M2[16][6] =
        {{   0.8061667761, -2.0673726568e-05,  5006.1026633397, 14051.4121779034, 7.0267197843e-01, 5.1567172280e-01},
         {   0.8039604405, -2.6961512570e-05,  4009.6680848299, 11266.9276285690, 6.9585372404e-01, 5.0018702962e-01},
         {   0.7764026015, -3.2937668387e-05,  3272.0703072119,  9101.2368220237, 6.6862823479e-01, 4.7662908115e-01},
         {   0.7837493173, -3.9872186139e-05,  2844.5074985123,  7881.7300026531, 6.7033258488e-01, 4.6948751157e-01},
         {   0.7867663845, -4.9289271607e-05,  2387.3441028849,  6805.1077948272, 6.6909593264e-01, 4.5134757813e-01},
         {   0.7742202305, -4.8468499929e-05,  2418.8251315380,  6672.9859696172, 6.5698340481e-01, 4.5079061053e-01},
         {   0.7857545750, -4.8806800848e-05,  2231.4038540513,  6498.8795574470, 6.7684689146e-01, 4.6856505468e-01},
         {   0.7807589353, -4.8472962908e-05,  2423.4827063050,  6863.3010523105, 6.6328554796e-01, 4.4807439795e-01},
         {   0.7677332197, -4.4970965054e-05,  2573.2859868901,  7221.8722887807, 6.5201006553e-01, 4.4295865339e-01},
         {   0.7743533718, -4.1437548241e-05,  2641.7481001750,  7486.7255439324, 6.6488580745e-01, 4.6412182089e-01},
         {   0.7944775403, -3.9418548966e-05,  2785.5797337205,  7926.1806906557, 6.8467402917e-01, 4.8203899864e-01},
         {   0.7979095475, -3.7423880075e-05,  2963.9356636710,  8277.2350669869, 6.8698757472e-01, 4.8814329504e-01},
         {   0.7919858099, -3.3337175643e-05,  3215.2700773228,  9069.0549419732, 6.8479778662e-01, 4.8964913241e-01},
         {   0.7924143125, -3.0124528118e-05,  3347.6620350584,  9460.8998167800, 6.9156757338e-01, 5.0740916993e-01},
         {   0.8056033786, -2.9205868846e-05,  3590.0066330425,  9593.2027162024, 7.0075411573e-01, 5.2542555827e-01},
         {   0.8043903729, -2.9426344591e-05,  3666.2165957370,  9551.4651000019, 6.9650702002e-01, 5.2332566953e-01}};

      Real M3[16][6] =
        {{   1.3857220723, -6.1919068242e-05, 14051.4121779034, 19742.4806580544, 5.1567172280e-01, 1.6328606520e-01},
         {   1.4225597408, -8.1865504203e-05, 11266.9276285690, 16020.8546357155, 5.0018702962e-01, 1.1100439824e-01},
         {   1.3455428473, -9.5472053206e-05,  9101.2368220237, 12855.6157214642, 4.7662908115e-01, 1.1819081910e-01},
         {   1.3724601335, -1.1456528220e-04,  7881.7300026531, 11141.8265268803, 4.6948751157e-01, 9.5993633282e-02},
         {   1.2116716607, -1.1172844068e-04,  6805.1077948272,  9165.2482731342, 4.5134757813e-01, 1.8765276271e-01},
         {   1.3388620456, -1.3308456501e-04,  6672.9859696172,  9354.3361966610, 4.5079061053e-01, 9.3944281916e-02},
         {   1.3097054558, -1.2942852590e-04,  6498.8795574470,  9201.5480720997, 4.6856505468e-01, 1.1876265282e-01},
         {   1.2943102531, -1.2329866470e-04,  6863.3010523105,  9728.1094522476, 4.4807439795e-01, 9.4847347628e-02},
         {   1.2475716919, -1.1141335741e-04,  7221.8722887807,  9896.8873100281, 4.4295865339e-01, 1.4492624875e-01},
         {   1.2779938627, -1.0870867871e-04,  7486.7255439324, 10355.6555407047, 4.6412182089e-01, 1.5224423162e-01},
         {   1.3254876030, -1.0641299225e-04,  7926.1806906557, 11056.3435299397, 4.8203899864e-01, 1.4894900469e-01},
         {   1.3695558588, -1.0648635161e-04,  8277.2350669869, 11637.0295445919, 4.8814329504e-01, 1.3037103896e-01},
         {   1.4037996078, -1.0079886838e-04,  9069.0549419732, 12764.5281226635, 4.8964913241e-01, 1.1714961769e-01},
         {   1.3226403147, -8.6168457609e-05,  9460.8998167800, 13054.4147162437, 5.0740916993e-01, 1.9776153365e-01},
         {   1.2902992141, -7.9730792568e-05,  9593.2027162024, 13227.2937741280, 5.2542555827e-01, 2.3567659796e-01},
         {   1.2653973738, -7.7691924383e-05,  9551.4651000019, 12956.4709944725, 5.2332566953e-01, 2.5878420905e-01}};

      Real M4[16][6] =
        {{3223.8352701046, -1.6328606520e-01, 19742.4806580544, 19743.4806580544, 1.6328606520e-01, 0.0000000000e+00},
         {1778.4963325223, -1.1100439824e-01, 16020.8546357155, 16021.8546357155, 1.1100439824e-01, 0.0000000000e+00},
         {1519.5339430364, -1.1819081910e-01, 12855.6157214642, 12856.6157214642, 1.1819081910e-01, 0.0000000000e+00},
         {1069.6404033429, -9.5993633282e-02, 11141.8265268803, 11142.8265268803, 9.5993633282e-02, 0.0000000000e+00},
         {1720.0718121583, -1.8765276271e-01,  9165.2482731342,  9166.2482731342, 1.8765276271e-01, 0.0000000000e+00},
         { 878.8803410819, -9.3944281916e-02,  9354.3361966610,  9355.3361966610, 9.3944281916e-02, 0.0000000000e+00},
         {1092.9190217837, -1.1876265282e-01,  9201.5480720997,  9202.5480720997, 1.1876265282e-01, 0.0000000000e+00},
         { 922.7802263286, -9.4847347628e-02,  9728.1094522476,  9729.1094522476, 9.4847347628e-02, 0.0000000000e+00},
         {1434.4636784271, -1.4492624875e-01,  9896.8873100281,  9897.8873100281, 1.4492624875e-01, 0.0000000000e+00},
         {1576.7410649871, -1.5224423162e-01, 10355.6555407047, 10356.6555407047, 1.5224423162e-01, 0.0000000000e+00},
         {1646.9803133392, -1.4894900469e-01, 11056.3435299397, 11057.3435299397, 1.4894900469e-01, 0.0000000000e+00},
         {1517.2620031275, -1.3037103896e-01, 11637.0295445919, 11638.0295445919, 1.3037103896e-01, 0.0000000000e+00},
         {1495.4767391284, -1.1714961769e-01, 12764.5281226635, 12765.5281226635, 1.1714961769e-01, 0.0000000000e+00},
         {2581.8588367383, -1.9776153365e-01, 13054.4147162437, 13055.4147162437, 1.9776153365e-01, 0.0000000000e+00},
         {3117.5992734402, -2.3567659796e-01, 13227.2937741280, 13228.2937741280, 2.3567659796e-01, 0.0000000000e+00},
         {3353.1888826374, -2.5878420905e-01, 12956.4709944725, 12957.4709944725, 2.5878420905e-01, 0.0000000000e+00}};

   };

   /****************************************/
   /****************************************/

   /**
    * A linear battery discharge model
    */
   class CEPuck2BatteryDischargeModelLinear : public CEPuck2BatteryDischargeModel {

   public:

      CEPuck2BatteryDischargeModelLinear() :
         m_psAnchor(NULL) {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void SetBattery(CEPuck2BatteryEquippedEntity* pc_battery);

      virtual void operator()();

   protected:

      const SAnchor* m_psAnchor;
      CVector3 m_cOldPosition;
      CQuaternion m_cOldOrientation;
   private:
      // (m)
      Real L[16] = {-5.064963049e-05, -6.241474678e-05, -7.778096675e-05, -8.974383632e-05,
                    -1.090958885e-04, -1.068908673e-04, -1.086655557e-04, -1.027843304e-04,
                    -1.010316615e-04, -9.655626723e-05, -9.043763516e-05, -8.592519861e-05,
                    -7.833596780e-05, -7.659657098e-05, -7.559553916e-05, -7.717555381e-05};
   };

   /****************************************/
   /****************************************/


   /**
    * A simplified battery discharge model
    */
   class CEPuck2BatteryDischargeModelSimple : public CEPuck2BatteryDischargeModel {

   public:

      CEPuck2BatteryDischargeModelSimple() :
         m_psAnchor(NULL) {}

      virtual void Init(TConfigurationNode& t_tree);

      virtual void SetBattery(CEPuck2BatteryEquippedEntity* pc_battery);

      virtual void operator()();

   private:

      const SAnchor* m_psAnchor;
      CVector3 m_cOldPosition;
      CQuaternion m_cOldOrientation;
      const Real M0 = -5.064963049e-05;
      const Real M1 = -8.699123633e-05;

   };

   /****************************************/
   /****************************************/

   
}

#endif
