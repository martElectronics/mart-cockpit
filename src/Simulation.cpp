#include "Vcu.h"

// Genera lecturas simuladas (APPS, freno, Vbat) y estados falsos del inversor por CAN,
// para probar el sistema sin hardware real. Da el SDC por presente para poder probar R2D.
void runSimulation() {
  stsSDC = true;        // En simulación damos el SDC por presente para poder probar el R2D.
  lastBMSMsg = millis();
  switch (simProfile) {
    case SIM_APPS_STEP:
      if (millis() - simStepT > 50) {
        simStepT = millis();
        simAppsProgress = min(simAppsProgress + 10, 1000);
      }
      rawApps1 = map(simAppsProgress, 0, 1000, 1100, 1900);
      rawApps2 = map(simAppsProgress, 0, 1000, 2300, 2050);
      stsBrake = 300; stsBrake2 = 300; stsVbatRAW = 360;
      { short fakeTempsOK[4] = {350, 280, 0, (short)UNUSED_SHORT};
        CAN.setPacket(id2StsInverter, fakeTempsOK, 4);
        byte fakeDrive[8] = { (byte)(simAppsProgress/10), 0, 0, 1, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, 23 };
        CAN.setPacket(id4StsInverter, fakeDrive, 8);
      }
      break;

    case SIM_FAULT_OVERVOLTAGE:
    case SIM_FAULT_UNDERVOLTAGE:
    case SIM_FAULT_CTRL_OVERTEMP:
    case SIM_FAULT_MOTOR_OVERTEMP: {
      uint8_t faultCode = 0;
      if (simProfile == SIM_FAULT_OVERVOLTAGE)       faultCode = 1;
      else if (simProfile == SIM_FAULT_UNDERVOLTAGE) faultCode = 2;
      else if (simProfile == SIM_FAULT_CTRL_OVERTEMP)faultCode = 5;
      else if (simProfile == SIM_FAULT_MOTOR_OVERTEMP)faultCode = 6;

      rawApps1 = 1600;
      rawApps2 = 2150;
      stsBrake = 300; stsBrake2 = 300; stsVbatRAW = 360;

      short fakeTempsFail[4] = {400, 350, (short)faultCode, (short)UNUSED_SHORT};
      CAN.setPacket(id2StsInverter, fakeTempsFail, 4);
      { byte fakeDrive[8] = {50, 0, 0, 0, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, 23};
        CAN.setPacket(id4StsInverter, fakeDrive, 8);
      }
      break;
    }

    case SIM_RANDOM:
    default:
      rawApps1 = random(1000, 2000);
      rawApps2 = random(1000, 2000);
      stsBrake   = random(200, 800);
      stsBrake2  = random(200, 800);
      stsVbatRAW = random(300, 400);
      { short fakeTempsRnd[4] = {350, 280, 0, (short)UNUSED_SHORT};
        CAN.setPacket(id2StsInverter, fakeTempsRnd, 4);
        byte fakeDrive[8] = { (byte)random(0, 100), 0, 0, 1, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, 23 };
        CAN.setPacket(id4StsInverter, fakeDrive, 8);
      }
      break;
  }
}
