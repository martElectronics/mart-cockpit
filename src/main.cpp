#include <Arduino.h>
#include <MART_CAN.h>
#include <SPI.h>
#include "global.h"
#include <Mcp320x.h>
#include <Adafruit_NeoPixel.h>

//**LED RGB */
#define LED_PIN 48
#define LED_COUNT 1

//**MCP3208 */
#define SPI_CS 10      // SPI slave select
#define ADC_VREF 3300  // 3.3V Vref
#define ADC_CLK 160000 // SPI clock 1.6MHz //estaba a 1600000

CAN_BUS CAN(HardwareType::Transciever, 125, 1);
MCP3208 adc(ADC_VREF, SPI_CS);
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// PINES

int pinTSON = 21, pinStart = 15, pinBUZZ = 8, pinTSON_EXT = 9, pinSDC = 16;

//**GLOBAL CONTROL */
bool cfgCtrlBySpeed = false;
bool cfgCtrlBySerial = false;
bool cfgCtrlByPctg = true;
//*******CAN

int configsGEN[12];
int idConfigsDEF[12];
int numConfigDefs;

uint32_t idMens, idR2D = 100;
uint32_t idPacket, idNode = 1;
int data;
bool enableInverter;
int RPMtarget, cfgRPMax = 1500, currentTarget;

uint32_t idCmdRPM = combineInts(3, idNode);              // 97
uint32_t idCmdEN = combineInts(12, idNode);              // 385
uint32_t idCmdCurrent = combineInts(1, idNode);          // ??
uint32_t idCmdCurrentPCTG = combineInts(5, idNode);      // ??
uint32_t idCmdSetMaxACCurrent = combineInts(8, idNode);  // ??
uint32_t idCmdSetMaxDCCurrent = combineInts(10, idNode); // ??
// Packet ID 0x20: ERPM, Duty, Input Voltage
uint32_t id0StsInverter = combineInts(0x20, idNode);

// Packet ID 0x21: AC Current, DC Current
uint32_t id1StsInverter = combineInts(0x21, idNode);

// Packet ID 0x22: Controller Temp., Motor Temp., Fault code
uint32_t id2StsInverter = combineInts(0x22, idNode);

// Packet ID 0x23: Id, Iq values
uint32_t id3StsInverter = combineInts(0x23, idNode);

// Packet ID 0x24: Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits, CAN map version
uint32_t id4StsInverter = combineInts(0x24, idNode);
// APPS
struct sensorData apps1Data, apps2Data;

// DATA
int32_t cmdDataRPM[2];
int16_t cmdDataCurrent[4], cmdDataCurrentACMax[4], cmdDataCurrentDCMax[4];
byte cmdDataDriveEN[8];
int globalPCTG;
//*CONFIG**/

// Write separated by commas the packet id to send periodically to the inverter
char packetIDStoConfig[] = ""; //= "8,9,10,11";

int cfgCurrent = 0,           // 1
    cfgCurrentBrake = 0,      // 2
    cfgERPM = 0,              // 3
    cfgPos = 0,               // 4
    cfgCurrentRel,            // 5
    cfgCurrentRelBrake,       // 6
    cfgDO,                    // 7
    cfgCurrentACMAX = 190,    // 8
    cfgCurrentACBrakeMAX = 0, // 9
    cfgCurrentDCMAX = 60,     // 10
    cfgCurrentDCBrakeMAX = 0, // 11
    cfgDriveEnable = 0;       // 12

int timeUpdateConfig = 5000, timeUpdateCTRL = 0;

void configInverterLimits();                                            // Sets CAN packet timers for configuring the inverter's limits
int getFilteredAnalogRead(int, int sampleSize);                         // Does sliding window average on a given input
bool R2D(bool tson, bool start, bool brake);                            // Returns de R2D state
int apps(int valAPPS1, int valAPPS2, int difMAX, int max, int valDesc); // Returns the APPS implausability state
void debug();                                                           // Shows debug info
void debugShowADC();                                                    // Shows debug ADC info
void readInverterStatus();                                              // Reads inverter info
uint32_t combineInts(uint32_t int1, uint32_t int2);                     // Calculates the inverter CAN packets IDs
void fillConfigsArray();                                                // Makes a copy of the configuration
void parseConfigIds(char *input);                                       // Reads the IDs set by the user to send periodacally
void controlInverter();
void processSerialCommand(int *p_var, int *c_var, int *md_var, int *ma_var, bool *mode, bool enable);

// PRG
bool stsTSON, stsStart, stsR2D, stsAPPS, stsSDC;
int stsBrake, stsBrake2, stsPot, stsSDCAnalog, stsVbatRAW;
int cfgBrakeTH = 550, cfgChannelAPPS1 = 0, cfgChannelAPPS2 = 1, cfgChannelBrake = 2, cfgChannelSteering = 3;
int cfgAPPSdiff = 10, cfgAPPSdesc = 5000, cfgAPPSdescScaled = 200, cfgAPPSmax = 0, cfgTimeSoundR2D = 2000;
float cfgAPPSMargin = 0.1; // (0.1=10%)
int cfgScaleFactor = 1000; // Aumenta resolución al hacer map(), luego se divide por el mismo factor en la consigna final
uint32_t tA = millis();

// CAN DATA

unsigned long int idBMSStatus = 10;
unsigned long int idBMSMaxValues = 11;
unsigned long int idAPPSState = 1163;
unsigned long int idBrakeState = 1164;
unsigned long int idVCUSignals = 1166;

byte canDataBMSStatus[8];
uint16_t CANAppsState[4];
uint16_t CANBrakeState[4];
uint8_t CANVCUSignals[8];

const int canSendingPeriod=100;

void configureAPPS()
{
  apps1Data.valAnalogUP = 1055;   // 2275  
  apps1Data.valAnalogDOWN = 1935; // 1850
  apps2Data.valAnalogUP = 2290;   // 790
  apps2Data.valAnalogDOWN = 2068; // 1670
  apps1Data.valScaledUP = apps2Data.valScaledUP = 0;
  apps1Data.valScaledDOWN = apps2Data.valScaledDOWN = 1000;
  apps1Data.range = abs(apps1Data.valAnalogUP - apps1Data.valAnalogDOWN);
  apps2Data.range = abs(apps2Data.valAnalogUP - apps2Data.valAnalogDOWN);
  // brake 422 1350
}

void setup()
{

  Serial.begin(115200);
  // PINES
  pinMode(pinStart, INPUT_PULLUP);
  pinMode(pinTSON, INPUT);
  pinMode(pinTSON_EXT, INPUT);
  pinMode(pinBUZZ, OUTPUT);
  pinMode(pinSDC, INPUT);

  // INIT
  parseConfigIds(packetIDStoConfig);
  fillConfigsArray();
  // configInverterLimits();
  configureAPPS();

  for (int i = 1; i < 8; i++)
  {
    cmdDataDriveEN[i] = 255;
  }
  cmdDataRPM[1] = 0xFFFFFFFF;
  for (int i = 1; i < 4; i++)
  {
    cmdDataCurrent[i] = 0xFFFF;
    cmdDataCurrentACMax[i] = 0xFFFF;
    cmdDataCurrentDCMax[i] = 0xFFFF;
  }

  //***MCP3208 */

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);

  // initialize SPI interface for MCP3208
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  pixels.clear(); // Set all pixel colors to 'off'

  //***CAN 
  CAN.setPacketTimer(idCmdEN,canSendingPeriod);
   CAN.setPacketTimer(idCmdCurrentPCTG,canSendingPeriod);
    CAN.setPacketTimer(idCmdCurrent,canSendingPeriod);
    CAN.setPacketTimer(idCmdSetMaxACCurrent,canSendingPeriod);
     CAN.setPacketTimer(idCmdSetMaxDCCurrent,canSendingPeriod);

  // CAN.setPacketTimer(idCmdEN,50);
}

void loop()
{

  controlInverter();
  // CAN.getCANStatusData();
  // readInverterStatus();
  debug();
}

void controlInverter()
{
  tA = millis();
  //****SERIAL
  static int serialP = 0;
  static int serialC = 0;
  static bool serialEnableContol = false;

  //****LECTURA GPIO
  stsStart = !digitalRead(pinStart);
  stsSDCAnalog = adc.read(MCP3208::Channel::SINGLE_1);
  apps1Data.valAnalog = adc.read(MCP3208::Channel::SINGLE_2);
  apps2Data.valAnalog = adc.read(MCP3208::Channel::SINGLE_3);
  stsBrake = adc.read(MCP3208::Channel::SINGLE_4);
  stsBrake2 = adc.read(MCP3208::Channel::SINGLE_5);
  stsVbatRAW = adc.read(MCP3208::Channel::SINGLE_6);

  //****LECTURA CAN
  CAN.receive();
  CAN.getPacket(idBMSStatus, canDataBMSStatus, 8);

  //****Procesamiento

  apps1Data.valScaled =
      map(apps1Data.valAnalog, apps1Data.valAnalogUP, apps1Data.valAnalogDOWN, apps1Data.valScaledUP, apps1Data.valScaledDOWN);

  apps2Data.valScaled =
      map(apps2Data.valAnalog, apps2Data.valAnalogUP, apps2Data.valAnalogDOWN, apps2Data.valScaledUP, apps2Data.valScaledDOWN);

  // Constrain limits
  if (apps1Data.valScaled > apps1Data.valScaledUP)
    apps1Data.valScaled = apps1Data.valScaledUP;
  if (apps1Data.valScaled < apps1Data.valScaledDOWN)
    apps1Data.valScaled = apps1Data.valScaledDOWN;
  if (apps2Data.valScaled > apps2Data.valScaledUP)
    apps2Data.valScaled = apps2Data.valScaledUP;
  if (apps2Data.valScaled < apps2Data.valScaledDOWN)
    apps2Data.valScaled = apps2Data.valScaledDOWN;

  stsAPPS = apps(apps1Data.valScaled, apps2Data.valScaled, 100, 0, 100);

  // *R2D

  stsR2D = R2D(stsSDCAnalog>3000, stsStart, (stsBrake2 >= cfgBrakeTH));

//****MOTOR CONTROL LOGIC
  if (stsR2D && stsAPPS == 0)
  {
    cfgDriveEnable = 1;
    if (!cfgCtrlBySerial)
    {
      currentTarget = apps2Data.valScaled;
      // cfgCurrentACMAX = 180;
      // cfgCurrentDCMAX = 15;
    }

    RPMtarget = map(apps1Data.valScaled, apps1Data.valScaledDOWN, apps1Data.valScaledUP, 0, cfgRPMax * 10);

    cmdDataDriveEN[0] = (byte)cfgDriveEnable;
    cmdDataRPM[0] = RPMtarget;
    cmdDataCurrent[0] = currentTarget;
    cmdDataCurrentACMax[0] = cfgCurrentACMAX * 10;
    cmdDataCurrentDCMax[0] = cfgCurrentDCMAX * 10;

    if (cfgCtrlBySpeed)
    {
      // CAN.setPacket(idCmdRPM, cmdDataRPM, 2);
    }

    //Control por corriente
    else
    {
      //Control en función de la corriente máxima establecida
      if (cfgCtrlByPctg)
      {
        CAN.setPacket(idCmdCurrentPCTG, cmdDataCurrent, 2);
        CAN.DataOUT.removePacket(idCmdCurrent);
      }
      //Control corriente absoluto
      else
      {
        CAN.setPacket(idCmdCurrent, cmdDataCurrent, 2);
        CAN.DataOUT.removePacket(idCmdCurrentPCTG);
      }
      CAN.setPacket(idCmdSetMaxACCurrent, cmdDataCurrentACMax, 4);
      CAN.setPacket(idCmdSetMaxDCCurrent, cmdDataCurrentDCMax, 4);
    }
    CAN.setPacket(idCmdEN, cmdDataDriveEN, 1);
  }
  else
  {
    
    CAN.DataOUT.removePacket(idCmdEN);
    CAN.DataOUT.removePacket(idCmdCurrentPCTG);
    CAN.DataOUT.removePacket(idCmdCurrent);
    CAN.DataOUT.removePacket(idCmdSetMaxACCurrent);
    CAN.DataOUT.removePacket(idCmdSetMaxDCCurrent);
  }

  // ******WRITE CAN DATA

  

  //processSerialCommand(&currentTarget, &currentTarget, &cfgCurrentDCMAX, &cfgCurrentACMAX, &cfgCtrlByPctg, cfgCtrlBySerial);

  CANAppsState[0] = apps1Data.valScaled;
  CANAppsState[1] = apps2Data.valScaled;
  CANAppsState[2] = apps1Data.valAnalog;
  CANAppsState[3] = apps2Data.valAnalog;
  CANBrakeState[0] = CANBrakeState[1] = 777;
  CANBrakeState[2] = stsBrake;
  CANBrakeState[3] = stsBrake2;
  CANVCUSignals[0] = stsVbatRAW;

  CAN.setPacket(idAPPSState, CANAppsState, 4);
  CAN.setPacket(idBrakeState, CANBrakeState, 4);
  CAN.setPacket(idVCUSignals, CANVCUSignals, 8);

  CAN.send();
}

void readInverterStatus()
{
  static uint64_t tAux = millis();
  CAN.getPacket(id2StsInverter, stsInverterCAN_22_FULL, 8); // ID status = 1217 (decimal) 0x4C1 (hex) //REVISAR

  stsInverterCAN_FaultCode = stsInverterCAN_22_FULL[4];
  CAN.getPacket(id4StsInverter, stsInverterCAN_24_FULL, 8);
  stsInverterCAN_DriveEnable = stsInverterCAN_24_FULL[3];

  if ((millis() - tAux) >= 1000)
  {
    CAN.printByteArray(stsInverterCAN_22_FULL, 8);
    Serial.println(id2StsInverter);
    Serial.println(id4StsInverter);
    Serial.println(getErrorMessage(stsInverterCAN_FaultCode));
    if (stsInverterCAN_DriveEnable == 1)
    {
      Serial.println("Drive enable OK");
    }
    else if (stsInverterCAN_DriveEnable == 0)
    {
      Serial.println("Drive enable FAIL");
    }
    else
    {
      Serial.println("ERROR COMM");
    }
    if (stsInverterCAN_FaultCode != 0)
    {
      pixels.setPixelColor(0, pixels.Color(255, 0, 0));
      pixels.show(); // Send the updated pixel colors to the hardware.
    }
    else
    {
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show(); // Send the updated pixel colors to the hardware.
    }

    Serial.println();
    tAux = millis();
  }
}

void debug()
{
  static uint64_t tAux = millis();

  //  Serial.println((String) "APPS1: " + apps1Data.valScaled + " APPS2: " + apps2Data.valScaled);
  // Serial.println((String) "Brake: " + stsBrake + " TSON: " + stsTSON + " Start: " + stsStart + " R2D: " + stsR2D);
  // Serial.println((String) "Drive Enable: " + cmdDataDriveEN[0] + " Current: " + cmdDataCurrent[0] + " EERPM: " + cmdDataRPM[0]);

  // Serial.println((String)apps1Data.valAnalog + " valScaled: " + apps1Data.valScaled + " RPM: " + RPMtarget + " Current pctg: " + currentTarget);
  // Serial.println();
  // delay(500);

  if ((millis() - tAux) >= 100)
  {
    // Serial.println((String) "APPS1: " + apps1Data.valScaled + " APPS2: " + apps2Data.valScaled);
    // Serial.println((String) "Current=" + cmdDataCurrent[0]);
    // Serial.println((String) "APPS1 analog: " + apps1Data.valAnalog + " APPS2: " + apps2Data.valAnalog + " Brake= " + stsBrake + " Brake2= " + stsBrake2);
    // Serial.println((String) "Start = " + stsStart);
    // Serial.println((String) "R2D = " + stsR2D);
    // // Serial.println((String) "TSON = " + stsTSON);
    // // Serial.println((String) "SDC = " + stsSDCAnalog);
    // Serial.println((String) "Control by serial = " + cfgCtrlBySerial);
    // Serial.println((String) "Control by PCTG = " + cfgCtrlByPctg);
    // Serial.println((String) "MAX DC = " + cfgCurrentDCMAX+ " MAX AC = " + cfgCurrentACMAX);
    // Serial.println();
    // // CAN.printReceivedIds();
    // tAux = millis();
    // Serial.println((String) "ID= " + idCmdCurrentPCTG);

    // Print a header for each block of readings
    Serial.println("--- Sensor Readings ---");

    // Print Start Button status
    Serial.print("Start Button (stsStart):       ");
    Serial.println(stsStart); // Prints 1 (pressed) or 0 (not pressed)

    // Print SDC Analog value
    Serial.print("SDC Analog (stsSDCAnalog):   ");
    Serial.println(stsSDCAnalog); // Prints 0-4095

    // Print APPS1 value
    Serial.print("APPS 1 (apps1Data.valAnalog):  ");
    Serial.println(apps1Data.valAnalog); // Prints 0-4095

    // Print APPS2 value
    Serial.print("APPS 2 (apps2Data.valAnalog):  ");
    Serial.println(apps2Data.valAnalog); // Prints 0-4095

    // Print Brake 1 value
    Serial.print("Brake 1 (stsBrake):          ");
    Serial.println(stsBrake); // Prints 0-4095

    // Print Brake 2 value
    Serial.print("Brake 2 (stsBrake2):         ");
    Serial.println(stsBrake2); // Prints 0-4095

    // Print Raw Battery Voltage value
    Serial.print("V-Battery RAW (stsVbatRAW):  ");
    Serial.println(stsVbatRAW);

    // Add a separator for readability
    Serial.println("-------------------------");

    // Add a delay so the serial monitor is readable and not flooded
    tAux = millis();
  }
}

bool R2D(bool tson, bool start, bool brake)
{
  static int step = 0;
  static uint32_t tAux = millis();
  bool r2d = false;
  // BUZZER
  if ((millis() - tAux) >= cfgTimeSoundR2D)
  {
    digitalWrite(pinBUZZ, false);
  }
  switch (step)
  {
  case 0:
    if (tson)
    {
      step += 10;
    }
    break;
  case 10:
    if (!tson)
    {
      step = 0;
    }
    else if (start && brake)
    {
      tAux = millis();
      digitalWrite(pinBUZZ, true);
      step += 10;
    }
    break;
  case 20:
    if (!tson)
    {
      step = 0;
    }
    r2d = true;
    break;

  default:
    break;
  }
  return r2d;
}

int apps(int valAPPS1, int valAPPS2, int difMAX, int max, int valDesc)
{
  static unsigned long int t = millis();
  int val = abs(valAPPS1 - valAPPS2);
  bool apps1ok= (apps1Data.valAnalog > 700) && (apps1Data.valAnalog < 2300);
  bool apps2ok= (apps1Data.valAnalog > 1700) && (apps1Data.valAnalog < 2700);

  if(apps1ok && apps2ok)
  {
    return 0;
  }
  else{
    return -1;
  }
}

void configInverterLimits()
{

  for (int i = 0; i < numConfigDefs + 1; i++)
  {

    idPacket = idConfigsDEF[i];
    idMens = combineInts(idPacket, idNode);
    Serial.print(idPacket);
    Serial.print("-");
    Serial.println(idMens);

    data = configsGEN[idPacket - 1];

    Serial.println(data);

    if (idPacket == 1 || idPacket == 3 || idPacket == 12)
    {
      if (timeUpdateCTRL > 0)
      {

        CAN.setPacketTimer(idMens, timeUpdateCTRL);
      }
    }
    else
    {
      short dataCurrent[4];
      for (int i = 0; i < 4; i++)
      {
        dataCurrent[i] = 0xFFFF;
      }
      dataCurrent[0] = data * 10;
      CAN.setPacket(idMens, dataCurrent, 4);
      CAN.setPacketTimer(idMens, timeUpdateConfig);
    }
    Serial.println();
  }
}

void fillConfigsArray()
{
  configsGEN[0] = cfgCurrent;
  configsGEN[1] = cfgCurrentBrake;
  configsGEN[2] = cfgERPM;
  configsGEN[3] = cfgPos;
  configsGEN[4] = cfgCurrentRel;
  configsGEN[5] = cfgCurrentRelBrake;
  configsGEN[6] = cfgDO;
  configsGEN[7] = cfgCurrentACMAX;
  configsGEN[8] = cfgCurrentACBrakeMAX;
  configsGEN[9] = cfgCurrentDCMAX;
  configsGEN[10] = cfgCurrentDCBrakeMAX;
  configsGEN[11] = cfgDriveEnable;
}

void parseConfigIds(char *input)
{
  int index = 0;                 // Index for idConfigsDEF array
  int currentNumber = 0;         // Temporary variable to store the current number being processed
  bool numberInProgress = false; // Flag to check if a number is being processed

  // Initialize the idConfigsDEF array with zeros
  for (int i = 0; i < 12; i++)
  {
    idConfigsDEF[i] = 0;
  }

  // Iterate over each character in the input char array
  for (int i = 0; input[i] != '\0'; i++)
  {
    if (input[i] >= '0' && input[i] <= '9')
    {
      // If the character is a digit, update the current number being processed
      currentNumber = currentNumber * 10 + (input[i] - '0');
      numberInProgress = true;
    }
    else if (input[i] == ',' || input[i] == ' ')
    {
      // If the character is a comma or space, store the current number in the array
      if (numberInProgress && index < 12)
      {
        idConfigsDEF[index++] = currentNumber;
        currentNumber = 0;
        numberInProgress = false;
        numConfigDefs++;
      }
    }
  }

  // Store the last number if the input does not end with a comma
  if (numberInProgress && index < 12)
  {
    idConfigsDEF[index] = currentNumber;
  }
}

void processSerialCommand(int *p_var, int *c_var, int *md_var, int *ma_var, bool *mode, bool enable)
{
  // Proceed only if there is data available to read in the serial buffer.
  if (Serial.available() > 0)
  {

    // Read the command part of the input as a String until a space is found.
    String command = Serial.readStringUntil(' ');
    command.trim(); // Clean up any whitespace

    // Only proceed if a command was actually read.
    if (command.length() > 0)
    {
      // Serial.parseInt() conveniently skips any non-numeric characters (like the space)
      // and reads the next valid integer from the stream.
      // NOTE: This is a blocking function with a 1-second default timeout.
      long receivedValue = Serial.parseInt();

      // Use an if-else if structure to check the received command string.
      if (command == "p")
      {
        if (enable)
        {
          *p_var = receivedValue;
          Serial.print("OK: Set 'p' variable to ");
          Serial.println(*p_var);
          *mode = true;
        }
        else
        {
          Serial.println("**********************Contorl by serial Not allowed*******");
        }
      }
      else if (command == "c")
      {
        if (enable)
        {

          *c_var = receivedValue;
          Serial.print("OK: Set 'c' variable to ");
          Serial.println(*c_var);
          *mode = false;
        }
        else
        {
          Serial.println("**********************Contorl by serial Not allowed*******");
        }
      }
      else if (command == "md")
      {
        *md_var = receivedValue;
        Serial.print("OK: Set 'md' variable to ");
        Serial.println(*md_var);
      }
      else if (command == "ma")
      {
        *ma_var = receivedValue;
        Serial.print("OK: Set 'ma' variable to ");
        Serial.println(*ma_var);
      }
      else
      {
        // Handle cases where the command is not recognized.
        Serial.print("Error: Unknown command '");
        Serial.print(command);
        Serial.println("'.");
      }
    }

    // Clear any remaining data from the input buffer for this line to
    // avoid processing leftover characters on the next loop iteration.
    while (Serial.available() > 0)
    {
      Serial.read();
    }
  }
}