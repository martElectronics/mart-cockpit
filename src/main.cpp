#include <Arduino.h>
#include <MART_CAN.h>
#include <Adafruit_ADS1X15.h>
#include "global.h"
CAN_BUS CAN(5, 2);    // 21 18 23 19 5 MCP2515
Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

// PINES

int idPotDSP = 10, idENDDSP = 11;
int pinTSON = 34, pinStart = 17, pinBUZZ = 25;

//**GLOBAL CONTROL */
bool ctrlBYPOT = true, ctrlByR2D = false, ctrlBYDSP = false, ctrlByExternalADC = true;
bool cfgCtrlBySpeed = false;

int appsGlobalValue;

int16_t adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;

//*******CAN

byte info[8];
int configsGEN[12];
int idConfigsDEF[12];
int dataR2D[1];
int numConfigDefs;

unsigned long int idMens, idR2D = 100;
unsigned long int idPacket, idNode = 1;
int data;
bool enableInverter;
int RPMtarget, cfgRMPMax = 8000, currentTarget;

unsigned long int idCmdRPM = combineInts(3, idNode);     // 97
unsigned long int idCmdEN = combineInts(12, idNode);     // 385
unsigned long int idCmdCurrent = combineInts(1, idNode); // ??
// Packet ID 0x20: ERPM, Duty, Input Voltage
unsigned long int id0StsInverter = combineInts(36, idNode);

// Packet ID 0x21: AC Current, DC Current
unsigned long int id1StsInverter = combineInts(37, idNode);

// Packet ID 0x22: Controller Temp., Motor Temp., Fault code
unsigned long int id2StsInverter = combineInts(38, idNode);

// Packet ID 0x23: Id, Iq values
unsigned long int id3StsInverter = combineInts(39, idNode);

// Packet ID 0x24: Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits, CAN map version
unsigned long int id4StsInverter = combineInts(40, idNode);
// APPS
struct sensorData apps1Data, apps2Data;

// DATA
int cmdDataRPM[2];
short cmdDataCurrent[4];
byte cmdDataDriveEN[8];

//*CONFIG**/

char packetIDStoConfig[] = "";

int cfgCurrent = 10,          // 1
    cfgCurrentBrake = 10,     // 2
    cfgERPM = 0,              // 3
    cfgPos = 0,               // 4
    cfgCurrentRel,            // 5
    cfgCurrentRelBrake,       // 6
    cfgDO,                    // 7
    cfgCurrentACMAX = 100,     // 8
    cfgCurrentACBrakeMAX = 0, // 9
    cfgCurrentDCMAX = 100,     // 10
    cfgCurrentDCBrakeMAX = 0, // 11
    cfgDriveEnable = 0;       // 12

int timeUpdateConfig = 100, timeUpdateCTRL = 25;

void intToByteArray(int a, byte *byteArray);
unsigned long int combineInts(uint32_t int1, uint32_t int2);
void printHex(unsigned long int num);
void fillConfigsArray();
void parseConfigIds(char *input);
void configInverter();
void controlInverter();
int getFilteredAnalogRead(int, int sampleSize);
int getFilteredAnalogRead2(int val, int sampleSize);
bool R2D(bool tson, bool start, bool brake);
int apps(int valAPPS1, int valAPPS2, int difMAX, int max, int valDesc);
void debug();
void readInverterStatus();

bool flagBot;

// PRG
int stepindexGlobal = 0;
bool stsTSON, stsStart, stsR2D, stsAPPS;
int stsBrake;
int cfgBrakeTH=3300, cfgChannelAPPS1 = 0, cfgChannelAPPS2 = 1, cfgChannelBrake = 2, cfgChannelSteering = 3;
int cfgAPPSdiff = 10, cfgAPPSdesc = 5000, cfgAPPSdescScaled = 200, cfgAPPSmax = 0, cfgTimeSoundR2D = 1100;
float cfgAPPSMargin = 0.05;
unsigned long int tA = millis();

void configureAPPS()
{
  apps1Data.valAnalogUP = 12270;
  apps1Data.valAnalogDOWN = 11050;
  apps2Data.valAnalogUP = 12320;
  apps2Data.valAnalogDOWN = 10920;
  apps1Data.valScaledUP = apps2Data.valScaledUP = 100;
  apps1Data.range = apps2Data.range = 1500;
  apps1Data.valScaledDOWN = apps2Data.valScaledDOWN = 0;
}

void setup()
{

  Serial.begin(115200);
  // PINES
  pinMode(pinStart, INPUT);
  pinMode(pinTSON, INPUT);
  pinMode(pinBUZZ, OUTPUT);
  //pinMode(pinBUZZ, INPUT);
  //

  parseConfigIds(packetIDStoConfig);
  fillConfigsArray();

  // INIT

  for (int i = 1; i < 8; i++)
  {
    cmdDataDriveEN[i] = 255;
  }
  cmdDataRPM[1] = 0xFFFFFFFF;
  for (int i = 1; i < 4; i++)
  {
    cmdDataCurrent[i] = 0xFFFF;
  }

  configInverter();
  configureAPPS();

  if (!ads.begin())
  {

    Serial.println("Failed to initialize ADS.");
    while (1)
      ;
  }
//ads.setDataRate(RATE_ADS1115_860SPS);
}

void loop()
{
  tA = millis();

  //****LECTURA
  CAN.receive();
  stsTSON = digitalRead(pinTSON); //cambiado por logica
  stsStart = !digitalRead(pinStart);
  
  // stsBrake=ads.readADC_SingleEnded(cfgChannelSteering);
  //stsBrake = cfgBrakeTH ;
  apps1Data.valAnalog = ads.readADC_SingleEnded(0);
  apps2Data.valAnalog = ads.readADC_SingleEnded(1);
  stsBrake = ads.readADC_SingleEnded(3);
       
  //****PROCESAMIENTO
  apps1Data.valScaled =
      map(apps1Data.valAnalog, apps1Data.valAnalogUP - apps1Data.range * cfgAPPSMargin, apps1Data.valAnalogDOWN + apps1Data.range * cfgAPPSMargin, apps1Data.valScaledDOWN, apps1Data.valScaledUP);

  apps2Data.valScaled =
      map(apps2Data.valAnalog, apps2Data.valAnalogUP - apps2Data.range * cfgAPPSMargin, apps2Data.valAnalogDOWN + apps2Data.range * cfgAPPSMargin, apps2Data.valScaledDOWN, apps2Data.valScaledUP);

  stsR2D = R2D(stsTSON, stsStart, (stsBrake >= cfgBrakeTH));
  stsAPPS = apps(apps1Data.valScaled, apps2Data.valScaled, cfgAPPSdiff, cfgAPPSmax, cfgAPPSdescScaled);

  //*****DESCOMENTAR
 // stsR2D=true;
  // VELOCIDAD
  if (!stsR2D || stsAPPS != 0)
  {
    RPMtarget = 0;
    currentTarget=0;
  }
  else if (stsR2D && stsAPPS == 0)
  {
    RPMtarget = map(apps1Data.valScaled, apps1Data.valScaledDOWN, apps1Data.valScaledUP, 0, cfgRMPMax);
    currentTarget = map(apps1Data.valScaled, apps1Data.valScaledDOWN, apps1Data.valScaledUP, 0, cfgCurrentACMAX*1000);
    if (RPMtarget < 0)
      RPMtarget = 0;
    if (currentTarget < 0)
      currentTarget = 0;
  }
  cfgERPM = 10 * RPMtarget;

  // DRIVE ENABLE
  
  if (!stsR2D)
  {
    cfgDriveEnable = 0;
    cfgERPM = 0;
  }
  else
  {
    cfgDriveEnable = 1;
  }

  cmdDataDriveEN[0] = cfgDriveEnable;
  cmdDataRPM[0] =RPMtarget;
  cmdDataCurrent[0] = currentTarget/100;

  // ******WRITE

  if (cfgCtrlBySpeed)
  {

    CAN.setPacket(idCmdRPM, cmdDataRPM);
  }
  else
  {

    CAN.setPacket(idCmdCurrent, cmdDataCurrent);
  }

  //Motor current setpoint control via MART-CAN library
  unsigned long int currentSetpointID=0x0122;
  byte currentSetpointData[8]={0x00,0x64,0x00,0x00,0x00,0x00,0x00,0x00};
  CAN.setPacket(currentSetpointID,currentSetpointData);
  CAN.send();


  CAN.setPacket(idCmdEN, cmdDataDriveEN);
 // readInverterStatus();
  CAN.send();
 //Serial.println(RPMtarget);
  // ads.st

 //Serial.println(cmdDataRPM[0]);
   
Serial.println((String)cmdDataCurrent[0]+" "+ stsBrake+" "+stsTSON+" "+stsStart+" rd2="+stsR2D);
//Serial.println((String)apps1Data.valAnalog+" "+apps2Data.valAnalog);
//Serial.println(cmdDataCurrent[0]);
  debug();
  // delay(50);
}
void readInverterStatus()
{

  CAN.getPacket(id2StsInverter, stsInverterCAN_22_FULL);

  stsInverterCAN_FaultCode = stsInverterCAN_22_FULL[4];
  CAN.getPacket(id4StsInverter, stsInverterCAN_24_FULL);
  stsInverterCAN_DriveEnable = stsInverterCAN_24_FULL[3];
  Serial.println(getErrorMessage(stsInverterCAN_FaultCode));
  if (stsInverterCAN_DriveEnable == 1)
  {
    Serial.println("Drive enable OK");
  }
  else if (stsInverterCAN_DriveEnable == 1)
  {
    Serial.println("Drive enable FAIL");
  }
  else
  {
    Serial.println("ERROR COMM");
  }
}

void debug()
{
  // if (stsR2D)
  // {
  //   Serial.println("R2D");
  // }
  // Serial.println((String)"T= "+(millis()-tA));
  // Serial.println((String)"Tson= "+stsTSON+" start= "+stsStart);
}

void globalPRG()
{
  switch (stepindexGlobal)
  {
  case 0:
    if (true)
      ;
    break;

  default:
    break;
  }
}

void debugShowADC()
{
  // Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: ");
  Serial.print(adc0);
  Serial.print("  ");
  Serial.print(volts0);
  Serial.println("V");
  Serial.print("AIN1: ");
  Serial.print(adc1);
  Serial.print("  ");
  Serial.print(volts1);
  Serial.println("V");
  // Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  Serial.print("AIN3: ");
  Serial.print(adc3);
  Serial.print("  ");
  Serial.print(volts3);
  Serial.println("V");
}

void debugCAN()
{
  Serial.println((String)cfgERPM + " " + enableInverter);
  Serial.println(dataR2D[0]);

  //  Serial.println();
  // Serial.println(numConfigDefs);

  // Serial.println();

  // for (int i = 0; i < numConfigDefs + 1; i++)
  // {
  //   Serial.println(idConfigsDEF[i]);
  // }
}

bool R2D(bool tson, bool start, bool brake)
{
  static int step = 0;
  static unsigned long int tAux = millis();
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

  int val = abs(valAPPS1 - valAPPS2);

  if ((valAPPS1 >= valDesc) || (valAPPS2 >= valDesc))
  {
    return -1;
  }
  else if (val >= difMAX)
  {
    return 1;
    // return 0;
  }
  else
  {
    return 0;
  }
}

void configInverter()
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
      CAN.setPacketTimer(idMens, timeUpdateCTRL);
    }
    else
    {
      short dataCurrent[4];
      for (int i = 0; i < 4; i++)
      {
        dataCurrent[i] == 0xFFFF;
      }
      dataCurrent[0] = data;
      CAN.setPacket(idMens, dataCurrent);
      CAN.setPacketTimer(idMens, timeUpdateConfig);
    }
    Serial.println();
  }
}

void controlInverter()
{
  // Speed
  idPacket = 3;
  idMens = combineInts(idPacket, idNode);
  data = cfgERPM;

  intToByteArray(data, info);
  CAN.setPacket(idMens, info);
  CAN.printByteArray(info, 8);

  // Serial.println((String)data+" " +idMens);
  idPacket = 12;
  idMens = combineInts(idPacket, idNode);
  data = cfgDriveEnable;
  intToByteArray(data, info);
  CAN.setPacket(idMens, info);

  // Serial.println((String)data+" " +idMens);
  // Enable
}

int getFilteredAnalogRead(int val, int sampleSize)
{
  static int *readings = new int[sampleSize]; // Array to store the samples
  static int readIndex = 0;                   // Index of the current sample
  static int total = 0;                       // Running total
  static bool initialized = false;            // To check if the array is initialized

  // Initialize the readings array once
  if (!initialized)
  {
    for (int i = 0; i < sampleSize; i++)
    {
      readings[i] = 0;
    }
    initialized = true;
  }

  // Subtract the last reading
  total -= readings[readIndex];

  // Read the next sample
  readings[readIndex] = val;

  // Add the reading to the total
  total += readings[readIndex];

  // Advance to the next position in the array
  readIndex = (readIndex + 1) % sampleSize;

  // Calculate the average
  int average = total / sampleSize;

  return average;
}

int getFilteredAnalogRead2(int val, int sampleSize)
{
  static int *readings = new int[sampleSize]; // Array to store the samples
  static int readIndex = 0;                   // Index of the current sample
  static int total = 0;                       // Running total
  static bool initialized = false;            // To check if the array is initialized

  // Initialize the readings array once
  if (!initialized)
  {
    for (int i = 0; i < sampleSize; i++)
    {
      readings[i] = 0;
    }
    initialized = true;
  }

  // Subtract the last reading
  total -= readings[readIndex];

  // Read the next sample
  readings[readIndex] = val;

  // Add the reading to the total
  total += readings[readIndex];

  // Advance to the next position in the array
  readIndex = (readIndex + 1) % sampleSize;

  // Calculate the average
  int average = total / sampleSize;
  initialized = false;
  return average;
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