

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
#define SPI_CS 10       // SPI slave select
#define ADC_VREF 3300   // 3.3V Vref
#define ADC_CLK 1600000 // SPI clock 1.6MHz

CAN_BUS CAN(HardwareType::Transciever, 125, 1);
MCP3208 adc(ADC_VREF, SPI_CS);
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// PINES

int pinTSON = 21, pinStart = 35, pinBUZZ = 15, pinTSON_EXT = 8;

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

uint32_t idMens, idR2D = 100;
uint32_t idPacket, idNode = 1;
int data;
bool enableInverter;
int RPMtarget, cfgRPMax = 1500, currentTarget;

uint32_t idCmdRPM = combineInts(3, idNode);              // 97
uint32_t idCmdEN = combineInts(12, idNode);              // 385
uint32_t idCmdCurrent = combineInts(5, idNode);          // ??
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
    cfgCurrentACMAX = 5,      // 8
    cfgCurrentACBrakeMAX = 0, // 9
    cfgCurrentDCMAX = 0,      // 10
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

// PRG
bool stsTSON, stsStart, stsR2D, stsAPPS;
int stsBrake, stsPot;
int cfgBrakeTH = 500, cfgChannelAPPS1 = 0, cfgChannelAPPS2 = 1, cfgChannelBrake = 2, cfgChannelSteering = 3;
int cfgAPPSdiff = 10, cfgAPPSdesc = 5000, cfgAPPSdescScaled = 200, cfgAPPSmax = 0, cfgTimeSoundR2D = 1100;
float cfgAPPSMargin = 0.1; // (0.1=10%)
int cfgScaleFactor = 1000; // Aumenta resolución al hacer map(), luego se divide por el mismo factor en la consigna final
uint32_t tA = millis();

void configureAPPS()
{
  apps1Data.valAnalogUP = 1850;   // 2710
  apps1Data.valAnalogDOWN = 2710; // 1850
  apps2Data.valAnalogUP = 790;   // 790
  apps2Data.valAnalogDOWN = 1670;  // 1670
  apps1Data.valScaledUP = apps2Data.valScaledUP = 0;
  apps1Data.valScaledDOWN = apps2Data.valScaledDOWN = 1000;
  apps1Data.range = abs(apps1Data.valAnalogUP - apps1Data.valAnalogDOWN);
  apps2Data.range = abs(apps2Data.valAnalogUP - apps2Data.valAnalogDOWN);
}

void setup()
{

  Serial.begin(115200);
  // PINES
  pinMode(pinStart, INPUT_PULLUP);
  pinMode(pinTSON, INPUT);
  pinMode(pinTSON_EXT, INPUT);
  pinMode(pinBUZZ, OUTPUT);

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

  //****LECTURA
  CAN.receive();
  stsTSON = digitalRead(pinTSON_EXT); // cambiado por logica
  stsStart = !digitalRead(pinStart);

  stsBrake = adc.read(MCP3208::Channel::SINGLE_5);
  // stsBrake = cfgBrakeTH ;
  apps1Data.valAnalog = adc.read(MCP3208::Channel::SINGLE_7);
  apps2Data.valAnalog = adc.read(MCP3208::Channel::SINGLE_6);
  stsPot = adc.read(MCP3208::Channel::SINGLE_4);
  stsPot = map(stsPot, 0, 4096, 0, 1000);
  // stsBrake = adc.read(MCP3208::Channel::SINGLE_5);;

  //****PROCESAMIENTO
  // APPS
  // Returns a negative number if the APPS is below the safety margin
  apps1Data.valScaled =
      map(apps1Data.valAnalog, apps1Data.valAnalogUP - apps1Data.range * cfgAPPSMargin, apps1Data.valAnalogDOWN + apps1Data.range * cfgAPPSMargin, apps1Data.valScaledDOWN, apps1Data.valScaledUP);

  apps2Data.valScaled =
      map(apps2Data.valAnalog, apps2Data.valAnalogUP - apps2Data.range * cfgAPPSMargin, apps2Data.valAnalogDOWN + apps2Data.range * cfgAPPSMargin, apps2Data.valScaledDOWN, apps2Data.valScaledUP);

  stsAPPS = apps(apps1Data.valScaled, apps2Data.valScaled, 100, 0, 100);

  // R2D
  // stsR2D = stsStart; //*****COMENTAR
  // stsR2D = true; //REVISAR
  stsAPPS = 0; //*****COMENTAR
  stsR2D = R2D(true, stsStart, (stsBrake >= cfgBrakeTH));

  // Target speed/rpm
  if (!stsR2D || stsAPPS != 0)
  {
    RPMtarget = 0;
    currentTarget = 0;
  }
  else if (stsR2D && stsAPPS == 0)
  {
    RPMtarget = map(apps1Data.valScaled, apps1Data.valScaledDOWN, apps1Data.valScaledUP, 0, cfgRPMax * 10);
    // currentTarget = map(apps1Data.valScaled, apps1Data.valScaledDOWN, apps1Data.valScaledUP, 0, 1000); // current target in % * 10
    currentTarget = map(apps1Data.valScaled, apps1Data.valScaledDOWN, apps1Data.valScaledUP, 0, cfgCurrentACMAX * 10 * cfgScaleFactor);
    if (RPMtarget < 0)
      RPMtarget = 0;
    else if (RPMtarget > (cfgRPMax * 10))
      RPMtarget = cfgRPMax * 10;
    if (currentTarget < 0)
      currentTarget = 0;
    else if (currentTarget > cfgCurrentACMAX * 10 * cfgScaleFactor)
      currentTarget = cfgCurrentACMAX * 10 * cfgScaleFactor;
  }
  // DRIVE ENABLE
  if (!stsR2D)
  {
    cfgDriveEnable = 0;
    currentTarget = 0;
  }
  else
  {
    cfgDriveEnable = 1;
  }

  cmdDataDriveEN[0] = (byte)cfgDriveEnable;
  cmdDataRPM[0] = RPMtarget;
  cmdDataCurrent[0] = currentTarget / cfgScaleFactor;

  cmdDataCurrentACMax[0] = cfgCurrentACMAX * 10;
  cmdDataCurrentDCMax[0] = 5 * 10;
  // ******WRITE

  CAN.setPacket(idCmdEN, cmdDataDriveEN, 1);
  if (cfgCtrlBySpeed)
  {

    // CAN.setPacket(idCmdRPM, cmdDataRPM, 2);
  }
  else
  {
    CAN.setPacket(idCmdCurrent, cmdDataCurrent, 2);
    CAN.setPacket(idCmdSetMaxACCurrent, cmdDataCurrentACMax, 4);
  }

  // cmdDataCurrentACMax[0] = stsPot;
  // CAN.setPacket(idCmdSetMaxACCurrent, cmdDataCurrentACMax, 4);

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

   Serial.println((String) "APPS1: " + apps1Data.valScaled + " APPS2: " + apps2Data.valScaled);
  // Serial.println((String) "Brake: " + stsBrake + " TSON: " + stsTSON + " Start: " + stsStart + " R2D: " + stsR2D);
  // Serial.println((String) "Drive Enable: " + cmdDataDriveEN[0] + " Current: " + cmdDataCurrent[0] + " EERPM: " + cmdDataRPM[0]);

  // Serial.println((String)apps1Data.valAnalog + " valScaled: " + apps1Data.valScaled + " RPM: " + RPMtarget + " Current pctg: " + currentTarget);
  // Serial.println();
  // delay(500);

  if ((millis() - tAux) >= 1000)
  {
    Serial.println((String) "Current=" + cmdDataCurrent[0]);
    //Serial.println((String) "APPS1 analog: " + apps1Data.valAnalog + " APPS2: " + apps2Data.valAnalog + " Brake= " + stsBrake);
    Serial.println((String) "Start = " + stsStart);
    Serial.println((String) "TSON = " + stsTSON);
    tAux = millis();
  }
}

void debugShowADC()
{
  // // Serial.println("-----------------------------------------------------------");
  // Serial.print("AIN0: ");
  // Serial.print(adc0);
  // Serial.print("  ");
  // Serial.print(volts0);
  // Serial.println("V");
  // Serial.print("AIN1: ");
  // Serial.print(adc1);
  // Serial.print("  ");
  // Serial.print(volts1);
  // Serial.println("V");
  // // Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  // Serial.print("AIN3: ");
  // Serial.print(adc3);
  // Serial.print("  ");
  // Serial.print(volts3);
  // Serial.println("V");
  // delay(500);
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

  if ((valAPPS1 <= valDesc) || (valAPPS2 <= valDesc))
  {
    return -1;
  }
  else if (val >= difMAX)
  {
    return 1;
  }
  else
  {
    return 0;
  }

  if ((millis() - t) >= 1000)
  {
    Serial.println("APPS DIF= " + val);
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