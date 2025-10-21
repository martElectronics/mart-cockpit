//********MART CAN CONVERTER LIBRARY
#ifndef MARTCAN_H
#define MARTCAN_H

#include <Arduino.h>

#include <bitset>
#include <cstring>
#include <utility>

// #include <optional>
#include "mcp_can.h"
#include "CAN_DATA.h"
#include "common.h"
#include "MCP2515_Config.h"

#if defined(ARDUINO_MICRO)
#include <ArduinoSTL.h>
#endif

#if defined(ESP32) || defined(ESP32S3)
#include <vector>
#include <algorithm>
#include <functional>
#include <ESP32-TWAI-CAN.hpp>
#endif

#define MCP_SPEED_500 500
#define MCP_SPEED_1000 1000

enum class HardwareType
{
    Transciever,
    Controller
};
class CAN_BUS
{

public:
    // CONVERTER converter;
    MCP_CAN _CAN;
    CAN_DATA DataIN, DataOUT;

    HardwareType type;
    int error;
    int RX, TX;
    int timeout;
    struct Config
    {
        bool respondToRRF;            // Automaticaly respond to a RRF
        bool autoRemoveRRFPacket;     // Automaticaly delete a received rrf when the requested data is sent
        bool simulating;              // Set to true for testing when no MCP2515 are connected to the microcontroller
        bool autoRemoveStoredFilters; // Removes the stored masks and filters when applied to the MCP2515 registers to save memory
        bool sendStatusData;          // Send status data such as runtime time, number of sent, received and collided paquets.
    } config;

    // Constructor: Initializes the MCP_CAN instance and sets up the CAN interface

    CAN_BUS(int pinCs) : _CAN(pinCs)
    {
        if (_CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
            Serial.println("MCP2515 Initialized Successfully!");
        else
            Serial.println("Error Initializing MCP2515...");
        _CAN.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

        // Default configuration
        config.respondToRRF = true;
        config.autoRemoveRRFPacket = true;
        config.simulating = false;
        config.autoRemoveStoredFilters = true;
        config.sendStatusData = false;
    }

    // Constructor: Initializes the MCP_CAN instance and sets up the CAN interface
    CAN_BUS(int pinCs, int _nodeID) : _CAN(pinCs)
    {
        if (_CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
            Serial.println("MCP2515 Initialized Successfully!");
        else
            Serial.println("Error Initializing MCP2515...");
        _CAN.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted

        // Default configuration
        config.respondToRRF = true;
        config.autoRemoveRRFPacket = true;
        config.simulating = false;
        config.autoRemoveStoredFilters = true;
        config.sendStatusData = false;
    }

    // Destructor
    ~CAN_BUS() {}

    void setupCANHardware(unsigned int speed, uint16_t txQueue = 10, uint16_t rxQueue = 10, int timeoutRead = 100)
    {
        error = 0;
        timeout = timeoutRead;
        // this->type = type; --> Cambiado, está en el constructor para que no de error a la hora de llamarlo en el main
        if (this->type == HardwareType::Controller)
        {
            if (speed == MCP_SPEED_500)
            {
                if (_CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
                {
                    Serial.println("MCP2515 Initialized Successfully!");
                }
                else
                {
                    Serial.println("Error Initializing MCP2515...");
                    error = 1;
                    return;
                }
                _CAN.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted
            }
            else if (speed == MCP_SPEED_1000)
            {
                if (_CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
                {
                    Serial.println("MCP2515 Initialized Successfully!");
                }
                else
                {
                    Serial.println("Error Initializing MCP2515...");
                    error = 1;
                    return;
                }
            }
            else
            {
                Serial.println("Error Initializing MCP2515 INVALID SPEED...");
                error = 1;
                return;
            }
            _CAN.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted
        }
        else if (this->type == HardwareType::Transciever)
        {
#if defined(ESP32) || defined(ESP32S3)
            // ESP32Can.setPins(RX, TX);
            ESP32Can.setSpeed(ESP32Can.convertSpeed(speed));
            //**CORREGIR3: El tamaño de la cola debe de ser genérica y configurable por parámetro, podéis usar unos parámetros por defecto como en el caso del constructor
            if (!ESP32Can.begin(ESP32Can.convertSpeed(speed), TX, RX, txQueue, rxQueue))
            {
                Serial.println("Error Initializing ESP32Can...");
                error = 1;
            }
#endif
        }
    }
    // Constructor: Initializes the transciever or controller instance and sets up the CAN interface
    CAN_BUS(HardwareType type, unsigned int speed, int _nodeID, int pinCs = 0, int8_t TX = 5, int8_t RX = 4, uint16_t txQueue = 10, uint16_t rxQueue = 10) : _CAN(pinCs), type(type), RX(RX), TX(TX)
    {

        //**CORREGIR3: Sigue sin estar bien. En el caso de que se use el MCP2515 se tendrían que ignorar los argumentos TX y RX y no es posible ya que el pinCs está al final.
        //** Para solucionar esto podéis:
        //**1. Poner el pinCs como cuarto argumento y tx y rx los últimos además de implementar lógica adicional para configurar el controller o el transceiver, ya que aunque el constructor se llame con sólo 4 argumentos (en caso del controller),
        //** no se sabría el tipo de hardware a usar. */  o bien:
        //**2. Crear dos constructores que inicialicen un hardware u otro (con 4 argumentos para el controller y 6 para el transceiver) */
        //**3. Otra forma que se os ocurra a vosotros */
        //**Si veis que los constructores que yo implementé en su día os estan fastidiando y queréis usar otros para que sean más compatibles con vuestra lógica los podéis cambiar, no problem */

        timeout = 100;
        setupCANHardware(speed, txQueue, rxQueue);
        // Default configuration
        config.respondToRRF = true;
        config.autoRemoveRRFPacket = true;
        config.simulating = false;
        config.autoRemoveStoredFilters = true;
        config.sendStatusData = false;
    }
    // Sends all stored data packets in DataOUT
    bool send();
    // setup for transciever or controller

    int SetupState()
    {
        return error;
    }

    // Sends a specific stored data packet in DataOUT
    bool send(unsigned long id);

    // Sends a specific stored data packet in DataOUT
    bool sendRequestedRRF(unsigned long id);

    // Receives data packets and stores them in DataIN
    void receive();

    template <typename T>
    std::size_t calculateTotalSize(T &arg)
    {
        return arraySizeInBits(arg);
    }

    template <typename T, typename... Args>
    std::size_t calculateTotalSize(T &first, Args &...rest)
    {
        return arraySizeInBits(first) + calculateTotalSize(rest...);
    }

    //*** NEW SETPACKET AND GETPACKET METHODS */
    template <typename T>
    bool setPacket(uint32_t canId, const T *data, size_t dataSize, bool cmdConversionToBigEndian = true)
    {
        bool ok = true;
        // Calculate total size needed
        size_t dataBytes = sizeof(T) * dataSize;

        // Check for overflow
        if (dataBytes > 8)
        {
            ERROR_LOOP("ERROR, SETPACKET OVERFLOW");
            ok = false;
        }
        else
        {
            DataOUT.dataRaw.size = 8;
            DataOUT.dataRaw.id = canId;
            DataOUT.dataRaw.typeExtendedId = DataOUT.dataRaw.id > 0x7FF;
            DataOUT.dataRaw.rrf = false;
            DataOUT.dataRaw.WaitForRRF = false;
            uint8_t *outputArray = DataOUT.dataRaw.bytes;
            // Clear the output array
            std::memset(outputArray, 0xFF, 8);

            // Copy the data
            if (data != nullptr && dataSize > 0)
            {
                if (!cmdConversionToBigEndian)
                {
                    std::memcpy(outputArray, data, dataBytes);
                }
                else
                {
                    for (size_t i = 0; i < dataSize; ++i)
                    {
                        T value = data[i];
                        size_t elementSize = sizeof(T);

                        // Calculate starting position for this element in the output array
                        size_t startPos = i * elementSize;

                        // Copy bytes in reverse order (little endian to big endian)
                        for (size_t j = 0; j < elementSize; ++j)
                        {
                            outputArray[startPos + j] = static_cast<byte>((value >> ((elementSize - 1 - j) * 8)) & 0xFF);
                        }
                    }
                }
                DataOUT.addPacket(DataOUT.dataRaw);
            }
            else
            {
                ERROR_LOOP("ERROR, SETPACKET EMPTY");
                ok = false;
            }
        }
        return ok;
    }

    /**
     * Deserializes data from an 8-byte packet.
     *
     * @param inputArray The input 8-byte array
     * @param data Pointer to the data array to populate
     * @param dataSize Number of elements expected in the data array
     * @throws std::overflow_error If trying to extract more data than available
     */
    template <typename T>
    bool getPacket(uint32_t canId, T *data, size_t dataSize, bool cmdConversionToLittleEndian = true)
    {
        bool ok = true;
        // Calculate size
        size_t dataBytes = sizeof(T) * dataSize;

        // Check for overflow
        if (dataBytes > 8)
        {
            ERROR_LOOP("ERROR, GETPACKET OVERFLOW");
            ok = false;
        }
        else
        {
            const CanPacketRawData *packet = DataIN.getPacketById(canId);
            if (packet != nullptr)
            {
                if (data != nullptr && dataSize > 0)
                {
                    // Extract the data
                    if (!cmdConversionToLittleEndian)
                    {
                        std::memcpy(data, packet->bytes, dataBytes);
                    }

                    else
                    {
                        for (size_t i = 0; i < dataSize; ++i)
                        {
                            T value = 0;
                            size_t elementSize = sizeof(T);

                            // Calculate starting position for this element in the input array
                            size_t startPos = i * elementSize;

                            // Convert big endian to little endian by reading bytes in reverse order
                            for (size_t j = 0; j < elementSize; ++j)
                            {
                                value = (value << 8) | data[startPos + j];
                            }

                            data[i] = value;
                        }
                    }
                }
                else
                {
                    ERROR_PRINTLN("Error: getPacket Empty array");
                    ok = false;
                }
            }
            else
            {
                ERROR_PRINTLN("Error: getPacket No matching packet found.");
                ok = false;
            }
        }

        return (ok || config.simulating);
    }

    template <typename T>
    void packArgumentsRecursive(uint8_t *outputArray, size_t &offset, T &arg)
    {
        offset = packArgument(arg, outputArray, offset);
    }
    template <typename T, typename... Args>
    void packArgumentsRecursive(uint8_t *outputArray, size_t &offset, T &first, Args &...rest)
    {
        offset = packArgument(first, outputArray, offset);
        packArgumentsRecursive(outputArray, offset, rest...);
    }
    // Packs RRF message
    void setPacket(unsigned long canId)
    {
        DataOUT.dataRaw.size = 8;
        DataOUT.dataRaw.rrf = true;
        DataOUT.dataRaw.id = canId;
        DataOUT.dataRaw.typeExtendedId = (DataOUT.dataRaw.id > 0x7FF);
        DataOUT.dataRaw.WaitForRRF = false;
        DataOUT.addPacket(DataOUT.dataRaw);
    }

    // Function template to calculate size in bits of a single array
    template <typename T, std::size_t N>
    constexpr std::size_t arraySizeInBits(const T (&)[N])
    {
        return sizeof(T) * N * 8; // Size of each element times number of elements times 8 bits/byte
    }
    // Function template to calculate size in bits of a single array
    template <std::size_t N>
    constexpr std::size_t arraySizeInBits(const bool (&)[N])
    {
        return N; // 1 bit for each bool
    }

    // Helper function to pack a single array

    void printByteArray(const uint8_t *array, size_t size)
    {
        for (size_t i = 0; i < size; ++i)
        {
            if (array[i] < 0x10)
                Serial.print("0");
            Serial.print(array[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Template function to print an array of any type and size
    template <typename T, size_t N>
    void printArray(const T (&array)[N])
    {
        Serial.print("[");
        for (size_t i = 0; i < N; ++i)
        {
            if (i > 0)
            {
                Serial.print(", ");
            }
            Serial.print(array[i]);
        }
        Serial.println("]");
    }
    // Configures the RRF pairs
    void setRRFId(unsigned long inId, unsigned long outId);

    // Calculates and writes the masks and filters to the MCP2515 registers given a set of IDs
    bool setFilters(const unsigned long ids[], unsigned size);

    // Prints the calculated masks and filters
    void printFilters();

    // Test by software if the given IDs are accepted by the created filters or not
    void testFilters(const std::vector<uint16_t> &testIds);

    void setPacketTimer(unsigned long packetID, unsigned long time);
    void printStatusData(unsigned _nodeID);
    void printReceivedIds();

    //** CAN STATUS DATA**//
    void setCANStatusData();
    void getCANStatusData();

    //** CAN BUS STATUS DATA **//
    unsigned nodeID, statusPacketOffset;                                                             // IDs
    unsigned runtimeTime, numRXPaqOK, numTXPaqOK, numTxPaqError;                                     // Actual data
    unsigned previousStatusIntervalTime, previousStatusRuntimeTime, intervalTime, numCurrentSamples; // Aux data

private:
    struct RRFIds
    {
        std::vector<unsigned long> INRRFid;
        std::vector<unsigned long> OUTRRFid;
    };
    struct PacketTimer
    {
        unsigned long packetID;
        unsigned long interval;
    };
    std::vector<PacketTimer> packetTimers;

    std::vector<RRFIds> rrfIdsList;       // Vector holding INRRFid and OUTRRFid vectors
    std::vector<unsigned long> filterIDs; // Vector holding INRRFid and OUTRRFid vectors
    MCP2515Configurator configurator;

    bool readBytes();
    bool writeBytes();
    // Method to search for an InID and return the associated OUTids vector
    std::vector<unsigned long> getOutIdsByInId(unsigned long inId);

    // Method to search for an OUTid and return true if found
    bool searchOutId(unsigned long outId);

    /*
    //Change the speed of the bus while running (ms)
    void ChangeSpeed(unsigned int speed){
        ESP32Can.setSpeed(ESP32Can.convertSpeed(speed));
    }
    */
    // Method to print all RRFIds
    void printRRFIds()
    {
        for (const auto &rrfIds : rrfIdsList)
        {
            Serial.print("INRRFid: ");
            for (const auto &id : rrfIds.INRRFid)
            {
                Serial.print(id);
                Serial.print(" ");
            }
            Serial.println(); // New line after printing INRRFid

            Serial.print("OUTRRFid: ");
            for (const auto &id : rrfIds.OUTRRFid)
            {
                Serial.print(id);
                Serial.print(" ");
            }
            Serial.println(); // New line after printing OUTRRFid
        }
    }
};

#endif