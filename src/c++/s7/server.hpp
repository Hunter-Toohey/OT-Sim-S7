#ifndef OTSIM_S7_SERVER_HPP
#define OTSIM_S7_SERVER_HPP

#include <atomic>
#include <mutex>
#include <thread>

#include "common.hpp"
#include "msgbus/envelope.hpp"
#include "msgbus/metrics.hpp"
#include "msgbus/pusher.hpp"

#include "snap7.h"

namespace otsim {
namespace s7 {

struct ServerConfig {
    std::string id;
    std::uint16_t address;
    std::string logLevel = "info";
};

class Server : public std::enable_shared_from_this<Server> {
public:
    static std::shared_ptr<Server> Create(ServerConfig config, Pusher pusher) {
        return std::make_shared<Server>(config, pusher);
    }

    Server(ServerConfig config, Pusher pusher);
    ~Server() {}

    std::string ID() { return config.id; }

    void Run(std::shared_ptr<TS7Server> ts7server);

    bool AddBinaryInput(BinaryInputPoint point);
    bool AddBinaryOutput(BinaryOutputPoint point);
    bool AddAnalogInput(AnalogInputPoint point);
    bool AddAnalogOutput(AnalogOutputPoint point);

    void WriteBinary(uint16_t address, bool value);
    void WriteAnalog(uint16_t address, double value);

    const BinaryOutputPoint* GetBinaryOutput(const uint16_t address);
    const AnalogOutputPoint* GetAnalogOutput(const uint16_t address);

    void ResetOutputs();
    void HandleMsgBusStatus(const otsim::msgbus::Envelope<otsim::msgbus::Status>& env);

    static void OnClientWrite(int area, int dbNumber, int start, int size, void* usrPtr);
    
    // Event callback handlers for server events
    static void OnServerEvent(void* usrPtr, PSrvEvent pEvent, int size);
    static void OnReadEvent(void* usrPtr, PSrvEvent pEvent, int size);
    
    // Read/Write area callback for ResourceLess mode support
    static int OnRWAreaCallback(void* usrPtr, int sender, int operation, PS7Tag pTag, void* pUsrData);

private:
    ServerConfig config;

    Pusher pusher;
    MetricsPusher metrics;

    std::shared_ptr<TS7Server> ts7server;

    std::map<std::uint16_t, BinaryInputPoint> binaryInputs;
    std::map<std::uint16_t, BinaryOutputPoint> binaryOutputs;
    std::map<std::uint16_t, AnalogInputPoint> analogInputs;
    std::map<std::uint16_t, AnalogOutputPoint> analogOutputs;

    std::map<std::string, otsim::msgbus::Point> points;
    std::mutex pointsMu;

    std::atomic<bool> running{false};
    
    // Memory layout constants matching real PLC architecture
    // Note: BINARY_OFFSET is 0 but defined for code clarity and future flexibility
    static constexpr uint16_t BINARY_OFFSET = 0;     // Bytes 0-255: Binary I/O (PIB/PQB)
    static constexpr uint16_t ANALOG_OFFSET = 256;   // Bytes 256-511: Analog I/O (PIW/PQW)
    static constexpr uint16_t BINARY_SIZE = 256;
    static constexpr uint16_t ANALOG_SIZE = 256;
    
    // S7 memory buffers - PE/PA hold both binary and analog I/O like real PLCs
    byte peBuffer[512] = {0};  // PE (Process Eingänge): Digital inputs + Analog inputs
    byte paBuffer[512] = {0};  // PA (Process Ausgänge): Digital outputs + Analog outputs
    byte mkBuffer[256] = {0};  // MK (Merker): Internal flags/markers
    byte dbBuffer[1024] = {0}; // DB (Data Blocks): Structured data, recipes, parameters
};

} // namespace s7
} // namespace otsim

#endif // OTSIM_S7_SERVER_HPP