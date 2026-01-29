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
    //friend function to allow rwCallback to access private buffers
    friend int rwCallback(void *usrPtr, int Sender, int Operation, PS7Tag PTag, void *pUsrData);

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

    static void OnClientWrite(int area, int dbNumber, int start, int size, void* usrPtr, void* pUsrData);
    
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
    
    // define memory buffer offsets for where we store things
    // the data is partitoned into areas for binary and analog I/O
    static constexpr uint16_t BINARY_OFFSET = 0;     // bytes 0-255: binary I/O (bit-level addressing)
    static constexpr uint16_t ANALOG_OFFSET = 256;   // bytes 256-511: analog I/O (4-byte floats)
    static constexpr uint16_t BINARY_SIZE = 256;
    static constexpr uint16_t ANALOG_SIZE = 256;
    
    // memory buffers for S7 PLC
    byte peBuffer[512] = {0};  // Process Image Inputs (I area) currently we don't actually use this buffer
    byte paBuffer[512] = {0};  // Process Image Outputs (Q area) - PLC writes feedback here (if we publish, we pull from PA)
    byte mkBuffer[256] = {0};  // Merker/Flags (M area) - internal memory, currently we don't use this buffer either
                                // I think this server would be used if we wanted to simulate logic in the PLC
                                        // but that's not something we are interested in right now
    byte dbBuffer[1024] = {0}; // Data Blocks (DB area) - commands from OT-sim logic written here (if we subscribe, it goes to DB)
};

} // namespace s7
} // namespace otsim

#endif // OTSIM_S7_SERVER_HPP