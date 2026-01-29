#include <iostream>
#include <cstring>
#include "server.hpp"
#include "s7_utils.hpp"
#include "fmt/format.h"
#include "msgbus/metrics.hpp"
#include "snap7.h"

namespace otsim {
namespace s7 {
  //when a client talks to a server, this function packages that data into a format understandle by the server
  int S7API rwCallback(void *usrPtr, int Sender, int Operation, PS7Tag PTag, void *pUsrData) {
    if (!PTag || !pUsrData || !usrPtr) return 0;

    auto server = reinterpret_cast<Server*>(usrPtr);

    int area = PTag->Area;
    if (area == S7AreaMK) area = srvAreaMK;
    else if (area == S7AreaDB) area = srvAreaDB;
    else if (area == S7AreaPE) area = srvAreaPE;
    else if (area == S7AreaPA) area = srvAreaPA;

    if (Operation == OperationRead) {
      //if client is reading from server, copy from buffer to pUsrData
      std::unique_lock<std::mutex> lock(server->pointsMu);
      if (area == srvAreaPA && static_cast<size_t>(PTag->Start + PTag->Size) <= sizeof(server->paBuffer)) {
        std::memcpy(pUsrData, server->paBuffer + PTag->Start, PTag->Size);
      } else if (area == srvAreaDB && static_cast<size_t>(PTag->Start + PTag->Size) <= sizeof(server->dbBuffer)) {
        std::memcpy(pUsrData, server->dbBuffer + PTag->Start, PTag->Size);
      } else if (area == srvAreaPE && static_cast<size_t>(PTag->Start + PTag->Size) <= sizeof(server->peBuffer)) {
        std::memcpy(pUsrData, server->peBuffer + PTag->Start, PTag->Size);
      } else if (area == srvAreaMK && static_cast<size_t>(PTag->Start + PTag->Size) <= sizeof(server->mkBuffer)) {
        std::memcpy(pUsrData, server->mkBuffer + PTag->Start, PTag->Size);
      }
    } else if (Operation == OperationWrite) {
      //if client is writing to server, copy from pUsrData to buffer
      std::unique_lock<std::mutex> lock(server->pointsMu);
      if (area == srvAreaPA && static_cast<size_t>(PTag->Start + PTag->Size) <= sizeof(server->paBuffer)) {
        std::memcpy(server->paBuffer + PTag->Start, pUsrData, PTag->Size);
      } else if (area == srvAreaDB && static_cast<size_t>(PTag->Start + PTag->Size) <= sizeof(server->dbBuffer)) {
        std::memcpy(server->dbBuffer + PTag->Start, pUsrData, PTag->Size);
      }
      lock.unlock();

      //now publish to the msgbus if that write is an output we're publishing to through an xml output
      Server::OnClientWrite(area, PTag->DBNumber, PTag->Start, PTag->Size, usrPtr, pUsrData);
    }

    return 0;
  }

  // writes a binary value to S7 buffer using bit-level addressing
  void WriteBinaryToS7(byte* buffer, size_t bufLen, std::uint16_t bitAddress, bool value) {
    if (!utils::writeBit(buffer, bufLen, bitAddress, value)) {
      std::cerr << "[S7] Binary write out of bounds: bitAddr=" << bitAddress
                << " (byte " << utils::getByteOffset(bitAddress)
                << ", bit " << static_cast<int>(utils::getBitOffset(bitAddress))
                << ") bufLen=" << bufLen << std::endl;
    }
  }

  // reads a binary value from S7 buffer using bit-level addressing
  bool ReadBinaryFromS7(byte* buffer, size_t bufLen, std::uint16_t bitAddress) {
    bool value = false;
    if (!utils::readBit(buffer, bufLen, bitAddress, value)) {
      std::cerr << "[S7] Binary read out of bounds: bitAddr=" << bitAddress
                << " (byte " << utils::getByteOffset(bitAddress)
                << ", bit " << static_cast<int>(utils::getBitOffset(bitAddress))
                << ") bufLen=" << bufLen << std::endl;
    }
    return value;
  }

  // writes analog value to S7 buffer as big-endian IEEE 754 float
  // addr is byte offset in buffer (must be properly aligned)
  void WriteAnalogToS7(byte* buffer, size_t bufLen, std::uint16_t addr, float value) {
    if (!utils::writeReal(buffer, bufLen, addr, value)) {
      std::cerr << "[S7] Analog write out of bounds: addr=" << addr << " bufLen=" << bufLen << std::endl;
    }
  }

  // reads analog value from S7 buffer with big-endian to host conversion
  float ReadAnalogFromS7(byte* buffer, size_t bufLen, std::uint16_t addr) {
    float value = 0.0f;
    if (!utils::readReal(buffer, bufLen, addr, value)) {
      std::cerr << "[S7] Analog read out of bounds: addr=" << addr << " bufLen=" << bufLen << std::endl;
    }
    return value;
  }

  // constructor, initializer metrics
  Server::Server(ServerConfig config, Pusher pusher): config(config), pusher(pusher) {
    metrics = std::make_shared<otsim::msgbus::MetricsPusher>();
    if (!metrics) {
      std::cerr << "[S7] Metrics pointer not initialized!" << std::endl;
    } else {
      metrics->NewMetric("Counter", "status_count", "number of OT-sim status messages processed");
      metrics->NewMetric("Counter", "update_count", "number of OT-sim update messages generated");
      metrics->NewMetric("Counter", "s7_binary_write_count", "number of S7 binary writes processed");
      metrics->NewMetric("Counter", "s7_analog_write_count", "number of S7 analog writes processed");
    }
  }
  
  // run the server loop and update memory
  void Server::Run(std::shared_ptr<TS7Server> ts7server) {
    this->ts7server = ts7server;

    //debugging output
    if (!metrics || !pusher) {
        std::cerr << "[S7] Metrics or pusher not initialized!" << std::endl;
        return;
    }
    metrics->Start(pusher, config.id);
    running = true;
    //start server before registering areas, otherwise we segfault
    if (ts7server->Start() != 0) {
        std::cerr << "[S7] Failed to start Snap7 server!" << std::endl;
        return;
    }
    
    //set CPU to RUN status to allow block operations
    ts7server->SetCpuStatus(S7CpuStatusRun);
    std::cout << "[S7] CPU status set to RUN (block operations enabled)" << std::endl;
    //register memory buffers for PE, PA, MK, and DB areas matching real PLC architecture
    int peResult = ts7server->RegisterArea(srvAreaPE, 0, peBuffer, sizeof(peBuffer));
    if (peResult != 0) {
      std::cerr << "[S7] Failed to register PE area! Error code: " << peResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(peResult) << std::endl;
      return;
    }
    int paResult = ts7server->RegisterArea(srvAreaPA, 0, paBuffer, sizeof(paBuffer));
    if (paResult != 0) {
      std::cerr << "[S7] Failed to register PA area! Error code: " << paResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(paResult) << std::endl;
      return;
    }
    int mkResult = ts7server->RegisterArea(srvAreaMK, 0, mkBuffer, sizeof(mkBuffer));
    if (mkResult != 0) {
      std::cerr << "[S7] Failed to register MK area! Error code: " << mkResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(mkResult) << std::endl;
      return;
    }
    int dbResult = ts7server->RegisterArea(srvAreaDB, 1, dbBuffer, sizeof(dbBuffer));
    if (dbResult != 0) {
      std::cerr << "[S7] Failed to register DB area! Error code: " << dbResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(dbResult) << std::endl;
      return;
    }

    //enable important server events
    //enable block upload/download, directory operations, and client events
    //this maybe can be deleted?
    longword eventMask = evcServerStarted | evcServerStopped |
                         evcClientAdded | evcClientDisconnected |
                         evcUpload | evcDownload | evcDirectory |
                         evcDataRead | evcDataWrite;
    ts7server->SetEventsMask(eventMask);

    //register read/write callback to handle client writes to outputs
    int callbackResult = ts7server->SetRWAreaCallback(rwCallback, this);
    if (callbackResult != 0) {
      std::cerr << "[S7] Failed to register RW area callback! Error code: " << callbackResult << std::endl;
      return;
    }
    std::cout << "[S7] RW area callback registered" << std::endl;

    //debugging output
    std::cout << "[S7] Server started, memory areas registered, and callbacks configured." << std::endl;

    //this is the main running loop, it scans the subscribed points and writes them to memory
    while (running) {
      TSrvEvent event;
      if (ts7server->PickEvent(&event)) {
          char eventText[256]; // Buffer for the event text
          Srv_EventText(&event, eventText, sizeof(eventText));
          if (event.EvtCode == evcClientAdded) {
              std::cout << fmt::format("[{}] CLIENT CONNECTED: [{}]", config.id, eventText) << std::endl;
          } else if (event.EvtCode == evcClientDisconnected) {
              std::cout << fmt::format("[{}] CLIENT DISCONNECTED: [{}]", config.id, eventText) << std::endl;
          } 
      }

      std::unique_lock<std::mutex> lock(pointsMu);

      //write binary inputs to DB area (bytes 0-255)
      //inputs receive data from message bus status messages and expose them to S7 clients as DB data
      for (auto& kv : binaryInputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] binaryInputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteBinaryToS7(dbBuffer, sizeof(dbBuffer), addr, point.value != 0);
        std::cout << fmt::format("[{}] updated binary input DB.{} to {}", config.id, addr, point.value) << std::endl;
        metrics->IncrMetric("s7_binary_write_count");
      }

      //write analog inputs to DB area (bytes 256+, 4 bytes per analog value)
      //inputs receive data from message bus Status messages and expose them to S7 clients as DB data
      for (auto& kv : analogInputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] analogInputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteAnalogToS7(dbBuffer, sizeof(dbBuffer), ANALOG_OFFSET + addr, static_cast<float>(point.value));
        std::cout << fmt::format("[{}] updated analog input DB.{} to {}", config.id, addr, point.value) << std::endl;
        metrics->IncrMetric("s7_analog_write_count");
      }

      lock.unlock();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  void Server::OnClientWrite(int area, int dbNumber, int start, int size, void* usrPtr, void* pUsrData) {
    auto server = reinterpret_cast<Server*>(usrPtr);
    std::unique_lock<std::mutex> lock(server->pointsMu);

    //NOTE: Data has already been copied from pUsrData to our buffer in rwCallback
    //This function just processes the write (updates points map and publishes to message bus)

    //handle PA area writes (Process Image Outputs - both binary and analog)
    if (area == srvAreaPA) {
      //check if this is binary output write (Q area, bytes 0-255)
      if (start >= server->BINARY_OFFSET && start < server->BINARY_OFFSET + server->BINARY_SIZE) {
        if (static_cast<size_t>(start) < sizeof(server->paBuffer)) {
          // use bit-level addressing to read the binary value
          uint16_t bitAddr = start - server->BINARY_OFFSET;
          bool val = ReadBinaryFromS7(server->paBuffer, sizeof(server->paBuffer), bitAddr);

          if (server->binaryOutputs.find(bitAddr) == server->binaryOutputs.end()) {
            std::cerr << "[S7] OnClientWrite PA: binaryOutputs not found for bit address " << bitAddr << " (Q" << utils::getByteOffset(bitAddr) << "." << static_cast<int>(utils::getBitOffset(bitAddr)) << ")" << std::endl;
            return;
          }
          const auto& tag = server->binaryOutputs[bitAddr].tag;
          if (server->points.find(tag) == server->points.end()) {
            std::cerr << "[S7] OnClientWrite PA: points not found for tag=" << tag << std::endl;
            return;
          }
          server->points[tag].value = val ? 1.0 : 0.0;
          server->WriteBinary(bitAddr, val);
          std::cout << fmt::format("[S7] Client wrote Q{}.{} (bit addr {}) = {}", utils::getByteOffset(bitAddr), static_cast<int>(utils::getBitOffset(bitAddr)), bitAddr, val) << std::endl;
        }
      }
      //check if this is analog output write (Q area, bytes 256+)
      else if (start >= server->ANALOG_OFFSET && start + sizeof(float) <= sizeof(server->paBuffer)) {
        uint16_t addr = start - server->ANALOG_OFFSET;
        // Use big-endian aware read function
        float val = ReadAnalogFromS7(server->paBuffer, sizeof(server->paBuffer), start);

        if (server->analogOutputs.find(addr) == server->analogOutputs.end()) {
          std::cerr << "[S7] OnClientWrite PA: analogOutputs not found for PQW." << addr << std::endl;
          return;
        }
        const auto& tag = server->analogOutputs[addr].tag;
        if (server->points.find(tag) == server->points.end()) {
          std::cerr << "[S7] OnClientWrite PA: points not found for tag=" << tag << std::endl;
          return;
        }
        server->points[tag].value = val;
        server->WriteAnalog(addr, val);
        std::cout << fmt::format("[S7] Client wrote PQW.{} (byte {}) = {}", addr, start, val) << std::endl;
      }
    }
    // DB area writes are for structured data, not I/O
    else if (area == srvAreaDB) {
      std::cout << fmt::format("[S7] Client wrote to DB{} at offset {}, size {}", dbNumber, start, size) << std::endl;
    }
  }


  bool Server::AddBinaryInput(BinaryInputPoint point) {
    /*
    * in our binary inputs array, at the position equal to the address of the 
    * point being passed in, set the value equal to the incoming point. Then
    * create a msgbus Point structure and store it in the list of points
    */
    binaryInputs[point.address] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 0.0, 0};

    return true; //assuming this doesn't fail, return true
  }

  bool Server::AddBinaryOutput(BinaryOutputPoint point) {
    point.output = true;

    //store the point and point tag into the binaryOutputs and points arrays respectively
    binaryOutputs[point.address] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 0.0, 0};

    return true; 
  }

  bool Server::AddAnalogInput(AnalogInputPoint point) {

    //store the point and point tag into the analogInputs and points arrays respectively
    analogInputs[point.address] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 0.0, 0};

    return true;
  }

  bool Server::AddAnalogOutput(AnalogOutputPoint point) {
    point.output = true;

    //store the point and point tag into the analogOutputs and points arrays respectively
    analogOutputs[point.address] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 0.0, 0};

    return true;
  }

  //this function interacts with the message bus to store status information for tags so other modules can access it (binary)
  void Server::WriteBinary(std::uint16_t address, bool status) {
    auto iter = binaryOutputs.find(address);
    if (iter == binaryOutputs.end()) {
      return;
    }

    std::cout << fmt::format("[{}] setting tag {} to {}", config.id, iter->second.tag, status) << std::endl;

    otsim::msgbus::Points points;
    points.push_back(otsim::msgbus::Point{iter->second.tag, status ? 1.0 : 0.0});

    otsim::msgbus::Update contents = {.updates = points};
    auto env = otsim::msgbus::NewEnvelope(config.id, contents);

    pusher->Push("RUNTIME", env);
    metrics->IncrMetric("update_count");
  }

  //this function interacts with the message bus to store status information for tags so other modules can access it (analog)
  void Server::WriteAnalog(std::uint16_t address, double value) {
    auto iter = analogOutputs.find(address);
    if (iter == analogOutputs.end()) {
      return;
    }

    std::cout << fmt::format("[{}] setting tag {} to {}", config.id, iter->second.tag, value) << std::endl;

    otsim::msgbus::Points points;
    points.push_back(otsim::msgbus::Point{iter->second.tag, value});

    otsim::msgbus::Update contents = {.updates = points};
    auto env = otsim::msgbus::NewEnvelope(config.id, contents);

    pusher->Push("RUNTIME", env);
    metrics->IncrMetric("update_count");
  }

  const BinaryOutputPoint* Server::GetBinaryOutput(const uint16_t address) {
    auto iter = binaryOutputs.find(address);
    if (iter == binaryOutputs.end()) {
      return NULL;
    }

    //if the function hasn't returned, it must've found the output, so it returns the value
    return &iter->second;
  }

  const AnalogOutputPoint* Server::GetAnalogOutput(const uint16_t address) {
    auto iter = analogOutputs.find(address);
    if (iter == analogOutputs.end()) {
      return NULL;
    }

    //if the function hasn't returned, it must've found the output, so it returns the value
    return &iter->second;
  }

  /*
  * set all outputs to new points with zero values, create a new envelope with those points
  * and then push that envelope with the pusher
  */
  void Server::ResetOutputs() {
    otsim::msgbus::Points points;

    for (const auto& kv : binaryOutputs) {
      points.push_back(otsim::msgbus::Point{kv.second.tag, 0.0});
    }

    for (const auto& kv : analogOutputs) {
      points.push_back(otsim::msgbus::Point{kv.second.tag, 0.0});
    }

    if (points.size()) {
      std::cout << fmt::format("[{}] setting outputs to zero values", config.id) << std::endl;

      otsim::msgbus::Update contents = {.updates = points};
      auto env = otsim::msgbus::NewEnvelope(config.id, contents);

      pusher->Push("RUNTIME", env);
    }
  }

  void Server::HandleMsgBusStatus(const otsim::msgbus::Envelope<otsim::msgbus::Status>& env) {
    auto sender = otsim::msgbus::GetEnvelopeSender(env);
    
    //if the status sender is the current s7 device, return because the status does not need to be handled
    if (sender == config.id) {
      return;
    }

    //increment status count
    metrics->IncrMetric("status_count");

    //add each point in measurements to the points array based on tag
    for (auto &p : env.contents.measurements) {
      if (points.count(p.tag)) {
        std::cout << fmt::format("[{}] status received for tag {}", config.id, p.tag) << std::endl;

        auto lock = std::unique_lock<std::mutex>(pointsMu);
        points[p.tag] = p;
      }
    }
  }

} // namespace s7
} // namespace otsim
