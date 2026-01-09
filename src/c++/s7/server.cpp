#include <iostream>
#include <cstring>
#include "server.hpp"
#include "fmt/format.h"
#include "msgbus/metrics.hpp"
#include "snap7.h"

namespace otsim {
namespace s7 {
  //when a client talks to a server, this function packages that data into a format understandle by the server
  int S7API rwCallback(void *usrPtr, int Sender, int Operation, PS7Tag PTag, void *pUsrData) {
    if (!PTag) return 0;

    int area = PTag->Area;
    if (area == S7AreaMK) area = srvAreaMK;
    else if (area == S7AreaDB) area = srvAreaDB;
    else if (area == S7AreaPE) area = srvAreaPE;
    else if (area == S7AreaPA) area = srvAreaPA;

    Server::OnClientWrite(area, PTag->DBNumber, PTag->Start, PTag->Size, usrPtr);

    return 0;
  }

  //writes a binary to the buffer
  void WriteBinaryToS7(byte* buffer, size_t bufLen, std::uint16_t addr, bool value) {
  if (buffer && addr < bufLen) {
    buffer[addr] = value ? 1 : 0;
  } else {
    std::cerr << "[S7] Binary write out of bounds: addr=" << addr << " bufLen=" << bufLen << std::endl;
  }
  }

  //writes analog to the buffer (as 4-byte float at byte-aligned address)
  void WriteAnalogToS7(byte* buffer, size_t bufLen, std::uint16_t addr, float value) {
    if (buffer && addr + sizeof(float) <= bufLen) {
      memcpy(&buffer[addr], &value, sizeof(float));
    } else {
      std::cerr << "[S7] Analog write out of bounds: addr=" << addr << " bufLen=" << bufLen << std::endl;
    }
  }

  //constructor, initializer metrics
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
  
  //run the server loop and update memory
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
    //PE = process Inputs (PIB: digital inputs + PIW: analog inputs)
    //PA = process Outputs (PQB: digital outputs + PQW: analog outputs)
    //MK = merker (internal flags/markers for intermediate calculations)
    //DB = data blocks
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

    //debugging output
    std::cout << "[S7] Server started, memory areas registered, and callbacks configured." << std::endl;

    //this is the main running loop, it scans the subscribed points and writes them to memory
    while (running) {
      std::unique_lock<std::mutex> lock(pointsMu);

      //write binary inputs to PE area (PIB - process input bytes, bytes 0-255)
      for (auto& kv : binaryInputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] binaryInputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteBinaryToS7(peBuffer, sizeof(peBuffer), addr, point.value != 0);
        std::cout << fmt::format("[{}] updated binary input PIB.{} to {}", config.id, addr, point.value) << std::endl;
        metrics->IncrMetric("s7_binary_write_count");
      }

      //write binary outputs to PA area (PQB - process output bytes, bytes 0-255)
      for (auto& kv : binaryOutputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] binaryOutputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteBinaryToS7(paBuffer, sizeof(paBuffer), addr, point.value != 0);  // addr is already 0-255
        std::cout << fmt::format("[{}] updated binary output PQB.{} to {}", config.id, addr, point.value) << std::endl;
        WriteBinary(addr, point.value != 0);
        metrics->IncrMetric("s7_binary_write_count");
      }

      //write analog inputs to PE area (PIW - process input words, bytes 256+)
      for (auto& kv : analogInputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] analogInputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteAnalogToS7(peBuffer, sizeof(peBuffer), ANALOG_OFFSET + addr, static_cast<float>(point.value));
        std::cout << fmt::format("[{}] updated analog input PIW.{} to {}", config.id, addr, point.value) << std::endl;
        metrics->IncrMetric("s7_analog_write_count");
      }

      //write analog outputs to PA area (PQW - process output words, bytes 256+)
      for (auto& kv : analogOutputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] analogOutputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteAnalogToS7(paBuffer, sizeof(paBuffer), ANALOG_OFFSET + addr, static_cast<float>(point.value));
        std::cout << fmt::format("[{}] updated analog output PQW.{} to {}", config.id, addr, point.value) << std::endl;
        WriteAnalog(addr, point.value);
        metrics->IncrMetric("s7_analog_write_count");
      }

      lock.unlock();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  //when a client writes into server memory this function updates the message bus
  void Server::OnClientWrite(int area, int dbNumber, int start, int size, void* usrPtr) {
    auto server = reinterpret_cast<Server*>(usrPtr);
    std::unique_lock<std::mutex> lock(server->pointsMu);

    //handle PA area writes (both binary and analog outputs)
    if (area == srvAreaPA) {
      //check if this is binary output write (PQB, bytes 0-255)
      if (start >= server->BINARY_OFFSET && start < server->BINARY_OFFSET + server->BINARY_SIZE) {
        if (static_cast<size_t>(start) < sizeof(server->paBuffer)) {
          uint16_t addr = start - server->BINARY_OFFSET;
          bool val = server->paBuffer[start] != 0;
          if (server->binaryOutputs.find(addr) == server->binaryOutputs.end()) {
            std::cerr << "[S7] OnClientWrite PA: binaryOutputs not found for PQB." << addr << std::endl;
            return;
          }
          const auto& tag = server->binaryOutputs[addr].tag;
          if (server->points.find(tag) == server->points.end()) {
            std::cerr << "[S7] OnClientWrite PA: points not found for tag=" << tag << std::endl;
            return;
          }
          server->points[tag].value = val ? 1.0 : 0.0;
          server->WriteBinary(addr, val);
          std::cout << fmt::format("[S7] Client wrote PQB.{} = {}", addr, val) << std::endl;
        }
      }
      //check if this is analog output write (PQW, bytes 256+)
      else if (start >= server->ANALOG_OFFSET && start + sizeof(float) <= sizeof(server->paBuffer)) {
        uint16_t addr = start - server->ANALOG_OFFSET;
        float val;
        memcpy(&val, &server->paBuffer[start], sizeof(float));
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
        std::cout << fmt::format("[S7] Client wrote PQW.{} = {}", addr, val) << std::endl;
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
