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
    else if (area == S7AreaTM) area = srvAreaTM;
    else if (area == S7AreaCT) area = srvAreaCT;

    auto server = reinterpret_cast<Server*>(usrPtr);
    server->OnClientWrite(area, PTag->DBNumber, PTag->Start, PTag->Size);

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

  //writes analog to the buffer
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

    // Initialize first 16 bytes of each buffer with test values
    for (int i = 0; i < 16; ++i) {
      tBuffer[i] = static_cast<byte>(i + 49);  // T0..T15: 49,50,51,...,64
      zBuffer[i] = static_cast<byte>(i + 65);  // Z0..Z15: 65,66,67,...,80
    }
    std::cout << "[S7] Buffers initialized with test values." << std::endl;

    //register memory buffers for PA and DB areas
    //PE, PA, DB, and MK are different types of Siemens memory areas
    int mkResult = ts7server->RegisterArea(srvAreaMK, 0, paBuffer, sizeof(paBuffer));
    if (mkResult != 0) {
      std::cerr << "[S7] Failed to register MK area! Error code: " << mkResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(mkResult) << std::endl;
      return;
    }
    int peResult = ts7server->RegisterArea(srvAreaPE, 0, ebBuffer, sizeof(ebBuffer));
    if (peResult != 0) {
      std::cerr << "[S7] Failed to register PE area! Error code: " << peResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(peResult) << std::endl;
      return;
    }
    int paResult = ts7server->RegisterArea(srvAreaPA, 0, abBuffer, sizeof(abBuffer));
    if (paResult != 0) {
      std::cerr << "[S7] Failed to register PA area! Error code: " << paResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(paResult) << std::endl;
      return;
    }
    int tmResult = ts7server->RegisterArea(srvAreaTM, 0, tBuffer, sizeof(tBuffer));
    if (tmResult != 0) {
      std::cerr << "[S7] Failed to register TM area! Error code: " << tmResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(tmResult) << std::endl;
      return;
    }
    int ctResult = ts7server->RegisterArea(srvAreaCT, 0, zBuffer, sizeof(zBuffer));
    if (ctResult != 0) {
      std::cerr << "[S7] Failed to register CT area! Error code: " << ctResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(ctResult) << std::endl;
      return;
    }
    int dbResult = ts7server->RegisterArea(srvAreaDB, 1, dbBuffer, sizeof(dbBuffer));
    if (dbResult != 0) {
      std::cerr << "[S7] Failed to register DB area! Error code: " << dbResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(dbResult) << std::endl;
      return;
    }
    // Register additional DB areas for EB, AB, T, Z if client uses DB numbers
    int db2Result = ts7server->RegisterArea(srvAreaDB, 2, ebBuffer, sizeof(ebBuffer));
    if (db2Result != 0) {
      std::cerr << "[S7] Failed to register DB2 area! Error code: " << db2Result << std::endl;
      std::cerr << "[S7] " << SrvErrorText(db2Result) << std::endl;
      return;
    }
    int db3Result = ts7server->RegisterArea(srvAreaDB, 3, abBuffer, sizeof(abBuffer));
    if (db3Result != 0) {
      std::cerr << "[S7] Failed to register DB3 area! Error code: " << db3Result << std::endl;
      std::cerr << "[S7] " << SrvErrorText(db3Result) << std::endl;
      return;
    }
    int db4Result = ts7server->RegisterArea(srvAreaDB, 4, tBuffer, sizeof(tBuffer));
    if (db4Result != 0) {
      std::cerr << "[S7] Failed to register DB4 area! Error code: " << db4Result << std::endl;
      std::cerr << "[S7] " << SrvErrorText(db4Result) << std::endl;
      return;
    }
    int db5Result = ts7server->RegisterArea(srvAreaDB, 5, zBuffer, sizeof(zBuffer));
    if (db5Result != 0) {
      std::cerr << "[S7] Failed to register DB5 area! Error code: " << db5Result << std::endl;
      std::cerr << "[S7] " << SrvErrorText(db5Result) << std::endl;
      return;
    }

    int cbResult = ts7server->SetRWAreaCallback(rwCallback, this);
    if (cbResult != 0) {
      std::cerr << "[S7] Failed to register RW-area callback! Error code: " << cbResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(cbResult) << std::endl;
    }

    //start server after registering areas
    if (ts7server->Start() != 0) {
        std::cerr << "[S7] Failed to start Snap7 server!" << std::endl;
        return;
    }

    //debugging output
    std::cout << "[S7] Server started and memory areas registered." << std::endl;

    //this is the main running loop, it scans the subscribed points and writes them to memory
    while (running) {
      std::unique_lock<std::mutex> lock(pointsMu);

      // --- TEST PATCH: write MB0..MB15 non-zero values ---
      for (int i = 0; i < 16; ++i) {
          paBuffer[i] = static_cast<byte>(i + 1);  // MB0=1, MB1=2, ..., MB15=16
          std::cout << fmt::format("[{}] test write MB{} = {}", config.id, i, paBuffer[i]) << std::endl;
      }
      // ------------------------------------------------------

      //write binary inputs S7 memory
      for (auto& kv : binaryInputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] binaryInputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteBinaryToS7(paBuffer, sizeof(this->paBuffer), addr, point.value != 0);
        WriteBinaryToS7(ebBuffer, sizeof(this->ebBuffer), addr, point.value != 0);
        //log that we updated an input
        std::cout << fmt::format("[{}] updated binary input {} to {}", config.id, addr, point.value) << std::endl;
        metrics->IncrMetric("s7_binary_write_count");
      }

      //write binary outputs to S7 memory
      for (auto& kv : binaryOutputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] binaryOutputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteBinaryToS7(abBuffer, sizeof(this->abBuffer), addr, point.value != 0);
        std::cout << fmt::format("[{}] updated binary output {} to {}", config.id, addr, point.value) << std::endl;
        WriteBinary(addr, point.value != 0);
        metrics->IncrMetric("s7_binary_write_count");
      }

      //write analog inputs to S7 memory
      for (auto& kv : analogInputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] analogInputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteAnalogToS7(dbBuffer, sizeof(this->dbBuffer), addr, static_cast<float>(point.value));
        std::cout << fmt::format("[{}] updated____analog input {} to {}", config.id, addr, point.value) << std::endl;
        metrics->IncrMetric("s7_analog_write_count");
      }

      //write analog outputs to S7 memory
      for (auto& kv : analogOutputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] analogOutputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteAnalogToS7(dbBuffer, sizeof(this->dbBuffer), addr, static_cast<float>(point.value));
        std::cout << fmt::format("[{}] updated analog output {} to {}", config.id, addr, point.value) << std::endl;
        WriteAnalog(addr, point.value);
        metrics->IncrMetric("s7_analog_write_count");
      }

      lock.unlock();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  //when a client writes into server memory this function updates the message bus
  void Server::OnClientWrite(int area, int dbNumber, int start, int size) {
    std::unique_lock<std::mutex> lock(pointsMu);

    //access the registered buffers
    if (area == srvAreaMK) {
      if (static_cast<size_t>(start) < sizeof(paBuffer)) {
        bool val = paBuffer[start] != 0;
        if (binaryOutputs.find(start) == binaryOutputs.end()) {
          std::cerr << "[S7] OnClientWrite: binaryOutputs not found for start=" << start << std::endl;
          return;
        }
        const auto& tag = binaryOutputs[start].tag;
        if (points.find(tag) == points.end()) {
          std::cerr << "[S7] OnClientWrite: points not found for tag=" << tag << std::endl;
          return;
        }
        points[tag].value = val ? 1.0 : 0.0;
        WriteBinary(start, val);
      } else {
        std::cerr << "[S7] OnClientWrite MK out of bounds: start=" << start << std::endl;
      }
    } else if (area == srvAreaPE) {
      if (static_cast<size_t>(start) < sizeof(ebBuffer)) {
        bool val = ebBuffer[start] != 0;
        // For PE, perhaps map to binaryInputs or something, but for now, maybe not handle
        std::cerr << "[S7] OnClientWrite PE not implemented" << std::endl;
      } else {
        std::cerr << "[S7] OnClientWrite PE out of bounds: start=" << start << std::endl;
      }
    } else if (area == srvAreaPA) {
      if (static_cast<size_t>(start) < sizeof(abBuffer)) {
        bool val = abBuffer[start] != 0;
        if (binaryOutputs.find(start) == binaryOutputs.end()) {
          std::cerr << "[S7] OnClientWrite: binaryOutputs not found for start=" << start << std::endl;
          return;
        }
        const auto& tag = binaryOutputs[start].tag;
        if (points.find(tag) == points.end()) {
          std::cerr << "[S7] OnClientWrite: points not found for tag=" << tag << std::endl;
          return;
        }
        points[tag].value = val ? 1.0 : 0.0;
        WriteBinary(start, val);
      } else {
        std::cerr << "[S7] OnClientWrite PA out of bounds: start=" << start << std::endl;
      }
    } else if (area == srvAreaTM) {
      // Timers, special handling
      std::cerr << "[S7] OnClientWrite TM not implemented" << std::endl;
    } else if (area == srvAreaCT) {
      // Counters, special handling
      std::cerr << "[S7] OnClientWrite CT not implemented" << std::endl;
    } else if (area == srvAreaDB) {
      if (static_cast<size_t>(start) + sizeof(float) <= sizeof(dbBuffer)) {
        float val;
        memcpy(&val, &dbBuffer[start], sizeof(float));
        if (analogOutputs.find(start) == analogOutputs.end()) {
          std::cerr << "[S7] OnClientWrite: analogOutputs not found for start=" << start << std::endl;
          return;
        }
        const auto& tag = analogOutputs[start].tag;
        if (points.find(tag) == points.end()) {
          std::cerr << "[S7] OnClientWrite: points not found for tag=" << tag << std::endl;
          return;
        }
        points[tag].value = val;
        WriteAnalog(start, val);
      } else {
        std::cerr << "[S7] OnClientWrite DB out of bounds: start=" << start << std::endl;
      }
    }
  }


  bool Server::AddBinaryInput(BinaryInputPoint point) {
    if (nextBinInAddr >= sizeof(paBuffer)) {
      std::cerr << "[S7] AddBinaryInput: buffer full" << std::endl;
      return false;
    }
    binaryInputs[nextBinInAddr] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 1.0, 0};
    nextBinInAddr++;

    return true;
  }

  bool Server::AddBinaryOutput(BinaryOutputPoint point) {
    if (nextBinOutAddr >= sizeof(abBuffer)) {
      std::cerr << "[S7] AddBinaryOutput: buffer full" << std::endl;
      return false;
    }
    point.output = true;
    binaryOutputs[nextBinOutAddr] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 1.0, 0};
    nextBinOutAddr++;

    return true;
  }

  bool Server::AddAnalogInput(AnalogInputPoint point) {
    if (nextAnaInAddr + sizeof(float) > sizeof(dbBuffer)) {
      std::cerr << "[S7] AddAnalogInput: buffer full" << std::endl;
      return false;
    }
    analogInputs[nextAnaInAddr] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 1.0, 0};
    nextAnaInAddr += sizeof(float);

    return true;
  }

  bool Server::AddAnalogOutput(AnalogOutputPoint point) {
    if (nextAnaOutAddr + sizeof(float) > sizeof(dbBuffer)) {
      std::cerr << "[S7] AddAnalogOutput: buffer full" << std::endl;
      return false;
    }
    point.output = true;
    analogOutputs[nextAnaOutAddr] = point;
    points[point.tag] = otsim::msgbus::Point{point.tag, 1.0, 0};
    nextAnaOutAddr += sizeof(float);

    return true;
  }

  //this function interacts with the message bus to store status information for tags so other modules can access it (binary)
  void Server::WriteBinary(std::uint16_t address, bool status) {
    if (!pusher) return;
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
    if (metrics) metrics->IncrMetric("update_count");
  }

  //this function interacts with the message bus to store status information for tags so other modules can access it (analog)
  void Server::WriteAnalog(std::uint16_t address, double value) {
    if (!pusher) return;
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
    if (metrics) metrics->IncrMetric("update_count");
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
