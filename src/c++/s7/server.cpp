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
  /*
   * This function initializes and runs the S7 server with full snap7 functionality:
   * 
   * Block Operations Support:
   * - Block Upload: Clients can request block info and directory listings. Actual block
   *   uploads return "need password" error (handled by underlying snap7 library).
   * - Block Download: Similar to upload - directory operations work, downloads are blocked.
   * - Block Info Queries: Fully supported - clients can query registered DB blocks.
   * - Async Operations: The underlying snap7 library handles async block operations via
   *   its worker thread architecture.
   * 
   * Callback Architecture:
   * - OnServerEvent: Monitors server lifecycle and block operation events
   * - OnReadEvent: Tracks all read operations for debugging/monitoring
   * - rwCallback: Handles real-time read/write from clients
   * - OnRWAreaCallback: Provides ResourceLess mode support for dynamic data
   * 
   * Event Handling:
   * - Server events (start/stop, client connect/disconnect)
   * - Block operation events (upload/download requests, directory queries)
   * - Data access events (read/write operations)
   * 
   * The server properly handles:
   * - Multiple concurrent clients
   * - PDU negotiation
   * - Memory area registration (MK, PA, DB areas)
   * - Thread-safe data access via mutexes
   */
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
    //register memory buffers for PE, PA, MK, and DB areas matching real PLC architecture
    //PE = Process Inputs (PIB: digital inputs + PIW: analog inputs)
    //PA = Process Outputs (PQB: digital outputs + PQW: analog outputs)
    //MK = Merker (internal flags/markers for intermediate calculations)
    //DB = Data Blocks (structured data, recipes, parameters - NOT for I/O)
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
    int cbResult = ts7server->SetRWAreaCallback(rwCallback, this);
    if (cbResult != 0) {
      std::cerr << "[S7] Failed to register RW-area callback! Error code: " << cbResult << std::endl;
      std::cerr << "[S7] " << SrvErrorText(cbResult) << std::endl;
    }

    // Add test data to buffers for debugging client reads
    std::cout << "[S7] Writing test data to buffers..." << std::endl;
    peBuffer[0] = 0xFF;  // EB0 (PE byte 0)
    peBuffer[1] = 0xAA;  // EB1
    peBuffer[2] = 0x55;  // EB2
    paBuffer[0] = 0x11;  // AB0 (PA byte 0)
    paBuffer[1] = 0x22;  // AB1
    paBuffer[2] = 0x33;  // AB2
    mkBuffer[0] = 0xCC;  // MB0 (MK byte 0)
    mkBuffer[1] = 0xDD;  // MB1
    mkBuffer[2] = 0xEE;  // MB2
    std::cout << "[S7] Test data: EB=FF,AA,55 AB=11,22,33 MB=CC,DD,EE" << std::endl;

    // Register server event callbacks for monitoring and logging
    int evtResult = ts7server->SetEventsCallback(OnServerEvent, this);
    if (evtResult != 0) {
      std::cerr << "[S7] Failed to register events callback! Error code: " << evtResult << std::endl;
    }

    // Register read event callback for monitoring read operations
    int readEvtResult = ts7server->SetReadEventsCallback(OnReadEvent, this);
    if (readEvtResult != 0) {
      std::cerr << "[S7] Failed to register read events callback! Error code: " << readEvtResult << std::endl;
    }

    // Enable important server events
    // Enable block upload/download, directory operations, and client events
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

      //write binary inputs to PE area (PIB - Process Input Bytes, bytes 0-255)
      for (auto& kv : binaryInputs) {
        const auto& addr = kv.first;
        if (points.find(kv.second.tag) == points.end()) {
          std::cerr << "[S7] binaryInputs: tag not found: " << kv.second.tag << std::endl;
          continue;
        }
        auto& point = points[kv.second.tag];
        WriteBinaryToS7(peBuffer, sizeof(peBuffer), addr, point.value != 0);  // addr is already 0-255
        std::cout << fmt::format("[{}] updated binary input PIB.{} to {}", config.id, addr, point.value) << std::endl;
        metrics->IncrMetric("s7_binary_write_count");
      }

      // Debug: Print PE buffer contents - binary section (first 32 bytes)
      std::string bufferHex;
      for (int i = 0; i < 32; i++) {
        bufferHex += fmt::format("{:02X} ", peBuffer[i]);
        if ((i + 1) % 16 == 0) bufferHex += "\n                ";
      }
      std::cout << fmt::format("[{}] PE Buffer (PIB 0-31): {}", config.id, bufferHex) << std::endl;

      //write binary outputs to PA area (PQB - Process Output Bytes, bytes 0-255)
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

      // Debug: Print PA buffer contents - binary section (first 32 bytes)
      std::string paBufferHex;
      for (int i = 0; i < 32; i++) {
        paBufferHex += fmt::format("{:02X} ", paBuffer[i]);
        if ((i + 1) % 16 == 0) paBufferHex += "\n                ";
      }
      std::cout << fmt::format("[{}] PA Buffer (PQB 0-31): {}", config.id, paBufferHex) << std::endl;

      //write analog inputs to PE area (PIW - Process Input Words, bytes 256+)
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

      // Debug: Print PE buffer contents - analog section (bytes 256-287)
      std::string peAnalogHex;
      for (int i = ANALOG_OFFSET; i < ANALOG_OFFSET + 32; i++) {
        peAnalogHex += fmt::format("{:02X} ", peBuffer[i]);
        if ((i - ANALOG_OFFSET + 1) % 16 == 0) peAnalogHex += "\n                ";
      }
      std::cout << fmt::format("[{}] PE Buffer (PIW 256-287): {}", config.id, peAnalogHex) << std::endl;

      //write analog outputs to PA area (PQW - Process Output Words, bytes 256+)
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

      // Debug: Print PA buffer contents - analog section (bytes 256-287)
      std::string paAnalogHex;
      for (int i = ANALOG_OFFSET; i < ANALOG_OFFSET + 32; i++) {
        paAnalogHex += fmt::format("{:02X} ", paBuffer[i]);
        if ((i - ANALOG_OFFSET + 1) % 16 == 0) paAnalogHex += "\n                ";
      }
      std::cout << fmt::format("[{}] PA Buffer (PQW 256-287): {}", config.id, paAnalogHex) << std::endl;

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
      // Check if this is binary output write (PQB, bytes 0-255)
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
      // Check if this is analog output write (PQW, bytes 256+)
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

  // Server event callback handler - handles general server events
  /*
   * This callback is invoked by snap7 for major server events including:
   * - evcServerStarted/Stopped: Server lifecycle events
   * - evcClientAdded/Disconnected: Client connection events
   * - evcUpload: Block upload requests (logged, returns need-password by library)
   * - evcDownload: Block download requests (logged, returns need-password by library)
   * - evcDirectory: Directory/block info queries (fully functional)
   * - evcNegotiatePDU: PDU size negotiation with clients
   * 
   * The underlying snap7 library handles block upload/download via:
   * - PerformFunctionUpload(): Returns Code7NeedPassword to prevent block extraction
   * - PerformFunctionDownload(): Returns Code7NeedPassword to prevent block injection  
   * - PerformGroupBlockInfo(): Provides full directory listing of registered DBs
   * 
   * This allows clients to browse the PLC directory structure but not extract/inject code.
   */
  void Server::OnServerEvent(void* usrPtr, PSrvEvent pEvent, int size) {
    if (!pEvent || !usrPtr) return;
    
    auto server = reinterpret_cast<Server*>(usrPtr);
    
    // Log important server events
    switch (pEvent->EvtCode) {
      case evcServerStarted:
        std::cout << fmt::format("[{}] Server started", server->config.id) << std::endl;
        break;
      case evcServerStopped:
        std::cout << fmt::format("[{}] Server stopped", server->config.id) << std::endl;
        break;
      case evcClientAdded:
        std::cout << fmt::format("[{}] Client connected (ID: {})", server->config.id, pEvent->EvtSender) << std::endl;
        break;
      case evcClientDisconnected:
        std::cout << fmt::format("[{}] Client disconnected (ID: {})", server->config.id, pEvent->EvtSender) << std::endl;
        break;
      case evcUpload:
        std::cout << fmt::format("[{}] Block upload requested (Result: 0x{:04X})", 
                  server->config.id, pEvent->EvtRetCode) << std::endl;
        break;
      case evcDownload:
        std::cout << fmt::format("[{}] Block download requested (Result: 0x{:04X})", 
                  server->config.id, pEvent->EvtRetCode) << std::endl;
        break;
      case evcDirectory:
        std::cout << fmt::format("[{}] Directory operation (SubCode: 0x{:04X})", 
                  server->config.id, pEvent->EvtParam1) << std::endl;
        break;
      default:
        // Log other events at debug level if needed
        break;
    }
  }

  // Read event callback handler - handles read operations for monitoring
  /*
   * This callback is invoked for data read operations, allowing monitoring of:
   * - Which areas are being read (PA, MK, DB, etc.)
   * - Read address ranges and sizes
   * - Read operation results
   * 
   * Useful for debugging, security monitoring, and performance analysis.
   */
  void Server::OnReadEvent(void* usrPtr, PSrvEvent pEvent, int size) {
    if (!pEvent || !usrPtr) return;
    
    auto server = reinterpret_cast<Server*>(usrPtr);
    
    // Log read events for debugging
    if (pEvent->EvtCode == evcDataRead) {
      std::cout << fmt::format("[{}] Data read - Area: {}, Index: {}, Start: {}, Size: {}", 
                server->config.id, pEvent->EvtParam1, pEvent->EvtParam2, 
                pEvent->EvtParam3, pEvent->EvtParam4) << std::endl;
    }
  }

  // Read/Write area callback for ResourceLess mode
  /*
   * This callback enables "ResourceLess" mode where the server doesn't need to
   * pre-register memory buffers. Instead, it provides data on-demand via this callback.
   * 
   * Supports:
   * - Async read operations: Client reads trigger this callback to get current data
   * - Async write operations: Client writes trigger this callback to update data
   * - Polling operations: Clients can poll areas without pre-allocated buffers
   * - Block operations: Underlying library uses this for block read/write if enabled
   * 
   * Operation flow:
   * 1. Client sends read/write request
   * 2. Snap7 library validates request and calls this callback
   * 3. Callback provides/receives data directly from application state
   * 4. Library completes the response to client
   * 
   * This is complementary to the rwCallback which is used for simpler operations.
   */
  int Server::OnRWAreaCallback(void* usrPtr, int sender, int operation, PS7Tag pTag, void* pUsrData) {
    if (!pTag || !usrPtr) return -1;
    
    auto server = reinterpret_cast<Server*>(usrPtr);
    std::unique_lock<std::mutex> lock(server->pointsMu);
    
    // Handle the operation based on type (Read or Write)
    if (operation == OperationRead) {
      // Client is reading from server - provide data
      std::cout << fmt::format("[{}] Read callback - Area: 0x{:02X}, DB: {}, Start: {}, Size: {}", 
                server->config.id, pTag->Area, pTag->DBNumber, pTag->Start, pTag->Size) << std::endl;
      
      // Provide the data based on area type
      if (pTag->Area == S7AreaPE) {
        // PE area contains both binary inputs (PIB) and analog inputs (PIW)
        if (pTag->Start < (int)sizeof(server->peBuffer)) {
          memcpy(pUsrData, &server->peBuffer[pTag->Start], 
                 std::min(pTag->Size, (int)(sizeof(server->peBuffer) - pTag->Start)));
          if (pTag->Start < server->ANALOG_OFFSET) {
            std::cout << fmt::format("[{}] Read PIB (binary input) at offset {}", 
                      server->config.id, pTag->Start) << std::endl;
          } else {
            std::cout << fmt::format("[{}] Read PIW (analog input) at offset {}", 
                      server->config.id, pTag->Start) << std::endl;
          }
          return 0; // Success
        }
      } else if (pTag->Area == S7AreaPA) {
        // PA area contains both binary outputs (PQB) and analog outputs (PQW)
        if (pTag->Start < (int)sizeof(server->paBuffer)) {
          memcpy(pUsrData, &server->paBuffer[pTag->Start], 
                 std::min(pTag->Size, (int)(sizeof(server->paBuffer) - pTag->Start)));
          if (pTag->Start < server->ANALOG_OFFSET) {
            std::cout << fmt::format("[{}] Read PQB (binary output) at offset {}", 
                      server->config.id, pTag->Start) << std::endl;
          } else {
            std::cout << fmt::format("[{}] Read PQW (analog output) at offset {}", 
                      server->config.id, pTag->Start) << std::endl;
          }
          return 0; // Success
        }
      } else if (pTag->Area == S7AreaDB) {
        // DB area for structured data
        if (pTag->Start < (int)sizeof(server->dbBuffer)) {
          memcpy(pUsrData, &server->dbBuffer[pTag->Start], 
                 std::min(pTag->Size, (int)(sizeof(server->dbBuffer) - pTag->Start)));
          std::cout << fmt::format("[{}] Read DB{} at offset {}", 
                    server->config.id, pTag->DBNumber, pTag->Start) << std::endl;
          return 0; // Success
        }
      }
      
      return -1; // Out of range or unsupported area
      
    } else if (operation == OperationWrite) {
      // Client is writing to server - process the write
      std::cout << fmt::format("[{}] Write callback - Area: 0x{:02X}, DB: {}, Start: {}, Size: {}", 
                server->config.id, pTag->Area, pTag->DBNumber, pTag->Start, pTag->Size) << std::endl;
      
      // Process write based on area type
      if (pTag->Area == S7AreaPA) {
        // PA area contains both binary outputs (PQB) and analog outputs (PQW)
        if (pTag->Start < (int)sizeof(server->paBuffer)) {
          memcpy(&server->paBuffer[pTag->Start], pUsrData, 
                 std::min(pTag->Size, (int)(sizeof(server->paBuffer) - pTag->Start)));
          
          // Check if this is binary output write (PQB, bytes 0-255)
          if (pTag->Start >= server->BINARY_OFFSET && pTag->Start < server->BINARY_OFFSET + server->BINARY_SIZE) {
            uint16_t addr = pTag->Start - server->BINARY_OFFSET;
            if (server->binaryOutputs.find(addr) != server->binaryOutputs.end()) {
              bool val = server->paBuffer[pTag->Start] != 0;
              const auto& tag = server->binaryOutputs[addr].tag;
              if (server->points.find(tag) != server->points.end()) {
                server->points[tag].value = val ? 1.0 : 0.0;
                lock.unlock();
                server->WriteBinary(addr, val);
                std::cout << fmt::format("[{}] Client wrote PQB.{} = {}", 
                          server->config.id, addr, val) << std::endl;
              }
            }
          }
          // Check if this is analog output write (PQW, bytes 256+)
          else if (pTag->Start >= server->ANALOG_OFFSET && pTag->Start + (int)sizeof(float) <= (int)sizeof(server->paBuffer)) {
            uint16_t addr = pTag->Start - server->ANALOG_OFFSET;
            if (server->analogOutputs.find(addr) != server->analogOutputs.end()) {
              float val;
              memcpy(&val, &server->paBuffer[pTag->Start], sizeof(float));
              const auto& tag = server->analogOutputs[addr].tag;
              if (server->points.find(tag) != server->points.end()) {
                server->points[tag].value = val;
                lock.unlock();
                server->WriteAnalog(addr, val);
                std::cout << fmt::format("[{}] Client wrote PQW.{} = {}", 
                          server->config.id, addr, val) << std::endl;
              }
            }
          }
          return 0; // Success
        }
      } else if (pTag->Area == S7AreaDB) {
        // DB area for structured data
        if (pTag->Start < (int)sizeof(server->dbBuffer)) {
          memcpy(&server->dbBuffer[pTag->Start], pUsrData, 
                 std::min(pTag->Size, (int)(sizeof(server->dbBuffer) - pTag->Start)));
          std::cout << fmt::format("[{}] Client wrote DB{} at offset {}", 
                    server->config.id, pTag->DBNumber, pTag->Start) << std::endl;
          return 0; // Success
        }
      }
      
      return -1; // Out of range or unsupported area
    }
    
    return -1; // Unknown operation
  }

} // namespace s7
} // namespace otsim
