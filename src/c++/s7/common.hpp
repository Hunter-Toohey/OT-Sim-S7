#ifndef OTSIM_S7_COMMON_HPP
#define OTSIM_S7_COMMON_HPP

#include "msgbus/metrics.hpp"
#include "msgbus/pusher.hpp"

namespace otsim {
namespace s7 {

/*
 * Point structure for S7 communication
 * Represents a single data point (binary or analog) that can be
 * read from or written to via S7 protocol
 */
template <typename T>
struct Point {
  std::uint16_t address {};  // Memory address in PLC address space
  std::string   tag     {};  // Unique identifier for message bus routing
  T value {};                // Current value of the point
  bool output {};            // True if this is an output point (writable by clients)
  bool sbo {};               // Select Before Operate - requires two-step write
  double deadband {};        // Deadband for analog values (prevents excessive updates)
};

using BinaryInputPoint  = Point<bool>;   // Digital input (client reads from server)
using BinaryOutputPoint = Point<bool>;   // Digital output (client writes to server)

using AnalogInputPoint  = Point<float>;  // Analog input (client reads from server)
using AnalogOutputPoint = Point<float>;  // Analog output (client writes to server)

using Pusher = std::shared_ptr<otsim::msgbus::Pusher>;
using MetricsPusher = std::shared_ptr<otsim::msgbus::MetricsPusher>;

/*
 * Block operation types for snap7 upload/download
 * These are used internally by the snap7 library to handle
 * block-related operations (directory listing, block info queries, etc.)
 */
enum class BlockOperationType {
  LIST_ALL,        // List all blocks in PLC
  LIST_BY_TYPE,    // List blocks of specific type (DB, FB, FC, etc.)
  GET_INFO,        // Get detailed info about a specific block
  UPLOAD,          // Upload block from PLC (returns need-password in implementation)
  DOWNLOAD         // Download block to PLC (returns need-password in implementation)
};

} // namespace s7
} // namespace otsim

#endif // OTSIM_S7_COMMON_HPP