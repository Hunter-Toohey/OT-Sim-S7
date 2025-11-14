#ifndef OTSIM_S7_COMMON_HPP
#define OTSIM_S7_COMMON_HPP

#include "msgbus/metrics.hpp"
#include "msgbus/pusher.hpp"

namespace otsim {
namespace s7 {

template <typename T>
struct Point {
  std::uint16_t address {};
  std::string   tag     {};
  T value {};
  bool output {};
  bool sbo {};
  double deadband {};
};

using BinaryInputPoint  = Point<bool>;
using BinaryOutputPoint = Point<bool>;

using AnalogInputPoint  = Point<float>;
using AnalogOutputPoint = Point<float>;

using Pusher = std::shared_ptr<otsim::msgbus::Pusher>;
using MetricsPusher = std::shared_ptr<otsim::msgbus::MetricsPusher>;

} // namespace s7
} // namespace otsim

#endif // OTSIM_S7_COMMON_HPP