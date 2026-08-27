#include "hc_vr_gateway/codec.hpp"
#include "hc_vr_gateway/vr_gateway_node.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include "builtin_interfaces/msg/time.hpp"
#include "hc_teleop_interfaces/msg/controller_input.hpp"
#include "hc_teleop_interfaces/msg/tracked_pose.hpp"
#include "hc_teleop_interfaces/msg/vr_frame.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/header.hpp"

namespace hc_vr_gateway
{
namespace
{

using namespace std::chrono_literals;

constexpr char kDiscoveryRequest[] = "PICO_DISCOVER_V1";

std::runtime_error socket_error(const std::string & operation)
{
  return std::runtime_error(operation + ": " + std::strerror(errno));
}

int create_bound_socket(const std::string & host, int port)
{
  const int descriptor = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (descriptor < 0) {
    throw socket_error("socket");
  }

  try {
    int enabled = 1;
    if (::setsockopt(
        descriptor, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled)) < 0)
    {
      throw socket_error("setsockopt(SO_REUSEADDR)");
    }

    const int flags = ::fcntl(descriptor, F_GETFL, 0);
    if (flags < 0 || ::fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) < 0) {
      throw socket_error("fcntl(O_NONBLOCK)");
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1) {
      throw std::invalid_argument("listen_host must be an IPv4 address: " + host);
    }
    if (::bind(
        descriptor, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0)
    {
      throw socket_error("bind(" + host + ":" + std::to_string(port) + ")");
    }
    return descriptor;
  } catch (...) {
    ::close(descriptor);
    throw;
  }
}

std::string endpoint_address(const sockaddr_in & endpoint)
{
  std::array<char, INET_ADDRSTRLEN> address{};
  if (::inet_ntop(AF_INET, &endpoint.sin_addr, address.data(), address.size()) == nullptr) {
    return "unknown";
  }
  return std::string(address.data()) + ":" + std::to_string(ntohs(endpoint.sin_port));
}

bool same_endpoint(const sockaddr_in & left, const sockaddr_in & right) noexcept
{
  return left.sin_family == right.sin_family &&
         left.sin_addr.s_addr == right.sin_addr.s_addr &&
         left.sin_port == right.sin_port;
}

std::string_view trim_ascii_whitespace(std::string_view value) noexcept
{
  constexpr std::string_view whitespace{" \t\r\n"};
  const std::size_t first = value.find_first_not_of(whitespace);
  if (first == std::string_view::npos) {
    return {};
  }
  const std::size_t last = value.find_last_not_of(whitespace);
  return value.substr(first, last - first + 1U);
}

builtin_interfaces::msg::Time seconds_to_time(double value) noexcept
{
  builtin_interfaces::msg::Time result;
  if (!std::isfinite(value)) {
    return result;
  }

  const double seconds = std::floor(value);
  if (seconds < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
    seconds > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
  {
    return result;
  }
  result.sec = static_cast<std::int32_t>(seconds);
  double fractional = value - seconds;
  auto nanoseconds = static_cast<std::int64_t>(std::llround(fractional * 1.0e9));
  if (nanoseconds >= 1000000000LL) {
    if (result.sec == std::numeric_limits<std::int32_t>::max()) {
      return builtin_interfaces::msg::Time{};
    }
    ++result.sec;
    nanoseconds -= 1000000000LL;
  }
  result.nanosec = static_cast<std::uint32_t>(std::max<std::int64_t>(0LL, nanoseconds));
  return result;
}

void copy_pose(
  const Pose & source, bool tracked,
  const std_msgs::msg::Header & header,
  hc_teleop_interfaces::msg::TrackedPose & destination) noexcept
{
  destination.header = header;
  destination.tracking_state = tracked ?
    hc_teleop_interfaces::msg::TrackedPose::TRACKING_TRACKED :
    hc_teleop_interfaces::msg::TrackedPose::TRACKING_UNAVAILABLE;
  destination.pose.position.x = source.position[0];
  destination.pose.position.y = source.position[1];
  destination.pose.position.z = source.position[2];
  destination.pose.orientation.x = source.quaternion[0];
  destination.pose.orientation.y = source.quaternion[1];
  destination.pose.orientation.z = source.quaternion[2];
  destination.pose.orientation.w = source.quaternion[3];
  destination.velocity_valid = false;
  destination.confidence = tracked ? 1.0F : 0.0F;
}

void copy_input(
  const ControllerInput & source,
  hc_teleop_interfaces::msg::ControllerInput & destination) noexcept
{
  destination.held_mask = source.held_mask;
  destination.pressed_mask = source.pressed_mask;
  destination.released_mask = source.released_mask;
  destination.trigger = std::clamp(source.trigger, 0.0F, 1.0F);
  destination.grip = std::clamp(source.grip, 0.0F, 1.0F);
  for (std::size_t index = 0; index < source.primary_axis.size(); ++index) {
    destination.primary_axis[index] = std::clamp(source.primary_axis[index], -1.0F, 1.0F);
    destination.secondary_axis[index] = std::clamp(source.secondary_axis[index], -1.0F, 1.0F);
  }
}

}  // namespace

class VrGatewayNode::Impl
{
public:
  explicit Impl(VrGatewayNode & node)
  : node_(node)
  {
    listen_host_ = node_.declare_parameter<std::string>("listen_host", "0.0.0.0");
    pose_port_ = node_.declare_parameter<int>("pose_port", 5005);
    discovery_port_ = node_.declare_parameter<int>("discovery_port", 5006);
    timeout_ms_ = node_.declare_parameter<int>("timeout_ms", 600);
    output_topic_ = node_.declare_parameter<std::string>("output_topic", "input/vr_frame");
    frame_id_ = node_.declare_parameter<std::string>("frame_id", "vr_tracking");
    validate_parameters();

    publisher_ = node_.create_publisher<hc_teleop_interfaces::msg::VrFrame>(
      output_topic_, rclcpp::SensorDataQoS().keep_last(1));
    pose_socket_ = create_bound_socket(listen_host_, pose_port_);
    try {
      discovery_socket_ = create_bound_socket(listen_host_, discovery_port_);
    } catch (...) {
      ::close(pose_socket_);
      pose_socket_ = -1;
      throw;
    }
    discovery_response_ = "PICO_RECEIVER_V1|" + std::to_string(pose_port_);

    worker_ = std::thread([this]() {run();});
    RCLCPP_INFO(
      get_logger(),
      "VR UDP gateway listening on %s:%d (discovery %d), publishing %s",
      listen_host_.c_str(), pose_port_, discovery_port_, output_topic_.c_str());
  }

  ~Impl()
  {
    stop_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
    close_sockets();
    RCLCPP_INFO(
      get_logger(),
      "VR UDP gateway stopped: received=%llu published=%llu lost=%llu old=%llu invalid=%llu",
      static_cast<unsigned long long>(received_packets_),
      static_cast<unsigned long long>(published_packets_),
      static_cast<unsigned long long>(total_lost_packets_),
      static_cast<unsigned long long>(old_packets_),
      static_cast<unsigned long long>(invalid_packets_));
  }

private:
  [[nodiscard]] rclcpp::Logger get_logger() const
  {
    return node_.get_logger();
  }

  [[nodiscard]] rclcpp::Time now() const
  {
    return node_.now();
  }

  void validate_parameters() const
  {
    const auto valid_port = [](int port) {return port > 0 && port <= 65535;};
    if (!valid_port(pose_port_) || !valid_port(discovery_port_)) {
      throw std::invalid_argument("pose_port and discovery_port must be in [1, 65535]");
    }
    if (pose_port_ == discovery_port_) {
      throw std::invalid_argument("pose_port and discovery_port must be different");
    }
    if (timeout_ms_ <= 0) {
      throw std::invalid_argument("timeout_ms must be positive");
    }
    if (output_topic_.empty()) {
      throw std::invalid_argument("output_topic must not be empty");
    }
  }

  void close_sockets() noexcept
  {
    if (pose_socket_ >= 0) {
      ::close(pose_socket_);
      pose_socket_ = -1;
    }
    if (discovery_socket_ >= 0) {
      ::close(discovery_socket_);
      discovery_socket_ = -1;
    }
  }

  void run() noexcept
  {
    const auto timeout = std::chrono::milliseconds(timeout_ms_);
    const auto select_interval = std::max(1ms, std::min(50ms, timeout / 2));
    last_valid_packet_ = std::chrono::steady_clock::now();

    const auto context = node_.get_node_base_interface()->get_context();
    while (!stop_.load(std::memory_order_acquire) && rclcpp::ok(context)) {
      fd_set read_descriptors;
      FD_ZERO(&read_descriptors);
      FD_SET(pose_socket_, &read_descriptors);
      FD_SET(discovery_socket_, &read_descriptors);
      const int maximum_descriptor = std::max(pose_socket_, discovery_socket_);
      timeval wait{};
      wait.tv_sec = static_cast<decltype(wait.tv_sec)>(select_interval.count() / 1000);
      wait.tv_usec = static_cast<decltype(wait.tv_usec)>(
        (select_interval.count() % 1000) * 1000);

      const int ready = ::select(
        maximum_descriptor + 1, &read_descriptors, nullptr, nullptr, &wait);
      if (ready < 0) {
        if (errno == EINTR) {
          continue;
        }
        RCLCPP_ERROR(get_logger(), "VR socket select failed: %s", std::strerror(errno));
        return;
      }
      if (ready > 0) {
        if (FD_ISSET(discovery_socket_, &read_descriptors)) {
          drain_discovery_socket();
        }
        if (FD_ISSET(pose_socket_, &read_descriptors)) {
          drain_pose_socket();
        }
      }
      check_timeout(timeout);
    }
  }

  void drain_discovery_socket() noexcept
  {
    std::array<std::uint8_t, 256> buffer{};
    while (true) {
      sockaddr_in sender{};
      socklen_t sender_size = sizeof(sender);
      const ssize_t size = ::recvfrom(
        discovery_socket_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&sender), &sender_size);
      if (size < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(get_logger(), "VR discovery receive failed: %s", std::strerror(errno));
        }
        return;
      }
      const auto request = trim_ascii_whitespace(std::string_view(
          reinterpret_cast<const char *>(buffer.data()), static_cast<std::size_t>(size)));
      if (request != kDiscoveryRequest) {
        continue;
      }
      const ssize_t sent = ::sendto(
        discovery_socket_, discovery_response_.data(), discovery_response_.size(), 0,
        reinterpret_cast<const sockaddr *>(&sender), sender_size);
      if (sent < 0) {
        RCLCPP_WARN(get_logger(), "VR discovery response failed: %s", std::strerror(errno));
      }
    }
  }

  void drain_pose_socket() noexcept
  {
    std::array<std::uint8_t, 65535> buffer{};
    while (true) {
      sockaddr_in sender{};
      socklen_t sender_size = sizeof(sender);
      const ssize_t size = ::recvfrom(
        pose_socket_, buffer.data(), buffer.size(), 0,
        reinterpret_cast<sockaddr *>(&sender), &sender_size);
      if (size < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_WARN(get_logger(), "VR pose receive failed: %s", std::strerror(errno));
        }
        return;
      }
      ++received_packets_;
      const DecodeResult decoded = decode_pose_packet(
        buffer.data(), static_cast<std::size_t>(size));
      if (!decoded) {
        ++invalid_packets_;
        if (invalid_packets_ == 1U || invalid_packets_ % 100U == 0U) {
          RCLCPP_WARN(
            get_logger(), "Rejected invalid VR packet (%llu total): %s",
            static_cast<unsigned long long>(invalid_packets_), decoded.error.c_str());
        }
        continue;
      }
      handle_packet(decoded.packet, sender);
    }
  }

  void begin_session(const sockaddr_in & sender)
  {
    peer_ = sender;
    have_peer_ = true;
    ++session_generation_;
    session_id_ = endpoint_address(sender) + "/" + std::to_string(session_generation_);
    previous_sequence_.reset();
    session_lost_packets_ = 0U;
    timed_out_ = false;
    RCLCPP_INFO(get_logger(), "VR session started: %s", session_id_.c_str());
  }

  void handle_packet(const PosePacket & packet, const sockaddr_in & sender)
  {
    if (!have_peer_ || !same_endpoint(peer_, sender) || timed_out_) {
      begin_session(sender);
    }
    if (!is_sequence_newer(packet.sequence, previous_sequence_)) {
      ++old_packets_;
      return;
    }

    const std::uint32_t lost = packet_loss_count(packet.sequence, previous_sequence_);
    session_lost_packets_ += lost;
    total_lost_packets_ += lost;
    previous_sequence_ = packet.sequence;
    last_valid_packet_ = std::chrono::steady_clock::now();
    timed_out_ = false;

    hc_teleop_interfaces::msg::VrFrame message;
    const builtin_interfaces::msg::Time received_stamp = now();
    message.header.stamp = received_stamp;
    message.header.frame_id = frame_id_;
    message.source_id = session_id_;
    message.sequence = packet.sequence;
    message.source_stamp = seconds_to_time(packet.device_timestamp);
    message.protocol_version = packet.protocol_version;
    copy_pose(packet.head, packet.tracked(kHeadTrackedFlag), message.header, message.head);
    copy_pose(
      packet.left, packet.tracked(kLeftTrackedFlag), message.header,
      message.left_controller);
    copy_pose(
      packet.right, packet.tracked(kRightTrackedFlag), message.header,
      message.right_controller);
    copy_input(packet.left_input, message.left_input);
    copy_input(packet.right_input, message.right_input);
    message.packet_loss_total = total_lost_packets_;
    publisher_->publish(std::move(message));
    ++published_packets_;
  }

  void check_timeout(std::chrono::milliseconds timeout)
  {
    if (!have_peer_ || timed_out_) {
      return;
    }
    const auto elapsed = std::chrono::steady_clock::now() - last_valid_packet_;
    if (elapsed <= timeout) {
      return;
    }
    timed_out_ = true;
    previous_sequence_.reset();
    RCLCPP_WARN(
      get_logger(), "VR session %s timed out after %d ms",
      session_id_.c_str(), timeout_ms_);
  }

  std::string listen_host_;
  int pose_port_{5005};
  int discovery_port_{5006};
  int timeout_ms_{600};
  std::string output_topic_;
  std::string frame_id_;

  rclcpp::Publisher<hc_teleop_interfaces::msg::VrFrame>::SharedPtr publisher_;
  int pose_socket_{-1};
  int discovery_socket_{-1};
  std::string discovery_response_;
  std::atomic_bool stop_{false};
  std::thread worker_;

  bool have_peer_{false};
  bool timed_out_{false};
  sockaddr_in peer_{};
  std::uint64_t session_generation_{0U};
  std::string session_id_;
  std::optional<std::uint32_t> previous_sequence_;
  std::chrono::steady_clock::time_point last_valid_packet_{};
  std::uint64_t session_lost_packets_{0U};

  std::uint64_t received_packets_{0U};
  std::uint64_t published_packets_{0U};
  std::uint64_t total_lost_packets_{0U};
  std::uint64_t old_packets_{0U};
  std::uint64_t invalid_packets_{0U};

  VrGatewayNode & node_;
};

VrGatewayNode::VrGatewayNode(const rclcpp::NodeOptions & options)
: Node("hc_vr_gateway", options), impl_(std::make_unique<Impl>(*this))
{
}

VrGatewayNode::~VrGatewayNode() = default;

}  // namespace hc_vr_gateway

RCLCPP_COMPONENTS_REGISTER_NODE(hc_vr_gateway::VrGatewayNode)
