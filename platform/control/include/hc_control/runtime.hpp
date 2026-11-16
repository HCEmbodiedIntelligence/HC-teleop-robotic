#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>

namespace hc {
constexpr std::size_t kJoints = 64;
constexpr std::size_t kClients = 32;
using Mask = std::uint64_t;
using Nanoseconds = std::int64_t;
using JointArray = std::array<double, kJoints>;
using Pose = std::array<double, 7>;

template<std::size_t N> struct Text {
    std::array<char, N> bytes{};
    Text() = default;
    explicit Text(std::string_view s) { set(s); }
    void set(std::string_view s) noexcept {
        const auto n = std::min(s.size(), N - 1);
        std::memcpy(bytes.data(), s.data(), n);
        bytes[n] = '\0';
    }
    const char* c_str() const noexcept { return bytes.data(); }
    bool empty() const noexcept { return bytes[0] == '\0'; }
    bool equals(std::string_view s) const noexcept { return s == bytes.data(); }
};

// Exactly one producer and one consumer. All storage is allocated at startup.
template<class T, std::size_t N> class Spsc {
    static_assert(N > 1);
    std::array<T, N> data_{};
    alignas(64) std::atomic<std::size_t> head_{0};
    alignas(64) std::atomic<std::size_t> tail_{0};
public:
    bool push(const T& value) noexcept {
        const auto h = head_.load(std::memory_order_relaxed);
        const auto next = (h + 1) % N;
        if (next == tail_.load(std::memory_order_acquire)) return false;
        data_[h] = value;
        head_.store(next, std::memory_order_release);
        return true;
    }
    bool pop(T& value) noexcept {
        const auto t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        value = data_[t];
        tail_.store((t + 1) % N, std::memory_order_release);
        return true;
    }
};

// SPSC triple buffer: writer, middle, reader always own distinct slots.
// Unlike an atomic pointer to a two-slot buffer, slots cannot be recycled under
// an active read. Producer never waits and always replaces the pending value.
template<class T> class Latest {
    std::array<T, 3> data_{};
    alignas(64) std::atomic<unsigned> middle_{1};
    unsigned back_{2};
    unsigned front_{0};
public:
    void publish(const T& value) noexcept {
        data_[back_] = value;
        back_ = middle_.exchange(back_ | 4u, std::memory_order_acq_rel) & 3u;
    }
    bool consume(T& value) noexcept {
        if (!(middle_.load(std::memory_order_acquire) & 4u)) return false;
        front_ = middle_.exchange(front_, std::memory_order_acq_rel) & 3u;
        value = data_[front_];
        return true;
    }
};

enum class Group : unsigned { LeftArm, RightArm, LeftGripper, RightGripper, Body, Count };
constexpr std::size_t kGroups = static_cast<std::size_t>(Group::Count);
enum class Role { None, Api, Backend, Vr, Program, Exo, Replay, Ik, Recorder };
enum class Safety { Stopped, Ready, Recovering, Fault };
enum class Mode { Idle, Teleop, Program, Exo, Replay, Home };
enum class Owner { None, Vr, Program, Exo, Replay, Human, System };
enum class Op {
    Hello, Disconnect, Status, Feedback, FenceAck, Vr, Command, IkResult,
    Renew, Recover, Start, Pause, Resume, End, Stop, ResetReference, Home, SetDagger, OutputAck
};
enum class Transition { None, Recover, Start, Pause, Resume, End, Takeover, Handback, Home, Reset };
enum class OutputMode { Stop, Hold, Position };

struct Config {
    std::string robot_id{"unknown"};
    std::string config_hash{"unversioned"};
    std::size_t count{0};
    std::array<std::string, kJoints> names{};
    JointArray lower{}, upper{}, velocity{}, acceleration{}, home{};
    std::array<Mask, kGroups> groups{};
    Mask allowed_mask{0};
    double control_hz{200};
    Nanoseconds feedback_timeout_ns{200000000};
    Nanoseconds command_timeout_ns{150000000};
    Nanoseconds lease_timeout_ns{1000000000};
    Nanoseconds handover_timeout_ns{1000000000};
    Nanoseconds home_timeout_ns{30000000000};
    double home_tolerance{0.02};
    double home_velocity{0.7}, home_acceleration{1.4};
    Mask home_mask{~Mask{0}};
    bool enable_grippers{false};
    bool enable_base{false};
    bool allow_independent_base{false};
    std::array<double, 3> base_limit{0.2, 0.2, 0.3};
    Mask group(Group g) const noexcept { return groups[static_cast<unsigned>(g)]; }
    Mask arms() const noexcept { return group(Group::LeftArm) | group(Group::RightArm); }
};

struct Event {
    Op op{Op::Status};
    std::uint64_t client{0};
    Role role{Role::None};
    Text<64> id{}, boot_id{};
    Text<64> expected_boot_id{};
    Text<64> reason{};
    Text<64> parse_error{};
    Nanoseconds received_ns{0}, sample_ns{0};
    Nanoseconds deadline_ns{0};
    std::uint64_t epoch{0}, lease_id{0}, reference_epoch{0}, seq{0}, sample_seq{0};
    std::uint64_t expected_epoch{0};
    std::uint64_t ingress_epoch{0};
    Mask mask{0}, requested_mask{0};
    JointArray positions{}, velocities{};
    bool has_velocities{false};
    bool ready{false}, stopped{false}, enabled{false};
    bool fence_ack_cap{false}, physical_stop_ack{false};
    bool has_grippers{false}, has_base{false}, has_head{false};
    bool has_groups{false};
    bool has_expected_epoch{false}, base_independent{false}, accepted{false};
    Mode mode{Mode::Idle};
    Owner source{Owner::None};
    Nanoseconds ttl_ns{0};
    std::array<bool, 2> grip{}, tracked{};
    bool head_tracked{false};
    std::array<double, 2> trigger{}, grippers{};
    std::array<bool, 2> grippers_valid{true,true};
    std::array<double, 3> base{};
    Pose left{0,0,0,0,0,0,1}, right{0,0,0,0,0,0,1}, head{0,0,0,0,0,0,1};
};

struct State {
    Text<64> boot_id{}, reason{};
    Safety safety{Safety::Stopped};
    Mode mode{Mode::Idle};
    Owner owner{Owner::None}, requested_owner{Owner::None};
    std::uint64_t epoch{1}, lease_id{0}, reference_epoch{1}, session_id{0};
    std::uint64_t feedback_seq{0}, accepted_commands{0}, rejected_commands{0};
    std::uint64_t last_command_seq{0};
    std::uint64_t ticks{0}, overruns{0}, transition_count{0};
    std::uint64_t audit_dropped{0}, outputs_dropped{0};
    bool paused{false}, dagger_enabled{false}, return_paused{false};
    bool transition_pending{false}, handback_waiting{false};
    bool backend_connected{false}, backend_ready{false}, physical_stop_ack{false};
    bool fence_ack_cap{false}, ik_connected{false}, vr_connected{false};
    std::array<bool, 2> grip{}, release_required{true,true};
    Mask feedback_mask{0}, fresh_feedback_mask{0}, available_mask{0}, leased_mask{0};
    JointArray feedback{}, commanded{};
    Nanoseconds now_ns{0}, feedback_age_ns{0}, cycle_max_ns{0};
};

struct Output {
    Text<64> boot_id{};
    std::uint64_t epoch{0}, lease_id{0}, reference_epoch{0}, seq{0}, tick{0};
    Nanoseconds issued_ns{0}, deadline_ns{0};
    OutputMode mode{OutputMode::Stop};
    Mask mask{0};
    JointArray positions{}, velocities{};
    std::array<double, 2> grippers{};
    std::array<bool, 2> gripper_valid{};
    std::array<double, 3> base{};
};

struct IkRequest {
    Text<64> boot_id{};
    std::uint64_t epoch{0}, lease_id{0}, reference_epoch{0}, seq{0};
    Nanoseconds issued_ns{0}, deadline_ns{0};
    Mask mask{0};
    JointArray positions{}, reference_left{}, reference_right{};
    Pose left{}, right{}, left_reference{}, right_reference{}, head{}, head_reference{};
    std::array<bool, 2> active{};
    bool allow_body{false};
    Mode mode{Mode::Idle};
    Owner source{Owner::None};
};

struct Outbound {
    enum class Kind { Response, Fence, Disconnect } kind{Kind::Response};
    std::uint64_t client{0}, seq{0};
    Text<64> id{}, code{};
    bool ok{false};
    Role assigned_role{Role::None};
    State state{};
};

struct Audit {
    enum class Kind { Ownership, OutputAck } kind{Kind::Ownership};
    std::uint64_t event_seq{0}, tick{0}, epoch{0}, output_seq{0};
    Nanoseconds mono_ns{0};
    Text<64> boot_id{}, reason{};
    Owner old_owner{Owner::None}, new_owner{Owner::None};
    bool accepted{false};
};

struct Shared {
    Shared();
    Spsc<Event, 513> events;
    Spsc<Outbound, 257> reliable;
    Spsc<Audit, 1025> audit;
    Spsc<Output, 1025> recording_outputs;
    Latest<State> state;
    Latest<Output> output;
    Latest<IkRequest> ik;
    std::atomic<bool> stop_requested{false}, overflow{false}, running{true};
    std::atomic<std::uint64_t> current_epoch{1};
};

class Controller {
public:
    Controller(const Config& config, Shared& shared, std::string_view boot_id);
    void accept(const Event& event, Nanoseconds now) noexcept;
    void tick(Nanoseconds now, Nanoseconds previous_cycle_ns = 0) noexcept;
    const State& state() const noexcept { return state_; }
private:
    struct Client { std::uint64_t id{0}; Role role{Role::None}; };
    struct Ticket {
        std::uint64_t seq{0}, epoch{0}, reference{0};
        Nanoseconds deadline{0};
        Mask allowed{0};
    };
    const Config& config_;
    Shared& shared_;
    State state_{};
    std::array<Client, kClients> clients_{};
    std::array<Nanoseconds, kJoints> feedback_time_{};
    std::array<std::uint64_t, 8> source_seq_{};
    std::array<Ticket, 256> tickets_{};
    std::uint64_t backend_{0}, vr_client_{0}, ik_client_{0};
    std::uint64_t out_seq_{0}, lease_counter_{0}, vr_seq_{0}, sample_seq_{0};
    std::uint64_t last_ik_sent_{0}, last_ik_result_{0};
    std::uint64_t audit_seq_{0};
    Nanoseconds now_{0}, last_tick_{0}, last_sample_{0}, lease_deadline_{0};
    Nanoseconds candidate_deadline_{0}, transition_deadline_{0}, home_deadline_{0};
    Nanoseconds ik_wait_since_{0};
    Nanoseconds home_stable_since_{0};
    std::uint64_t home_feedback_seq_{0};
    unsigned home_stable_samples_{0};
    Transition transition_{Transition::None};
    Mode target_mode_{Mode::Idle};
    Owner target_owner_{Owner::None};
    Mask session_mask_{0}, candidate_mask_{0};
    JointArray candidate_{}, q_command_{}, v_command_{}, left_reference_q_{}, right_reference_q_{};
    JointArray hold_q_{};
    JointArray feedback_velocity_{};
    Mask feedback_velocity_mask_{0};
    Event vr_{};
    Pose left_reference_{0,0,0,0,0,0,1}, right_reference_{0,0,0,0,0,0,1}, head_reference_{0,0,0,0,0,0,1};
    std::array<double,2> grippers_{};
    std::array<bool,2> gripper_valid_{};
    std::array<double,3> command_base_{};
    bool have_candidate_{false}, ever_commanded_{false};
    Role role_for(std::uint64_t client) const noexcept;
    bool permitted(const Event& e) const noexcept;
    bool tokens_match(const Event& e) const noexcept;
    bool feedback_fresh(Mask mask) const noexcept;
    Mask fresh_mask() const noexcept;
    Mask fresh_groups(Mask selection) const noexcept;
    Mask vr_allowed_mask() const noexcept;
    bool require_ready_feedback() const noexcept;
    void respond(const Event& e, bool ok, std::string_view code) noexcept;
    void reject(const Event& e, std::string_view code) noexcept;
    void revoke(std::string_view reason) noexcept;
    void fault(std::string_view reason) noexcept;
    void stop(std::string_view reason) noexcept;
    void begin(Transition transition, Mode mode, Owner owner, std::string_view reason) noexcept;
    void finish_transition() noexcept;
    void handle_vr(const Event& e) noexcept;
    void accept_command(const Event& e, bool ik) noexcept;
    void publish_output(double dt) noexcept;
    void emit_ik() noexcept;
    void initialize_command_from_feedback() noexcept;
    void set_owner(Owner owner, std::string_view reason) noexcept;
};

const char* name(Role value) noexcept;
const char* name(Safety value) noexcept;
const char* name(Mode value) noexcept;
const char* name(Owner value) noexcept;
const char* name(OutputMode value) noexcept;
const char* group_name(std::size_t index) noexcept;
Nanoseconds monotonic_now() noexcept;
} // namespace hc
