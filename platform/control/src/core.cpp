#include "hc_control/runtime.hpp"

#include <cmath>
#include <ctime>

namespace hc {
namespace {
constexpr Mask bit(std::size_t i) { return Mask{1} << i; }
Owner owner_for(Mode mode) {
    switch (mode) {
    case Mode::Teleop: return Owner::Vr;
    case Mode::Program: return Owner::Program;
    case Mode::Exo: return Owner::Exo;
    case Mode::Replay: return Owner::Replay;
    case Mode::Home: return Owner::System;
    default: return Owner::None;
    }
}
Owner source_for(Role role) {
    switch (role) {
    case Role::Program: return Owner::Program;
    case Role::Exo: return Owner::Exo;
    case Role::Replay: return Owner::Replay;
    default: return Owner::None;
    }
}
} // namespace

const char* name(Role v) noexcept {
    constexpr const char* names[]{"none","api","backend","vr","program","exo","replay","ik","recorder"};
    return names[static_cast<unsigned>(v)];
}
const char* name(Safety v) noexcept {
    constexpr const char* names[]{"stopped","ready","recovering","fault"};
    return names[static_cast<unsigned>(v)];
}
const char* name(Mode v) noexcept {
    constexpr const char* names[]{"idle","teleop","program","exo","replay","home"};
    return names[static_cast<unsigned>(v)];
}
const char* name(Owner v) noexcept {
    constexpr const char* names[]{"none","vr","program","exo","replay","human","system"};
    return names[static_cast<unsigned>(v)];
}
const char* name(OutputMode v) noexcept {
    constexpr const char* names[]{"stop","hold","position"};
    return names[static_cast<unsigned>(v)];
}
const char* group_name(std::size_t i) noexcept {
    constexpr const char* names[]{"left_arm","right_arm","left_gripper","right_gripper","body"};
    return i < kGroups ? names[i] : "unknown";
}
Nanoseconds monotonic_now() noexcept {
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return static_cast<Nanoseconds>(t.tv_sec) * 1000000000 + t.tv_nsec;
}

Controller::Controller(const Config& config, Shared& shared, std::string_view boot)
    : config_(config), shared_(shared) {
    state_.boot_id.set(boot);
    state_.reason.set("EXPLICIT_RECOVERY_REQUIRED");
    shared_.state.publish(state_);
}

Role Controller::role_for(std::uint64_t id) const noexcept {
    for (const auto& c : clients_) if (c.id == id && id) return c.role;
    return Role::None;
}
bool Controller::permitted(const Event& e) const noexcept {
    const auto role = role_for(e.client);
    if (role == Role::None) return false;
    if (e.op == Op::Status || e.op == Op::Stop || e.op == Op::Disconnect) return true;
    if (e.op == Op::Feedback || e.op == Op::FenceAck || e.op == Op::OutputAck) return e.client == backend_;
    if (e.op == Op::Vr) return e.client == vr_client_;
    if (e.op == Op::IkResult) return e.client == ik_client_;
    if (e.op == Op::Command || e.op == Op::Renew)
        return source_for(role) != Owner::None && source_for(role) == e.source;
    return role == Role::Api;
}
bool Controller::tokens_match(const Event& e) const noexcept {
    return e.boot_id.equals(state_.boot_id.c_str()) && e.epoch == state_.epoch
        && e.lease_id == state_.lease_id && state_.lease_id != 0
        && e.reference_epoch == state_.reference_epoch;
}
Mask Controller::fresh_mask() const noexcept {
    Mask result = 0;
    for (std::size_t i = 0; i < config_.count; ++i)
        if ((state_.feedback_mask & bit(i)) && now_ >= feedback_time_[i]
            && now_ - feedback_time_[i] <= config_.feedback_timeout_ns
            && state_.feedback[i] >= config_.lower[i] && state_.feedback[i] <= config_.upper[i])
            result |= bit(i);
    return result;
}
bool Controller::feedback_fresh(Mask mask) const noexcept {
    return mask != 0 && (fresh_mask() & mask) == mask && state_.backend_ready;
}
Mask Controller::fresh_groups(Mask selection) const noexcept {
    Mask result = 0;
    const Mask fresh = fresh_mask();
    for (const auto group : config_.groups)
        if (group && (group & selection) == group && (group & fresh) == group)
            result |= group;
    return result;
}
bool Controller::require_ready_feedback() const noexcept {
    return backend_ && state_.fence_ack_cap && state_.backend_ready
        && fresh_groups(config_.allowed_mask & config_.arms()) != 0;
}
Mask Controller::vr_allowed_mask() const noexcept {
    Mask mask = 0;
    if (state_.grip[0]) mask |= config_.group(Group::LeftArm) | config_.group(Group::LeftGripper);
    if (state_.grip[1]) mask |= config_.group(Group::RightArm) | config_.group(Group::RightGripper);
    if (state_.mode == Mode::Teleop && state_.grip[0] && vr_.head_tracked)
        mask |= config_.group(Group::Body);
    return mask & state_.leased_mask;
}
void Controller::respond(const Event& e, bool ok, std::string_view code) noexcept {
    if (e.id.empty() && e.op != Op::Hello) return;
    Outbound message;
    message.client = e.client;
    message.id = e.id;
    message.ok = ok;
    if (e.op == Op::Hello && ok) message.assigned_role = e.role;
    message.code.set(code);
    message.state = state_;
    if (!shared_.reliable.push(message)) shared_.overflow.store(true);
}
void Controller::reject(const Event& e, std::string_view code) noexcept {
    ++state_.rejected_commands;
    respond(e, false, code);
}
void Controller::set_owner(Owner owner, std::string_view reason) noexcept {
    if (state_.owner == owner) return;
    Audit audit;
    audit.event_seq = ++audit_seq_;
    audit.tick = state_.ticks;
    audit.epoch = state_.epoch;
    audit.mono_ns = now_;
    audit.boot_id = state_.boot_id;
    audit.reason.set(reason);
    audit.old_owner = state_.owner;
    audit.new_owner = owner;
    state_.owner = owner;
    if (!shared_.audit.push(audit)) ++state_.audit_dropped;
}

void Controller::revoke(std::string_view reason) noexcept {
    ++state_.epoch;
    ++state_.reference_epoch;
    ++state_.transition_count;
    state_.lease_id = 0;
    set_owner(Owner::None, reason);
    state_.requested_owner = Owner::None;
    state_.handback_waiting = false;
    state_.reason.set(reason);
    have_candidate_ = false;
    ever_commanded_ = false;
    candidate_mask_ = 0;
    candidate_deadline_ = 0;
    lease_deadline_ = 0;
    last_ik_sent_ = last_ik_result_ = 0;
    ik_wait_since_ = 0;
    command_base_.fill(0);
    gripper_valid_.fill(false);
    source_seq_.fill(0);
    state_.last_command_seq = 0;
    v_command_.fill(0);
    shared_.current_epoch.store(state_.epoch, std::memory_order_release);
}
void Controller::stop(std::string_view reason) noexcept {
    revoke(reason);
    command_initialized_mask_ = 0;
    state_.safety = Safety::Stopped;
    state_.mode = Mode::Idle;
    state_.paused = false;
    state_.dagger_enabled = false;
    state_.return_paused = false;
    state_.leased_mask = 0;
    state_.transition_pending = false;
    state_.grip.fill(false);
    state_.release_required.fill(true);
    transition_ = Transition::None;
    if (backend_) {
        Outbound message;
        message.kind = Outbound::Kind::Fence;
        message.client = backend_;
        message.seq = ++out_seq_;
        message.state = state_;
        if (!shared_.reliable.push(message)) shared_.overflow.store(true);
    }
}
void Controller::fault(std::string_view reason) noexcept {
    stop(reason);
    state_.safety = Safety::Fault;
}
void Controller::begin(Transition t, Mode mode, Owner owner, std::string_view reason) noexcept {
    revoke(reason);
    transition_ = t;
    target_mode_ = mode;
    target_owner_ = owner;
    state_.requested_owner = owner;
    state_.transition_pending = true;
    transition_deadline_ = now_ + config_.handover_timeout_ns;
    if (t == Transition::Recover || state_.safety != Safety::Ready) {
        command_initialized_mask_ = 0;
        hold_q_ = state_.feedback;
    }
    initialize_command_from_feedback();
    if (t == Transition::Recover) state_.safety = Safety::Recovering;
    Outbound message;
    message.kind = Outbound::Kind::Fence;
    message.client = backend_;
    message.seq = ++out_seq_;
    message.state = state_;
    if (!backend_ || !shared_.reliable.push(message)) fault("BACKEND_UNAVAILABLE");
}
void Controller::initialize_command_from_feedback() noexcept {
    const auto fresh = fresh_mask();
    for (std::size_t i = 0; i < config_.count; ++i)
        if ((fresh & bit(i)) && !(command_initialized_mask_ & bit(i))) {
            q_command_[i] = state_.feedback[i];
            command_initialized_mask_ |= bit(i);
        }
    v_command_.fill(0);
}
void Controller::finish_transition() noexcept {
    const auto t = transition_;
    transition_ = Transition::None;
    state_.transition_pending = false;
    if (!require_ready_feedback()) { fault("FEEDBACK_NOT_READY_AT_FENCE"); return; }
    state_.safety = Safety::Ready;
    state_.mode = target_mode_;
    set_owner(t == Transition::Handback ? Owner::None : target_owner_, state_.reason.c_str());
    state_.requested_owner = Owner::None;
    state_.reason.set("READY");
    if (t == Transition::Recover) command_initialized_mask_ = 0;
    initialize_command_from_feedback();
    if (t == Transition::Recover || t == Transition::End) {
        state_.mode = Mode::Idle;
        set_owner(Owner::None, "IDLE");
        state_.leased_mask = 0;
        state_.paused = false;
        state_.dagger_enabled = false;
        state_.return_paused = false;
        state_.grip.fill(false);
        state_.release_required.fill(true);
        return;
    }
    if (t == Transition::Pause) {
        state_.paused = true;
        set_owner(Owner::None, "PAUSED");
        state_.grip.fill(false);
        state_.release_required.fill(true);
        return;
    }
    state_.paused = false;
    state_.lease_id = ++lease_counter_;
    state_.leased_mask = session_mask_;
    lease_deadline_ = now_ + config_.lease_timeout_ns;
    if ((state_.owner == Owner::Vr || state_.owner == Owner::Human)
        && (state_.grip[0] || state_.grip[1])) ik_wait_since_ = now_;
    if (t == Transition::Handback) {
        set_owner(Owner::None, "AWAITING_PROGRAM");
        state_.requested_owner = Owner::Program;
        state_.handback_waiting = true;
        state_.reason.set("AWAITING_FRESH_PROGRAM_COMMAND");
    }
    if (t == Transition::Home) {
        home_deadline_ = now_ + config_.home_timeout_ns;
        home_stable_since_ = 0; home_stable_samples_ = 0;
        home_feedback_seq_ = state_.feedback_seq;
    }
}

void Controller::accept(const Event& e, Nanoseconds now) noexcept {
    now_ = now;
    state_.now_ns = now;
    if (!e.parse_error.empty()) { reject(e, e.parse_error.c_str()); return; }
    if (e.op == Op::Hello) {
        if (role_for(e.client) != Role::None || e.role == Role::None) {
            reject(e, "INVALID_HELLO"); return;
        }
        bool source_taken = false;
        if (e.role == Role::Program || e.role == Role::Exo || e.role == Role::Replay)
            for (const auto& client : clients_) source_taken = source_taken || (client.id && client.role == e.role);
        if (source_taken || (e.role == Role::Backend && backend_) || (e.role == Role::Vr && vr_client_)
            || (e.role == Role::Ik && ik_client_)) { reject(e, "ROLE_ALREADY_OWNED"); return; }
        if (e.role == Role::Backend && !e.fence_ack_cap) { reject(e, "FENCE_ACK_REQUIRED"); return; }
        Client* slot = nullptr;
        for (auto& c : clients_) if (!c.id) { slot = &c; break; }
        if (!slot) { reject(e, "CLIENT_LIMIT"); return; }
        *slot = Client{e.client, e.role};
        if (e.role == Role::Backend) {
            backend_ = e.client;
            state_.backend_connected = true;
            state_.backend_ready = false;
            state_.physical_stop_ack = e.physical_stop_ack;
            state_.fence_ack_cap = e.fence_ack_cap;
            state_.feedback_mask = 0;
            feedback_velocity_mask_ = 0;
            state_.feedback_seq = 0;
        }
        if (e.role == Role::Vr) {
            vr_client_ = e.client;
            state_.vr_connected = true;
            vr_seq_ = sample_seq_ = 0;
            last_sample_ = 0;
            state_.release_required.fill(true);
            state_.grip.fill(false);
        }
        if (e.role == Role::Ik) { ik_client_ = e.client; state_.ik_connected = true; }
        respond(e, true, "OK");
        return;
    }
    if (!permitted(e)) { reject(e, "FORBIDDEN_ROLE"); return; }
    if ((!e.expected_boot_id.empty() && !e.expected_boot_id.equals(state_.boot_id.c_str()))
        || (e.has_expected_epoch && e.expected_epoch != state_.epoch)) { reject(e,"EXPECTED_STATE_MISMATCH"); return; }
    const bool management = e.op == Op::Recover || e.op == Op::Start || e.op == Op::Pause
        || e.op == Op::Resume || e.op == Op::End || e.op == Op::ResetReference
        || e.op == Op::Home || e.op == Op::SetDagger;
    if (management && e.ingress_epoch && e.ingress_epoch != state_.epoch) {
        reject(e,"QUEUED_MANAGEMENT_STALE"); return;
    }
    if (e.op == Op::Disconnect) {
        if (e.client == backend_) {
            backend_ = 0;
            state_.backend_connected = state_.backend_ready = false;
            fault("BACKEND_DISCONNECTED");
        }
        if (e.client == vr_client_) {
            vr_client_ = 0; state_.vr_connected = false;
            if (state_.owner == Owner::Vr || state_.owner == Owner::Human
                || state_.requested_owner == Owner::Human) {
                if (state_.safety == Safety::Ready) begin(Transition::Pause, state_.mode, Owner::None, "VR_DISCONNECTED");
            }
            state_.grip.fill(false); state_.release_required.fill(true);
        }
        if (e.client == ik_client_) {
            ik_client_ = 0; state_.ik_connected = false;
            if (state_.safety == Safety::Ready && (state_.owner == Owner::Vr || state_.owner == Owner::Human))
                begin(Transition::Pause, state_.mode, Owner::None, "IK_DISCONNECTED");
        }
        for (auto& c : clients_) if (c.id == e.client) c = Client{};
        return;
    }
    switch (e.op) {
    case Op::OutputAck: {
        Audit audit;
        audit.kind = Audit::Kind::OutputAck;
        audit.event_seq = ++audit_seq_;
        audit.tick = state_.ticks;
        audit.epoch = state_.epoch;
        audit.mono_ns = e.sample_ns > 0 ? e.sample_ns : e.received_ns;
        audit.boot_id = state_.boot_id;
        audit.output_seq = e.seq;
        audit.accepted = e.accepted;
        if (!shared_.audit.push(audit)) ++state_.audit_dropped;
        respond(e,true,"OK"); return;
    }
    case Op::Status: respond(e, true, "OK"); return;
    case Op::Stop: stop(e.reason.empty() ? "SOFTWARE_STOP" : e.reason.c_str()); respond(e, true, "OK"); return;
    case Op::Feedback: {
        if (!e.seq || e.seq <= state_.feedback_seq) { reject(e, "FEEDBACK_SEQUENCE"); return; }
        const Nanoseconds sample = e.sample_ns > 0 ? e.sample_ns : e.received_ns;
        if (sample > e.received_ns + 1000000 || sample > now_
            || now_ - sample > config_.feedback_timeout_ns) { reject(e, "FEEDBACK_STALE"); return; }
        state_.feedback_seq = e.seq;
        state_.backend_ready = e.ready;
        bool invalid_active_feedback = false;
        for (std::size_t i = 0; i < config_.count; ++i) if (e.mask & bit(i)) {
            if (e.has_velocities) {
                feedback_velocity_[i] = e.velocities[i];
                feedback_velocity_mask_ |= bit(i);
            } else if ((state_.feedback_mask & bit(i)) && sample > feedback_time_[i]) {
                feedback_velocity_[i] = (e.positions[i]-state_.feedback[i]) /
                    (static_cast<double>(sample-feedback_time_[i])*1e-9);
                feedback_velocity_mask_ |= bit(i);
            } else feedback_velocity_mask_ &= ~bit(i);
            state_.feedback[i] = e.positions[i];
            feedback_time_[i] = sample;
            invalid_active_feedback = invalid_active_feedback || ((state_.leased_mask & bit(i))
                && (e.positions[i] < config_.lower[i] || e.positions[i] > config_.upper[i]));
        }
        state_.feedback_mask |= e.mask;
        if (invalid_active_feedback && state_.safety == Safety::Ready) fault("FEEDBACK_JOINT_LIMIT");
        if (!e.ready && (state_.safety == Safety::Ready || state_.safety == Safety::Recovering)) fault("HARDWARE_NOT_READY");
        respond(e, true, "OK"); return;
    }
    case Op::FenceAck:
        if (!e.boot_id.equals(state_.boot_id.c_str()) || e.epoch != state_.epoch) { reject(e,"FENCE_MISMATCH"); return; }
        if (!e.stopped) { fault("BACKEND_STOP_NOT_CONFIRMED"); reject(e,"STOP_NOT_CONFIRMED"); return; }
        if (state_.transition_pending) finish_transition();
        respond(e, true, "OK"); return;
    case Op::Vr: handle_vr(e); return;
    case Op::Command: accept_command(e, false); return;
    case Op::IkResult: accept_command(e, true); return;
    case Op::Renew:
        if (!tokens_match(e) || e.source != state_.owner || state_.transition_pending) { reject(e,"LEASE_MISMATCH"); return; }
        lease_deadline_ = e.received_ns + config_.lease_timeout_ns;
        respond(e,true,"OK"); return;
    case Op::Recover:
        if (state_.safety != Safety::Stopped && state_.safety != Safety::Fault) { reject(e,"RECOVERY_NOT_ALLOWED"); return; }
        if (!require_ready_feedback()) { reject(e,"FRESH_FEEDBACK_REQUIRED"); return; }
        begin(Transition::Recover, Mode::Idle, Owner::None, "RECOVERING");
        respond(e,true,"ACCEPTED"); return;
    case Op::Start:
        if (state_.safety != Safety::Ready || state_.mode != Mode::Idle || state_.transition_pending) { reject(e,"NOT_READY_IDLE"); return; }
        if (e.mode == Mode::Idle || e.mode == Mode::Home) { reject(e,"INVALID_SESSION_MODE"); return; }
        if (!require_ready_feedback()) { reject(e,"FRESH_FEEDBACK_REQUIRED"); return; }
        session_mask_ = fresh_groups(config_.allowed_mask & (e.has_groups ? e.requested_mask : config_.allowed_mask));
        if (!(session_mask_ & config_.arms())) { reject(e,"NO_AVAILABLE_ARM_GROUP"); return; }
        if (e.has_groups && (session_mask_ & e.requested_mask) != e.requested_mask) { reject(e,"REQUESTED_GROUP_UNAVAILABLE"); return; }
        ++state_.session_id;
        state_.dagger_enabled = e.mode == Mode::Program && e.enabled;
        state_.return_paused = false;
        state_.grip.fill(false); state_.release_required.fill(true);
        begin(Transition::Start, e.mode, owner_for(e.mode), "STARTING_SESSION");
        respond(e,true,"ACCEPTED"); return;
    case Op::Pause:
        if (state_.safety != Safety::Ready || state_.mode == Mode::Idle) { reject(e,"NO_ACTIVE_SESSION"); return; }
        state_.return_paused = true;
        begin(Transition::Pause, state_.mode, Owner::None, "PAUSING");
        respond(e,true,"ACCEPTED"); return;
    case Op::Resume:
        if (state_.safety != Safety::Ready || !state_.paused || state_.transition_pending || state_.mode == Mode::Home) { reject(e,"NOT_PAUSED"); return; }
        if (!feedback_fresh(session_mask_)) { reject(e,"FRESH_FEEDBACK_REQUIRED"); return; }
        state_.return_paused = false;
        state_.grip.fill(false); state_.release_required.fill(true);
        begin(Transition::Resume, state_.mode, owner_for(state_.mode), "RESUMING");
        respond(e,true,"ACCEPTED"); return;
    case Op::End:
        if (state_.safety != Safety::Ready) { reject(e,"NOT_READY"); return; }
        begin(Transition::End, Mode::Idle, Owner::None, "ENDING_SESSION");
        respond(e,true,"ACCEPTED"); return;
    case Op::ResetReference:
        if (state_.safety != Safety::Ready || state_.transition_pending) { reject(e,"NOT_READY"); return; }
        if (state_.mode == Mode::Idle) { ++state_.reference_epoch; respond(e,true,"OK"); return; }
        state_.grip.fill(false); state_.release_required.fill(true);
        begin(Transition::Pause, state_.mode, Owner::None, "REFERENCE_RESET");
        respond(e,true,"ACCEPTED"); return;
    case Op::Home:
        if (state_.safety != Safety::Ready || state_.transition_pending || !require_ready_feedback()) { reject(e,"NOT_READY"); return; }
        session_mask_ = config_.home_mask & config_.allowed_mask & (e.has_groups ? e.requested_mask : config_.allowed_mask);
        if (!session_mask_ || !feedback_fresh(session_mask_)) { reject(e,"HOME_FEEDBACK_OR_SCOPE_UNAVAILABLE"); return; }
        state_.dagger_enabled = false;
        state_.grip.fill(false); state_.release_required.fill(true);
        begin(Transition::Home, Mode::Home, Owner::System, "HOMING");
        respond(e,true,"ACCEPTED"); return;
    case Op::SetDagger:
        if (state_.mode != Mode::Program || state_.safety != Safety::Ready) { reject(e,"NOT_PROGRAM_SESSION"); return; }
        state_.dagger_enabled = e.enabled;
        if (!e.enabled && (state_.owner == Owner::Human || state_.requested_owner == Owner::Human))
            begin(Transition::Pause, Mode::Program, Owner::None, "DAGGER_DISABLED");
        state_.release_required.fill(true);
        respond(e,true,"OK"); return;
    default: reject(e,"UNSUPPORTED_OPERATION"); return;
    }
}

void Controller::handle_vr(const Event& e) noexcept {
    if (!e.seq || e.seq <= vr_seq_) { reject(e,"VR_SEQUENCE"); return; }
    if (e.sample_seq < sample_seq_) { reject(e,"VR_SAMPLE_SEQUENCE"); return; }
    vr_seq_ = e.seq;
    const bool new_sample = e.sample_seq > sample_seq_;
    if (new_sample) {
        sample_seq_ = e.sample_seq;
        const auto sample = e.sample_ns > 0 ? e.sample_ns : e.received_ns;
        if (sample > now_ || sample > e.received_ns + 1000000) { reject(e,"VR_SAMPLE_TIME"); return; }
        last_sample_ = sample;
    }
    vr_ = e;
    const bool vr_authority = state_.safety == Safety::Ready &&
        (state_.mode == Mode::Teleop || (state_.mode == Mode::Program && state_.dagger_enabled));
    if (!vr_authority) {
        // An unselected VR source is observation only. In particular Grip with
        // DAgger disabled must not reset a program's reference or candidate.
        state_.grip.fill(false);
        for (std::size_t hand=0;hand<2;++hand)
            state_.release_required[hand]=e.grip[hand] || !e.tracked[hand];
        respond(e,true,"OBSERVATION_ONLY");
        return;
    }
    const auto previous = state_.grip;
    bool tracking_lost = false, changed = false, pressed = false;
    for (std::size_t hand = 0; hand < 2; ++hand) {
        const auto arm = hand == 0 ? config_.group(Group::LeftArm) : config_.group(Group::RightArm);
        if (!e.tracked[hand]) {
            if (previous[hand]) tracking_lost = true;
            state_.grip[hand] = false;
            state_.release_required[hand] = true;
        } else if (!e.grip[hand]) {
            state_.grip[hand] = false;
            state_.release_required[hand] = false;
        } else if (!state_.release_required[hand] && new_sample
            && now_ - last_sample_ <= config_.command_timeout_ns
            && feedback_fresh(arm) && (session_mask_ & arm) == arm) {
            state_.grip[hand] = true;
        }
        if (state_.grip[hand] && !previous[hand]) {
            pressed = true;
            JointArray reference_q{};
            for (std::size_t i = 0; i < config_.count; ++i)
                reference_q[i] = (command_initialized_mask_ & bit(i)) ? q_command_[i] : state_.feedback[i];
            if (hand == 0) { left_reference_ = e.left; left_reference_q_ = reference_q; head_reference_ = e.head; }
            else { right_reference_ = e.right; right_reference_q_ = reference_q; }
        }
        changed = changed || state_.grip[hand] != previous[hand];
    }
    if (changed) {
        ++state_.reference_epoch;
        have_candidate_ = false;
        candidate_mask_ = 0;
        last_ik_sent_ = last_ik_result_ = 0;
        ik_wait_since_ = state_.grip[0] || state_.grip[1] ? now_ : 0;
        v_command_.fill(0);
    }
    if (tracking_lost && state_.safety == Safety::Ready
        && (state_.owner == Owner::Vr || state_.owner == Owner::Human || state_.requested_owner == Owner::Human)) {
        state_.return_paused = true;
        begin(Transition::Pause, state_.mode, Owner::None, "TRACKING_LOST");
        respond(e,true,"PAUSING"); return;
    }
    if (state_.safety == Safety::Ready && state_.mode == Mode::Program && state_.dagger_enabled) {
        if (pressed && !state_.transition_pending && state_.owner != Owner::Human) {
            state_.return_paused = state_.paused;
            begin(Transition::Takeover, Mode::Program, Owner::Human, "DAGGER_TAKEOVER");
        } else if (!state_.grip[0] && !state_.grip[1] && (previous[0] || previous[1])
            && (state_.owner == Owner::Human || state_.requested_owner == Owner::Human)) {
            if (state_.return_paused) begin(Transition::Pause, Mode::Program, Owner::None, "DAGGER_RETURN_PAUSED");
            else begin(Transition::Handback, Mode::Program, Owner::Program, "DAGGER_HANDBACK");
        }
    }
    if (new_sample && (state_.owner == Owner::Vr || state_.owner == Owner::Human))
        lease_deadline_ = last_sample_ + config_.lease_timeout_ns;
    if (config_.enable_grippers && (state_.owner == Owner::Vr || state_.owner == Owner::Human)) {
        for (std::size_t hand = 0; hand < 2; ++hand) if (state_.grip[hand]) {
            grippers_[hand] = e.trigger[hand]; gripper_valid_[hand] = true;
        }
    }
    respond(e,true,"OK");
}

void Controller::accept_command(const Event& e, bool ik) noexcept {
    if (state_.safety != Safety::Ready || state_.transition_pending) { reject(e,"NOT_ACTIVE"); return; }
    if (!tokens_match(e)) { reject(e,"FENCE_MISMATCH"); return; }
    const bool handback = state_.handback_waiting && e.source == Owner::Program;
    if ((!ik && e.source != state_.owner && !handback)
        || (ik && state_.owner != Owner::Vr && state_.owner != Owner::Human)) { reject(e,"NOT_OWNER"); return; }
    const auto sample = e.sample_ns > 0 ? e.sample_ns : e.received_ns;
    if (sample > e.received_ns || sample > now_) { reject(e,"TARGET_SAMPLE_FUTURE"); return; }
    Nanoseconds deadline = std::min(e.received_ns,sample) + std::min(e.ttl_ns, config_.command_timeout_ns);
    if (e.deadline_ns > 0) deadline = std::min(deadline,e.deadline_ns);
    Mask allowed = state_.leased_mask;
    if (ik) {
        const auto& ticket = tickets_[e.seq % tickets_.size()];
        if (!e.seq || e.seq <= last_ik_result_ || ticket.seq != e.seq
            || ticket.epoch != state_.epoch || ticket.reference != state_.reference_epoch) { reject(e,"IK_REQUEST_MISMATCH"); return; }
        deadline = std::min(deadline, ticket.deadline);
        allowed &= ticket.allowed & vr_allowed_mask();
    } else if (!e.seq || e.seq <= source_seq_[static_cast<unsigned>(e.source)]) { reject(e,"COMMAND_SEQUENCE"); return; }
    if (deadline <= now_ || e.ttl_ns <= 0) { reject(e,"TARGET_EXPIRED"); return; }
    if (!e.mask || (e.mask & allowed) != e.mask || !feedback_fresh(e.mask)) { reject(e,"GROUP_NOT_AUTHORIZED"); return; }
    for (std::size_t i = 0; i < config_.count; ++i) if (e.mask & bit(i)) {
        if (!std::isfinite(e.positions[i]) || e.positions[i] < config_.lower[i]
            || e.positions[i] > config_.upper[i]) { reject(e,"JOINT_LIMIT"); return; }
    }
    if (e.has_base && (!config_.enable_base || state_.owner == Owner::Human)) { reject(e,"BASE_NOT_AUTHORIZED"); return; }
    if (e.has_grippers && !config_.enable_grippers) { reject(e,"GRIPPER_NOT_AUTHORIZED"); return; }
    candidate_ = e.positions;
    candidate_mask_ = e.mask;
    candidate_deadline_ = deadline;
    have_candidate_ = ever_commanded_ = true;
    lease_deadline_ = e.received_ns + config_.lease_timeout_ns;
    if (ik) last_ik_result_ = e.seq;
    else source_seq_[static_cast<unsigned>(e.source)] = e.seq;
    if (!ik) gripper_valid_.fill(false);
    if (e.has_grippers) {
        for (std::size_t hand = 0; hand < 2; ++hand) if (e.grippers_valid[hand]) {
            const auto arm = hand == 0 ? config_.group(Group::LeftArm) : config_.group(Group::RightArm);
            if (arm && (state_.leased_mask & arm) == arm) {
                grippers_[hand] = e.grippers[hand]; gripper_valid_[hand] = true;
            }
        }
    }
    if (!ik) command_base_ = e.has_base ? e.base : std::array<double,3>{};
    if (handback) {
        set_owner(Owner::Program, "PROGRAM_RESUMED_WITH_FRESH_COMMAND");
        state_.requested_owner = Owner::None;
        state_.handback_waiting = false;
        state_.reason.set("PROGRAM_RESUMED_WITH_FRESH_COMMAND");
    }
    ++state_.accepted_commands;
    state_.last_command_seq = e.seq;
    respond(e,true,"OK");
}

void Controller::emit_ik() noexcept {
    if (!ik_client_ || !vr_.seq || last_ik_sent_ == vr_.seq || !vr_allowed_mask()
        || !last_sample_ || now_ >= last_sample_ + config_.command_timeout_ns) return;
    IkRequest request;
    request.boot_id = state_.boot_id;
    request.epoch = state_.epoch;
    request.lease_id = state_.lease_id;
    request.reference_epoch = state_.reference_epoch;
    request.seq = vr_.seq;
    request.issued_ns = now_;
    request.deadline_ns = last_sample_ + config_.command_timeout_ns;
    request.mask = fresh_mask();
    request.positions = state_.feedback;
    request.reference_left = left_reference_q_;
    request.reference_right = right_reference_q_;
    request.left = vr_.left; request.right = vr_.right; request.head = vr_.head;
    request.left_reference = left_reference_; request.right_reference = right_reference_;
    request.head_reference = head_reference_;
    request.active = state_.grip;
    request.allow_body = state_.mode == Mode::Teleop && state_.grip[0] && vr_.head_tracked
        && (vr_allowed_mask() & config_.group(Group::Body)) != 0;
    request.mode = state_.mode;
    request.source = state_.owner;
    tickets_[request.seq % tickets_.size()] = Ticket{request.seq,state_.epoch,state_.reference_epoch,request.deadline_ns,vr_allowed_mask()};
    last_ik_sent_ = vr_.seq;
    shared_.ik.publish(request);
}

void Controller::publish_output(double dt) noexcept {
    Output output;
    output.boot_id = state_.boot_id;
    output.epoch = state_.epoch;
    output.lease_id = state_.lease_id;
    output.reference_epoch = state_.reference_epoch;
    output.seq = ++out_seq_;
    output.tick = state_.ticks;
    output.issued_ns = now_;
    output.deadline_ns = now_ + config_.command_timeout_ns;
    output.mask = fresh_mask() & config_.allowed_mask;
    output.mode = state_.safety == Safety::Ready && !state_.transition_pending ? OutputMode::Hold : OutputMode::Stop;
    const bool active = output.mode != OutputMode::Stop && state_.owner != Owner::None;
    const bool home = active && state_.owner == Owner::System && state_.mode == Mode::Home;
    const bool candidate_valid = active && have_candidate_ && candidate_deadline_ > now_;
    if (candidate_valid) output.deadline_ns = std::min(output.deadline_ns, candidate_deadline_);
    for (std::size_t i = 0; i < config_.count; ++i) {
        if (!(output.mask & bit(i))) continue;
        const bool moving = home ? (session_mask_ & bit(i)) != 0 : candidate_valid && (candidate_mask_ & bit(i));
        if (moving) {
            const double target = home ? config_.home[i] : candidate_[i];
            const double delta = target - q_command_[i];
            const double acceleration = home ? std::min(config_.acceleration[i],config_.home_acceleration) : config_.acceleration[i];
            const double velocity = home ? std::min(config_.velocity[i],config_.home_velocity) : config_.velocity[i];
            const double braking = std::max(0.0, std::sqrt(acceleration*acceleration*dt*dt + 2*acceleration*std::abs(delta)) - acceleration*dt);
            const double desired_v = std::copysign(std::min(velocity, braking), delta);
            double v = std::clamp(desired_v, v_command_[i]-acceleration*dt, v_command_[i]+acceleration*dt);
            // Position bounds are checked again after bounded acceleration.
            const double next = q_command_[i] + v*dt;
            if (!std::isfinite(next) || next < config_.lower[i] || next > config_.upper[i]) {
                fault("MOTION_LIMIT_INFEASIBLE");
                output.mode = OutputMode::Stop;
                output.epoch = state_.epoch;
                output.lease_id = 0;
                output.reference_epoch = state_.reference_epoch;
                v_command_.fill(0);
                break;
            }
            q_command_[i] = next;
            v_command_[i] = v;
            output.mode = OutputMode::Position;
        } else {
            v_command_[i] = 0;
            if (state_.safety != Safety::Ready || !(command_initialized_mask_ & bit(i))) {
                q_command_[i] = state_.feedback[i];
                if (state_.safety == Safety::Ready) command_initialized_mask_ |= bit(i);
            }
        }
    }
    output.positions = q_command_;
    output.velocities = v_command_;
    if (active && config_.enable_grippers) {
        output.grippers = grippers_;
        output.gripper_valid = gripper_valid_;
    }
    if (active && config_.enable_base) {
        if (state_.mode == Mode::Teleop && state_.owner == Owner::Vr
            && (state_.grip[0] || (config_.allow_independent_base && vr_.base_independent))
            && last_sample_ && now_ < last_sample_ + config_.command_timeout_ns) output.base = vr_.base;
        else if (state_.owner != Owner::Human && candidate_valid) output.base = command_base_;
    }
    if (output.mode == OutputMode::Stop) {
        output.velocities.fill(0);
        output.base.fill(0);
        output.gripper_valid.fill(false);
    }
    state_.commanded = q_command_;
    shared_.output.publish(output);
    if (!shared_.recording_outputs.push(output)) ++state_.outputs_dropped;
}

void Controller::tick(Nanoseconds now, Nanoseconds previous_cycle_ns) noexcept {
    now_ = now;
    state_.now_ns = now;
    ++state_.ticks;
    state_.cycle_max_ns = std::max(state_.cycle_max_ns, previous_cycle_ns);
    const auto period = static_cast<Nanoseconds>(1000000000.0 / config_.control_hz);
    if (previous_cycle_ns > period) ++state_.overruns;
    if (shared_.overflow.exchange(false)) stop("EVENT_QUEUE_OVERFLOW");
    if (shared_.stop_requested.exchange(false)) stop("SOFTWARE_STOP");
    state_.available_mask = fresh_groups(config_.allowed_mask);
    state_.fresh_feedback_mask = fresh_mask();
    Nanoseconds newest = 0;
    for (std::size_t i = 0; i < config_.count; ++i) newest = std::max(newest, feedback_time_[i]);
    state_.feedback_age_ns = newest ? std::max<Nanoseconds>(0, now_-newest) : -1;
    if (state_.transition_pending && now_ > transition_deadline_) fault("FENCE_TIMEOUT");
    if ((state_.safety == Safety::Ready || state_.safety == Safety::Recovering)
        && !require_ready_feedback()) fault("FEEDBACK_TIMEOUT");
    if (state_.safety == Safety::Ready && !state_.transition_pending) {
        if (state_.owner != Owner::None && state_.leased_mask && !feedback_fresh(state_.leased_mask)) fault("LEASED_FEEDBACK_TIMEOUT");
        else if (state_.handback_waiting && now_ > lease_deadline_)
            begin(Transition::Pause, Mode::Program, Owner::None, "PROGRAM_HANDBACK_TIMEOUT");
        else if (state_.owner == Owner::Vr || state_.owner == Owner::Human) {
            const bool arm_active = state_.grip[0] || state_.grip[1];
            const bool base_active = state_.mode == Mode::Teleop && config_.enable_base
                && config_.allow_independent_base && vr_.base_independent
                && (std::abs(vr_.base[0])+std::abs(vr_.base[1])+std::abs(vr_.base[2]) > 1e-9);
            if ((arm_active || base_active) && (!last_sample_ || now_ >= last_sample_ + config_.command_timeout_ns)) {
                state_.return_paused = true;
                begin(Transition::Pause, state_.mode, Owner::None, "VR_INPUT_TIMEOUT");
            } else if (arm_active && !last_ik_result_ && ik_wait_since_
                && now_ - ik_wait_since_ > config_.command_timeout_ns) {
                state_.return_paused = true;
                begin(Transition::Pause, state_.mode, Owner::None, "IK_UNAVAILABLE");
            } else if (have_candidate_ && now_ >= candidate_deadline_ && (state_.grip[0] || state_.grip[1])) {
                state_.return_paused = true;
                begin(Transition::Pause, state_.mode, Owner::None, "IK_TARGET_TIMEOUT");
            } else emit_ik();
        } else if (state_.owner == Owner::System && state_.mode == Mode::Home) {
            if (now_ >= home_deadline_) fault("HOME_TIMEOUT");
            else {
                bool reached = true;
                for (std::size_t i = 0; i < config_.count; ++i) if (session_mask_ & bit(i))
                    reached = reached && std::abs(state_.feedback[i]-config_.home[i]) <= config_.home_tolerance
                        && std::abs(v_command_[i]) < 0.02 && (feedback_velocity_mask_ & bit(i))
                        && std::abs(feedback_velocity_[i]) < 0.03;
                if (!reached) { home_stable_since_ = 0; home_stable_samples_ = 0; }
                else if (state_.feedback_seq != home_feedback_seq_) {
                    if (!home_stable_since_) home_stable_since_ = now_;
                    ++home_stable_samples_;
                }
                home_feedback_seq_ = state_.feedback_seq;
                if (reached && home_stable_samples_ >= 2 && now_ - home_stable_since_ >= 100000000)
                    begin(Transition::End, Mode::Idle, Owner::None, "HOME_COMPLETE");
            }
        } else if (state_.owner != Owner::None && (now_ >= lease_deadline_ || (ever_commanded_ && now_ >= candidate_deadline_)))
            begin(Transition::Pause, state_.mode, Owner::None, "SOURCE_TIMEOUT");
    }
    // A stalled tick must not make one large movement when it resumes.
    const double dt = std::min(1.0/config_.control_hz, last_tick_ ? std::max(0.000001,static_cast<double>(now_-last_tick_)*1e-9) : 1.0/config_.control_hz);
    last_tick_ = now_;
    publish_output(dt);
    shared_.state.publish(state_);
}
} // namespace hc
