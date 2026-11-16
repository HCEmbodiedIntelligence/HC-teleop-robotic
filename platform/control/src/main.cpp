#include "hc_control/runtime.hpp"
#include "json.hpp"

#include <cerrno>
#include <cmath>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <poll.h>
#include <random>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

namespace {
using Json = nlohmann::json;
using namespace hc;
constexpr std::size_t kPacket = 65536;
volatile std::sig_atomic_t stop_signal = 0;
void signal_handler(int) { stop_signal = 1; }
std::uint64_t uint_value(const Json& j, const char* key, std::uint64_t fallback = 0) {
    if (!j.contains(key)) return fallback;
    const auto& v = j.at(key);
    if (!v.is_number_integer() || (v.is_number_integer() && !v.is_number_unsigned() && v.get<std::int64_t>() < 0))
        throw std::runtime_error(std::string("INVALID_INTEGER:") + key);
    return v.get<std::uint64_t>();
}
double number(const Json& value) {
    if (!value.is_number()) throw std::runtime_error("INVALID_NUMBER");
    double x = value.get<double>();
    if (!std::isfinite(x)) throw std::runtime_error("NONFINITE_NUMBER");
    return x;
}
std::string string_value(const Json& j, const char* key, const std::string& fallback = "") {
    if (!j.contains(key)) return fallback;
    if (!j.at(key).is_string()) throw std::runtime_error(std::string("INVALID_STRING:") + key);
    auto value = j.at(key).get<std::string>();
    if (value.size() > 255 || value.find('\0') != std::string::npos) throw std::runtime_error("STRING_TOO_LONG");
    return value;
}
bool boolean(const Json& j, const char* key, bool fallback = false) {
    if (!j.contains(key)) return fallback;
    if (!j.at(key).is_boolean()) throw std::runtime_error(std::string("INVALID_BOOL:") + key);
    return j.at(key).get<bool>();
}
Nanoseconds duration(const Json& j, const char* key, double fallback, double maximum = 60000) {
    const double x = j.contains(key) ? number(j.at(key)) : fallback;
    if (x <= 0 || x > maximum) throw std::runtime_error(std::string("INVALID_DURATION:")+key);
    return static_cast<Nanoseconds>(x * 1000000.0);
}
Role parse_role(const std::string& s) {
    for (unsigned i = 1; i <= static_cast<unsigned>(Role::Recorder); ++i)
        if (s == name(static_cast<Role>(i))) return static_cast<Role>(i);
    if (s == "observer") return Role::Recorder;
    throw std::runtime_error("INVALID_ROLE");
}
Mode parse_mode(const std::string& s) {
    for (unsigned i = 0; i <= static_cast<unsigned>(Mode::Home); ++i)
        if (s == name(static_cast<Mode>(i))) return static_cast<Mode>(i);
    throw std::runtime_error("INVALID_MODE");
}
Owner parse_owner(const std::string& s) {
    for (unsigned i = 0; i <= static_cast<unsigned>(Owner::System); ++i)
        if (s == name(static_cast<Owner>(i))) return static_cast<Owner>(i);
    throw std::runtime_error("INVALID_SOURCE");
}
std::size_t group_index(const std::string& s) {
    for (std::size_t i = 0; i < kGroups; ++i) if (s == group_name(i)) return i;
    throw std::runtime_error("UNKNOWN_GROUP:"+s);
}
std::size_t joint_index(const Config& c, const std::string& s) {
    for (std::size_t i=0; i<c.count; ++i) if (c.names[i] == s) return i;
    throw std::runtime_error("UNKNOWN_JOINT:"+s);
}
Mask group_selection(const Config& c, const Json& list) {
    if (!list.is_array() || list.empty()) throw std::runtime_error("INVALID_GROUPS");
    Mask mask = 0;
    for (const auto& item : list) {
        if (!item.is_string()) throw std::runtime_error("INVALID_GROUP");
        mask |= c.groups[group_index(item.get<std::string>())];
    }
    return mask;
}
Config read_config(const std::string& path) {
    std::ifstream stream(path);
    if (!stream) throw std::runtime_error("Cannot open config: "+path);
    const auto j = Json::parse(stream);
    Config c;
    c.robot_id = string_value(j,"robot_id","unknown");
    c.config_hash = string_value(j,"config_hash","unversioned");
    const auto& names = j.at("joint_names");
    if (!names.is_array() || names.empty() || names.size()>kJoints) throw std::runtime_error("joint_names must contain 1..64 names");
    c.count = names.size();
    std::unordered_map<std::string,bool> seen;
    for (std::size_t i=0; i<c.count; ++i) {
        if (!names[i].is_string()) throw std::runtime_error("Invalid joint name");
        c.names[i] = names[i].get<std::string>();
        if (c.names[i].empty() || c.names[i].size()>128 || !seen.emplace(c.names[i],true).second) throw std::runtime_error("Duplicate or invalid joint name");
    }
    const auto read_array = [&](const char* key, JointArray& target) {
        const auto& a = j.at(key);
        if (!a.is_array() || a.size()!=c.count) throw std::runtime_error(std::string("Wrong array length: ")+key);
        for (std::size_t i=0; i<c.count; ++i) target[i] = number(a[i]);
    };
    read_array("lower",c.lower); read_array("upper",c.upper);
    read_array("velocity",c.velocity); read_array("acceleration",c.acceleration); read_array("home",c.home);
    for (std::size_t i=0; i<c.count; ++i)
        if (c.lower[i]>c.upper[i] || c.home[i]<c.lower[i] || c.home[i]>c.upper[i]
            || c.velocity[i]<=0 || c.acceleration[i]<=0) throw std::runtime_error("Invalid joint bounds/home/dynamics");
    const auto& groups = j.at("groups");
    if (!groups.is_object()) throw std::runtime_error("groups must be an object");
    for (auto it=groups.begin(); it!=groups.end(); ++it) {
        const auto g = group_index(it.key());
        if (!it.value().is_array()) throw std::runtime_error("Invalid group members");
        for (const auto& item : it.value()) {
            if (!item.is_string()) throw std::runtime_error("Invalid group member");
            const auto i = joint_index(c,item.get<std::string>());
            const Mask b = Mask{1} << i;
            if (c.groups[g]&b) throw std::runtime_error("Duplicate group member");
            c.groups[g] |= b;
        }
    }
    if (!c.arms()) throw std::runtime_error("At least one arm group is required");
    c.allowed_mask = j.contains("allowed_groups") ? group_selection(c,j.at("allowed_groups")) : c.arms();
    if (!(c.allowed_mask & c.arms())) throw std::runtime_error("No arm group allowed");
    c.control_hz = j.contains("control_hz") ? number(j.at("control_hz")) : 200;
    if (c.control_hz<10 || c.control_hz>1000) throw std::runtime_error("control_hz must be 10..1000");
    c.feedback_timeout_ns = duration(j,"feedback_timeout_ms",200,10000);
    c.command_timeout_ns = duration(j,"command_timeout_ms",150,5000);
    c.lease_timeout_ns = duration(j,"lease_timeout_ms",1000,30000);
    c.handover_timeout_ns = duration(j,"handover_timeout_ms",1000,30000);
    c.home_timeout_ns = duration(j,"home_timeout_ms",30000,300000);
    c.home_tolerance = j.contains("home_tolerance") ? number(j.at("home_tolerance")) : 0.02;
    if (c.home_tolerance<=0 || c.home_tolerance>0.5) throw std::runtime_error("Invalid home_tolerance");
    c.home_velocity = j.contains("home_velocity") ? number(j.at("home_velocity")) : .7;
    c.home_acceleration = j.contains("home_acceleration") ? number(j.at("home_acceleration")) : 1.4;
    if(c.home_velocity<=0||c.home_acceleration<=0)throw std::runtime_error("Invalid home velocity/acceleration");
    c.home_mask=c.allowed_mask;
    if(j.contains("home_groups"))c.home_mask &= group_selection(c,j.at("home_groups"));
    if(j.contains("home_joint_names")) {
        const auto& home_names=j.at("home_joint_names");
        if(!home_names.is_array())throw std::runtime_error("Invalid home_joint_names");
        Mask configured=0;
        for(const auto& item:home_names) {
            if(!item.is_string())throw std::runtime_error("Invalid home joint name");
            const Mask b=Mask{1}<<joint_index(c,item.get<std::string>());
            if(configured&b)throw std::runtime_error("Duplicate home joint name");
            configured|=b;
        }
        c.home_mask &= configured;
    }
    c.enable_grippers = boolean(j,"enable_grippers");
    c.enable_base = boolean(j,"enable_base");
    c.allow_independent_base = boolean(j,"allow_independent_base");
    if (j.contains("base_velocity_limit")) {
        const auto& b = j.at("base_velocity_limit");
        if (!b.is_array() || b.size()!=3) throw std::runtime_error("Invalid base_velocity_limit");
        for (std::size_t i=0;i<3;++i) { c.base_limit[i]=number(b[i]); if(c.base_limit[i]<=0)throw std::runtime_error("Invalid base limit"); }
    }
    return c;
}
Pose parse_pose(const Json& j, const char* key, bool required) {
    Pose result{0,0,0,0,0,0,1};
    if (!j.contains(key)) { if(required)throw std::runtime_error(std::string("MISSING_POSE:")+key); return result; }
    const auto& a=j.at(key);
    if (!a.is_array() || a.size()!=7) throw std::runtime_error("INVALID_POSE");
    for(std::size_t i=0;i<7;++i) { result[i]=number(a[i]); if(std::abs(result[i])>1000000)throw std::runtime_error("POSE_RANGE"); }
    double norm=0;
    for(std::size_t i=3;i<7;++i)norm+=result[i]*result[i];
    norm=std::sqrt(norm);
    if(norm<0.5 || norm>1.5)throw std::runtime_error("QUATERNION_NORM");
    for(std::size_t i=3;i<7;++i)result[i]/=norm;
    return result;
}
void joint_values(const Config& c,const Json& j,Event& e,bool require=true) {
    if (!j.contains("joint_names")) { if(require)throw std::runtime_error("MISSING_JOINT_NAMES"); return; }
    const auto& names=j.at("joint_names"); const auto& q=j.at("positions");
    if(!names.is_array() || !q.is_array() || names.size()!=q.size() || names.size()>kJoints || (require&&names.empty())) throw std::runtime_error("JOINT_ARRAY_LENGTH");
    e.has_velocities=j.contains("velocities");
    if(e.has_velocities && (!j.at("velocities").is_array() || j.at("velocities").size()!=names.size()))throw std::runtime_error("VELOCITY_ARRAY_LENGTH");
    for(std::size_t n=0;n<names.size();++n) {
        if(!names[n].is_string())throw std::runtime_error("INVALID_JOINT_NAME");
        auto i=joint_index(c,names[n].get<std::string>()); Mask b=Mask{1}<<i;
        if(e.mask&b)throw std::runtime_error("DUPLICATE_JOINT");
        e.mask|=b; e.positions[i]=number(q[n]);
        if(e.has_velocities)e.velocities[i]=number(j.at("velocities")[n]);
    }
}
void parse_aux(const Config& c,const Json& j,Event& e) {
    if(j.contains("base_velocity")) {
        const auto& a=j.at("base_velocity");
        if(!a.is_array()||a.size()!=3)throw std::runtime_error("INVALID_BASE");
        for(std::size_t i=0;i<3;++i) { e.base[i]=number(a[i]); if(std::abs(e.base[i])>c.base_limit[i]+1e-9)throw std::runtime_error("BASE_LIMIT"); }
        e.has_base=true;
    }
    if(j.contains("grippers")) {
        const auto& a=j.at("grippers");
        if(!a.is_array()||a.size()!=2)throw std::runtime_error("INVALID_GRIPPERS");
        for(std::size_t i=0;i<2;++i) { e.grippers[i]=number(a[i]); if(e.grippers[i]<0||e.grippers[i]>1)throw std::runtime_error("GRIPPER_LIMIT"); }
        e.has_grippers=true;
        if(j.contains("grippers_valid")) {
            const auto& valid=j.at("grippers_valid");
            if(!valid.is_array()||valid.size()!=2||!valid[0].is_boolean()||!valid[1].is_boolean())throw std::runtime_error("INVALID_GRIPPER_VALIDITY");
            e.grippers_valid={valid[0].get<bool>(),valid[1].get<bool>()};
        }
    }
}
Event parse_event(const Config& c,const Json& j,std::uint64_t client,Nanoseconds received) {
    if(!j.is_object())throw std::runtime_error("EXPECTED_OBJECT");
    Event e; e.client=client; e.received_ns=received;
    const auto id=string_value(j,"id"); if(id.size()>63)throw std::runtime_error("ID_TOO_LONG"); e.id.set(id);
    e.expected_boot_id.set(string_value(j,"expected_boot_id"));
    e.reason.set(string_value(j,"reason"));
    e.has_expected_epoch=j.contains("expected_epoch"); e.expected_epoch=uint_value(j,"expected_epoch");
    e.boot_id.set(string_value(j,"boot_id"));
    e.epoch=uint_value(j,"epoch"); e.lease_id=uint_value(j,"lease_id"); e.reference_epoch=uint_value(j,"reference_epoch");
    e.seq=uint_value(j,"seq");
    const auto sample=uint_value(j,"sample_mono_ns");
    if(sample>static_cast<std::uint64_t>(INT64_MAX))throw std::runtime_error("INVALID_SAMPLE_TIME");
    e.sample_ns=static_cast<Nanoseconds>(sample);
    const auto deadline=uint_value(j,"deadline_mono_ns");
    if(deadline>static_cast<std::uint64_t>(INT64_MAX))throw std::runtime_error("INVALID_DEADLINE");
    e.deadline_ns=static_cast<Nanoseconds>(deadline);
    const auto op=string_value(j,"op");
    if(op=="hello") {
        e.op=Op::Hello; e.role=parse_role(string_value(j,"role"));
        if(j.contains("capabilities")) { const auto& caps=j.at("capabilities"); e.fence_ack_cap=boolean(caps,"fence_ack"); e.physical_stop_ack=boolean(caps,"physical_stop_ack"); }
    } else if(op=="status") e.op=Op::Status;
    else if(op=="feedback") { e.op=Op::Feedback; e.ready=boolean(j,"ready"); joint_values(c,j,e); }
    else if(op=="fence_ack") { e.op=Op::FenceAck; e.stopped=boolean(j,"stopped"); }
    else if(op=="output_ack") { e.op=Op::OutputAck; e.accepted=boolean(j,"accepted"); }
    else if(op=="vr") {
        e.op=Op::Vr; e.sample_seq=uint_value(j,"sample_seq");
        const auto& tracking=j.at("tracked"); const auto& grip=j.at("grip");
        e.tracked={boolean(tracking,"left"),boolean(tracking,"right")}; e.head_tracked=boolean(tracking,"head");
        e.grip={boolean(grip,"left"),boolean(grip,"right")};
        if(j.contains("trigger")) {
            const auto& trigger=j.at("trigger");
            e.trigger[0]=trigger.contains("left")?number(trigger.at("left")):0;
            e.trigger[1]=trigger.contains("right")?number(trigger.at("right")):0;
            for(auto x:e.trigger)if(x<0||x>1)throw std::runtime_error("TRIGGER_LIMIT");
        }
        e.left=parse_pose(j,"left_pose",e.tracked[0]); e.right=parse_pose(j,"right_pose",e.tracked[1]); e.head=parse_pose(j,"head_pose",e.head_tracked);
        e.has_head=j.contains("head_pose"); e.base_independent=boolean(j,"base_independent"); parse_aux(c,j,e);
    } else if(op=="command" || op=="ik_result") {
        e.op=op=="command"?Op::Command:Op::IkResult;
        e.source=op=="command"?parse_owner(string_value(j,"source")):Owner::None;
        e.ttl_ns=duration(j,"ttl_ms",150,5000); joint_values(c,j,e); parse_aux(c,j,e);
    } else if(op=="renew") { e.op=Op::Renew; e.source=parse_owner(string_value(j,"source")); }
    else if(op=="recover") e.op=Op::Recover;
    else if(op=="session_start") { e.op=Op::Start; e.mode=parse_mode(string_value(j,"mode")); e.enabled=boolean(j,"dagger_enabled"); }
    else if(op=="pause") e.op=Op::Pause;
    else if(op=="resume") e.op=Op::Resume;
    else if(op=="session_end") e.op=Op::End;
    else if(op=="stop") e.op=Op::Stop;
    else if(op=="reference_reset") e.op=Op::ResetReference;
    else if(op=="home") e.op=Op::Home;
    else if(op=="set_dagger") { e.op=Op::SetDagger; e.enabled=boolean(j,"enabled"); }
    else throw std::runtime_error("UNKNOWN_OPERATION");
    if(j.contains("groups")) { e.requested_mask=group_selection(c,j.at("groups")); e.has_groups=true; }
    return e;
}
void add_joint_arrays(Json& j,const Config& c,Mask mask,const JointArray& positions,const JointArray* velocities=nullptr) {
    j["joint_names"]=Json::array(); j["positions"]=Json::array();
    if(velocities)j["velocities"]=Json::array();
    for(std::size_t i=0;i<c.count;++i)if(mask&(Mask{1}<<i)) {
        j["joint_names"].push_back(c.names[i]); j["positions"].push_back(positions[i]);
        if(velocities)j["velocities"].push_back((*velocities)[i]);
    }
}
Json state_json(const Config& c,const State& s) {
    Json j={{"boot_id",s.boot_id.c_str()},{"robot_id",c.robot_id},{"config_hash",c.config_hash},
        {"epoch",s.epoch},{"lease_id",s.lease_id},{"reference_epoch",s.reference_epoch},{"session_id",s.session_id},
        {"safety",name(s.safety)},{"mode",name(s.mode)},{"owner",name(s.owner)},{"requested_owner",name(s.requested_owner)},
        {"reason",s.reason.c_str()},{"paused",s.paused},{"dagger_enabled",s.dagger_enabled},{"return_paused",s.return_paused},
        {"transition_pending",s.transition_pending},{"handback_waiting",s.handback_waiting},
        {"backend_connected",s.backend_connected},{"backend_ready",s.backend_ready},{"physical_stop_ack",s.physical_stop_ack},
        {"fence_ack",s.fence_ack_cap},{"ik_connected",s.ik_connected},{"vr_connected",s.vr_connected},
        {"feedback_seq",s.feedback_seq},{"feedback_age_ns",s.feedback_age_ns},{"issued_mono_ns",s.now_ns},
        {"last_command_seq",s.last_command_seq},
        {"accepted_commands",s.accepted_commands},{"rejected_commands",s.rejected_commands},
        {"tick",s.ticks},{"overruns",s.overruns},{"cycle_max_ns",s.cycle_max_ns},
        {"audit_dropped",s.audit_dropped},{"outputs_dropped",s.outputs_dropped},
        {"grip",{{"left",s.grip[0]},{"right",s.grip[1]}}},
        {"release_required",{{"left",s.release_required[0]},{"right",s.release_required[1]}}},
        {"enable_grippers",c.enable_grippers},{"enable_base",c.enable_base},
        {"physical_stop_boundary",s.physical_stop_ack?"backend_acknowledged":"software_gate_only"},
        {"realtime_guarantee","soft_realtime_unqualified"}};
    j["configured_groups"]=Json::array(); j["available_groups"]=Json::array(); j["leased_groups"]=Json::array();
    j["fresh_joint_names"]=Json::array();
    for(std::size_t i=0;i<c.count;++i)if(s.fresh_feedback_mask&(Mask{1}<<i))j["fresh_joint_names"].push_back(c.names[i]);
    for(std::size_t g=0;g<kGroups;++g) {
        const auto m=c.groups[g]; if(!m)continue;
        if((m&c.allowed_mask)==m)j["configured_groups"].push_back(group_name(g));
        if((m&s.available_mask)==m)j["available_groups"].push_back(group_name(g));
        if((m&s.leased_mask)==m)j["leased_groups"].push_back(group_name(g));
    }
    add_joint_arrays(j,c,s.feedback_mask,s.feedback);
    j["commanded_positions"]=Json::array();
    for(std::size_t i=0;i<c.count;++i)if(s.feedback_mask&(Mask{1}<<i))j["commanded_positions"].push_back(s.commanded[i]);
    return j;
}
Json output_json(const Config& c,const Output& o) {
    Json j={{"type","output"},{"stage","core_gate"},{"boot_id",o.boot_id.c_str()},
        {"config_hash",c.config_hash},{"epoch",o.epoch},{"lease_id",o.lease_id},{"reference_epoch",o.reference_epoch},
        {"seq",o.seq},{"tick",o.tick},{"issued_mono_ns",o.issued_ns},{"deadline_mono_ns",o.deadline_ns},
        {"valid_for_ms",static_cast<double>(o.deadline_ns-o.issued_ns)/1000000.0},{"mode",name(o.mode)},
        {"grippers",o.grippers},{"grippers_valid",o.gripper_valid},{"base_velocity",o.base}};
    add_joint_arrays(j,c,o.mask,o.positions,&o.velocities); return j;
}
Json ik_json(const Config& c,const IkRequest& r) {
    Json j={{"type","ik_request"},{"boot_id",r.boot_id.c_str()},{"config_hash",c.config_hash},
        {"epoch",r.epoch},{"lease_id",r.lease_id},{"reference_epoch",r.reference_epoch},{"seq",r.seq},
        {"issued_mono_ns",r.issued_ns},{"deadline_mono_ns",r.deadline_ns},{"valid_for_ms",static_cast<double>(r.deadline_ns-r.issued_ns)/1000000.0},
        {"left_pose",r.left},{"right_pose",r.right},{"head_pose",r.head},
        {"left_reference_pose",r.left_reference},{"right_reference_pose",r.right_reference},{"head_reference_pose",r.head_reference},
        {"active_arms",{{"left",r.active[0]},{"right",r.active[1]}}},{"allow_body",r.allow_body},
        {"mode",name(r.mode)},{"source",name(r.source)},{"source_frame","unity_tracking"}};
    add_joint_arrays(j,c,r.mask,r.positions);
    j["reference_positions_left"]=Json::array(); j["reference_positions_right"]=Json::array();
    for(std::size_t i=0;i<c.count;++i)if(r.mask&(Mask{1}<<i)) {
        j["reference_positions_left"].push_back(r.reference_left[i]);
        j["reference_positions_right"].push_back(r.reference_right[i]);
    }
    j["reference_positions_head"]=j["reference_positions_left"];
    return j;
}
Json audit_json(const Config& c,const Audit& a) {
    Json j={{"type",a.kind==Audit::Kind::Ownership?"ownership_transition":"output_ack"},
        {"event_seq",a.event_seq},{"mono_ns",a.mono_ns},{"tick",a.tick},{"boot_id",a.boot_id.c_str()},
        {"epoch",a.epoch},{"config_hash",c.config_hash}};
    if(a.kind==Audit::Kind::Ownership) {
        j["old_owner"]=name(a.old_owner); j["new_owner"]=name(a.new_owner); j["reason"]=a.reason.c_str();
    } else { j["output_seq"]=a.output_seq; j["accepted"]=a.accepted; j["stage"]="backend_dispatch"; }
    return j;
}
std::string make_boot_id() {
    std::ifstream input("/proc/sys/kernel/random/uuid");
    std::string id; if(input>>id && id.size()<64)return id;
    std::random_device rd;
    return std::to_string(monotonic_now())+"-"+std::to_string(rd());
}
struct SocketGuard {
    int fd{-1}, lock_fd{-1}; std::string path;
    ~SocketGuard() { if(fd>=0)::close(fd); if(!path.empty())::unlink(path.c_str()); if(lock_fd>=0)::close(lock_fd); }
};
void bind_socket(SocketGuard& guard,const std::string& path) {
    if(path.empty()||path.size()>=sizeof(sockaddr_un::sun_path))throw std::runtime_error("Socket path is too long");
    const auto parent=std::filesystem::path(path).parent_path();
    if(parent.empty())throw std::runtime_error("Use an absolute socket path in a private directory");
    struct stat st{};
    if(::lstat(parent.c_str(),&st)!=0) {
        if(errno!=ENOENT || ::mkdir(parent.c_str(),0700)!=0 || ::lstat(parent.c_str(),&st)!=0)throw std::runtime_error("Cannot create socket directory");
    }
    if(!S_ISDIR(st.st_mode)||st.st_uid!=::getuid()||(st.st_mode&0077))throw std::runtime_error("Socket directory must be uid-owned mode 0700 and not a symlink");
    guard.lock_fd=::open((path+".lock").c_str(),O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(guard.lock_fd<0||::flock(guard.lock_fd,LOCK_EX|LOCK_NB)!=0)throw std::runtime_error("Another control process owns this socket");
    if(::lstat(path.c_str(),&st)==0) {
        if(!S_ISSOCK(st.st_mode)||st.st_uid!=::getuid())throw std::runtime_error("Refusing to replace non-owned socket path");
        if(::unlink(path.c_str())!=0)throw std::runtime_error("Cannot remove stale socket");
    } else if(errno!=ENOENT)throw std::runtime_error("Cannot inspect socket path");
    guard.fd=::socket(AF_UNIX,SOCK_SEQPACKET|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
    if(guard.fd<0)throw std::runtime_error("socket failed");
    sockaddr_un address{}; address.sun_family=AF_UNIX; std::memcpy(address.sun_path,path.c_str(),path.size()+1);
    if(::bind(guard.fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))!=0)throw std::runtime_error("bind failed");
    guard.path=path;
    if(::chmod(path.c_str(),0600)!=0||::listen(guard.fd,16)!=0)throw std::runtime_error("socket chmod/listen failed");
}
struct Peer {
    int fd{-1}; std::uint64_t id{0}; Role role{Role::None};
    std::deque<std::string> pending;
};
class Server {
    const Config& c_; Shared& shared_; int listener_;
    std::array<Peer,kClients> peers_{};
    std::array<std::uint64_t,kClients> deferred_disconnects_{};
    State state_{}; std::uint64_t next_client_{1}; Nanoseconds last_state_{0};
    std::array<char,kPacket> buffer_{};
    bool queue_event(const Event& e) {
        if(shared_.events.push(e))return true;
        shared_.overflow.store(true); return false;
    }
    void disconnect(Peer& p) {
        if(p.fd<0)return;
        ::close(p.fd); p.fd=-1;
        Event e; e.op=Op::Disconnect; e.client=p.id; e.received_ns=monotonic_now();
        if (!queue_event(e)) {
            for(auto& id:deferred_disconnects_)if(!id){id=p.id;break;}
        }
        p.pending.clear(); p.role=Role::None;
    }
    bool try_send(Peer& p,const std::string& s) {
        const auto n=::send(p.fd,s.data(),s.size(),MSG_DONTWAIT|MSG_NOSIGNAL);
        if(n==static_cast<ssize_t>(s.size()))return true;
        if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK))return false;
        disconnect(p); return false;
    }
    void reliable(Peer& p,const std::string& s) {
        if(p.fd<0)return;
        if(p.pending.empty()&&try_send(p,s))return;
        if(p.fd<0)return;
        // Disconnect is observable to a recorder and marks its segment incomplete.
        // Neither an audit consumer nor a backend may block the control thread.
        if(p.pending.size()>=256) { disconnect(p); return; }
        p.pending.push_back(s);
    }
    Peer* find(std::uint64_t id) {
        for(auto& p:peers_)if(p.fd>=0&&p.id==id)return &p;
        return nullptr;
    }
    void flush(Peer& p) {
        for(unsigned n=0;p.fd>=0&&!p.pending.empty()&&n<64;++n) {
            if(!try_send(p,p.pending.front()))break;
            p.pending.pop_front();
        }
    }
    void accept_peers() {
        for(unsigned n=0;n<16;++n) {
            const int fd=::accept4(listener_,nullptr,nullptr,SOCK_NONBLOCK|SOCK_CLOEXEC);
            if(fd<0)return;
            ucred credentials{}; socklen_t length=sizeof(credentials);
            if(::getsockopt(fd,SOL_SOCKET,SO_PEERCRED,&credentials,&length)!=0||credentials.uid!=::getuid()) { ::close(fd); continue; }
            Peer* slot=nullptr; for(auto& p:peers_)if(p.fd<0){slot=&p;break;}
            if(!slot){::close(fd);continue;}
            slot->fd=fd; slot->id=next_client_++; slot->role=Role::None;
        }
    }
    void receive(Peer& p) {
        for(unsigned n=0;p.fd>=0&&n<16;++n) {
            iovec iov{buffer_.data(),buffer_.size()}; msghdr message{};message.msg_iov=&iov;message.msg_iovlen=1;
            const auto size=::recvmsg(p.fd,&message,MSG_DONTWAIT);
            if(size<0) { if(errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)disconnect(p); return; }
            if(size==0 || (message.msg_flags&MSG_TRUNC)) { disconnect(p); return; }
            const auto received=monotonic_now();
            Json j;
            try {
                j=Json::parse(buffer_.data(),buffer_.data()+size);
                auto e=parse_event(c_,j,p.id,received);
                e.ingress_epoch=shared_.current_epoch.load(std::memory_order_acquire);
                if(e.op==Op::Stop && p.role!=Role::None
                    && (!e.has_expected_epoch||e.expected_epoch==shared_.current_epoch.load())
                    && (e.expected_boot_id.empty()||e.expected_boot_id.equals(state_.boot_id.c_str())))
                    shared_.stop_requested.store(true);
                if(!queue_event(e))return;
            } catch(const std::exception& error) {
                Event e; e.client=p.id; e.received_ns=received; e.parse_error.set(error.what());
                if(j.is_object()&&j.contains("id")&&j["id"].is_string())e.id.set(j["id"].get<std::string>());
                if(!e.id.empty())queue_event(e);
            }
        }
    }
public:
    Server(const Config& c,Shared& shared,int listener):c_(c),shared_(shared),listener_(listener) {}
    ~Server(){for(auto& p:peers_)if(p.fd>=0)::close(p.fd);}
    void pump() {
        for(auto& id:deferred_disconnects_)if(id) {
            Event event;event.op=Op::Disconnect;event.client=id;event.received_ns=monotonic_now();
            if(shared_.events.push(event))id=0;
            else break;
        }
        shared_.state.consume(state_);
        Outbound message;
        for(unsigned n=0;n<512 && shared_.reliable.pop(message);++n) {
            auto* p=find(message.client); if(!p)continue;
            if(message.kind==Outbound::Kind::Response) {
                if(message.ok && message.assigned_role!=Role::None)p->role=message.assigned_role;
                reliable(*p,Json{{"type","response"},{"id",message.id.c_str()},{"ok",message.ok},
                    {"code",message.code.c_str()},{"state",state_json(c_,message.state)}}.dump());
            } else if(message.kind==Outbound::Kind::Fence) {
                reliable(*p,Json{{"type","fence"},{"boot_id",message.state.boot_id.c_str()},
                    {"epoch",message.state.epoch},{"seq",message.seq},{"reason",message.state.reason.c_str()},
                    {"issued_mono_ns",message.state.now_ns}}.dump());
            } else disconnect(*p);
        }
        for(auto& p:peers_)if(p.fd>=0)flush(p);
        Audit audit;
        for(unsigned n=0;n<2048&&shared_.audit.pop(audit);++n) {
            const auto text=audit_json(c_,audit).dump();
            for(auto& p:peers_)if(p.fd>=0&&(p.role==Role::Api||p.role==Role::Recorder))reliable(p,text);
        }
        Output recorded;
        for(unsigned n=0;n<2048&&shared_.recording_outputs.pop(recorded);++n) {
            bool wanted=false;for(const auto& p:peers_)wanted=wanted||(p.fd>=0&&p.role==Role::Recorder);
            if(!wanted)continue;
            const auto text=output_json(c_,recorded).dump();
            for(auto& p:peers_)if(p.fd>=0&&p.role==Role::Recorder)reliable(p,text);
        }
        // Fences are drained before this latest-only egress. The backend must
        // enforce the same generation at its actual hardware send boundary.
        Output output;
        if(shared_.output.consume(output)&&output.epoch==shared_.current_epoch.load(std::memory_order_acquire)
            &&output.deadline_ns>monotonic_now()) {
            const auto text=output_json(c_,output).dump();
            for(auto& p:peers_)if(p.fd>=0&&p.role==Role::Backend&&p.pending.empty())
                if(!try_send(p,text)&&p.fd>=0)disconnect(p);
        }
        IkRequest request;
        if(shared_.ik.consume(request)&&request.epoch==shared_.current_epoch.load(std::memory_order_acquire)
            &&request.deadline_ns>monotonic_now()) {
            const auto text=ik_json(c_,request).dump();
            for(auto& p:peers_)if(p.fd>=0&&p.role==Role::Ik&&p.pending.empty())try_send(p,text);
        }
        const auto now=monotonic_now();
        if(now-last_state_>=50000000) {
            auto j=state_json(c_,state_);j["type"]="state";const auto text=j.dump();
            for(auto& p:peers_)if(p.fd>=0&&p.role!=Role::None&&p.pending.empty())try_send(p,text);
            last_state_=now;
        }
        std::array<pollfd,kClients+1> fds{};
        fds[0]={listener_,POLLIN,0};
        for(std::size_t i=0;i<peers_.size();++i)fds[i+1]={peers_[i].fd,POLLIN,0};
        if(::poll(fds.data(),fds.size(),2)>0) {
            if(fds[0].revents&POLLIN)accept_peers();
            for(std::size_t i=0;i<peers_.size();++i)if(peers_[i].fd>=0) {
                if(fds[i+1].revents&(POLLERR|POLLHUP|POLLNVAL))disconnect(peers_[i]);
                else if(fds[i+1].revents&POLLIN)receive(peers_[i]);
            }
        }
    }
};
void periodic_loop(const Config& config,Shared& shared,const std::string& boot) {
    Controller controller(config,shared,boot);
    const auto period=static_cast<Nanoseconds>(1000000000.0/config.control_hz);
    auto deadline=monotonic_now(); Nanoseconds execution=0;
    while(shared.running.load(std::memory_order_relaxed)) {
        const auto start=monotonic_now();
        // Stops/overflows are consumed before ordinary queued work. Commands
        // already in the queue carry their original receive time and lease.
        controller.tick(start,execution);
        Event e;
        for(unsigned n=0;n<64&&shared.events.pop(e);++n)controller.accept(e,monotonic_now());
        execution=monotonic_now()-start;
        deadline+=period;
        if(deadline<=monotonic_now())deadline=monotonic_now()+period;
        timespec wake{static_cast<time_t>(deadline/1000000000),static_cast<long>(deadline%1000000000)};
        while(::clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&wake,nullptr)==EINTR&&shared.running.load()){}
    }
}
} // namespace

int main(int argc,char** argv) {
    try {
        std::string socket_path,config_path;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help") {std::cout<<"hc-control --socket /private/path/control.sock --config robot.json\n";return 0;}
            if((arg=="--socket"||arg=="--config")&&i+1<argc) {if(arg=="--socket")socket_path=argv[++i];else config_path=argv[++i];}
            else throw std::runtime_error("Unknown or incomplete argument: "+arg);
        }
        if(socket_path.empty()||config_path.empty())throw std::runtime_error("--socket and --config are required");
        const auto config=read_config(config_path);
        SocketGuard socket; bind_socket(socket,socket_path);
        const auto boot=make_boot_id();
        auto shared=std::make_unique<Shared>();
        std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
        std::thread control(periodic_loop,std::cref(config),std::ref(*shared),std::cref(boot));
        try {
            Server server(config,*shared,socket.fd);
            std::cout<<"hc-control ready socket="<<socket_path<<" boot_id="<<boot<<" mode=stopped\n"<<std::flush;
            Nanoseconds shutdown_at=0;
            while(!shutdown_at||monotonic_now()<shutdown_at) {
                if(stop_signal&&!shutdown_at) {shared->stop_requested.store(true);shutdown_at=monotonic_now()+100000000;}
                server.pump();
            }
        } catch(...) {shared->running.store(false);control.join();throw;}
        shared->running.store(false);control.join();return 0;
    } catch(const std::exception& e) {std::cerr<<"hc-control: "<<e.what()<<'\n';return 1;}
}
