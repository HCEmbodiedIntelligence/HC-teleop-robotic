#include "hc_control/runtime.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

using namespace hc;
#define CHECK(x) do { if (!(x)) throw std::runtime_error(std::string(__func__)+":"+std::to_string(__LINE__)+" " #x); } while(false)
namespace {
Config config() {
    Config c; c.count=3; c.names[0]="left";c.names[1]="right";c.names[2]="waist";
    for(std::size_t i=0;i<c.count;++i){c.lower[i]=-2;c.upper[i]=2;c.velocity[i]=0.6;c.acceleration[i]=2.4;c.home[i]=0;}
    c.groups[0]=1;c.groups[1]=2;c.groups[4]=4;c.allowed_mask=7;
    c.enable_grippers=true;c.enable_base=true;c.allow_independent_base=true;
    return c;
}
struct Fixture {
    Config cfg=config();
    std::unique_ptr<Shared> shared=std::make_unique<Shared>();
    Controller core{cfg,*shared,"test-boot"};
    Nanoseconds now=1000000000;
    std::uint64_t feedback_seq=0,vr_seq=0;
    Fixture() {
        hello(1,Role::Api);hello(2,Role::Backend);hello(3,Role::Vr);hello(4,Role::Program);hello(5,Role::Ik);hello(6,Role::Exo);hello(7,Role::Replay);
        feedback();
    }
    void send(Event e){e.received_ns=now;core.accept(e,now);}
    void hello(std::uint64_t client,Role role){Event e;e.op=Op::Hello;e.client=client;e.role=role;e.fence_ack_cap=true;send(e);}
    void feedback(Mask mask=7){Event e;e.op=Op::Feedback;e.client=2;e.seq=++feedback_seq;e.ready=true;e.mask=mask;e.positions=core.state().commanded;e.sample_ns=now;send(e);}
    void advance(Nanoseconds ns=5000000,bool fb=true){now+=ns;if(fb)feedback();core.tick(now);}
    void manage(Op op){Event e;e.op=op;e.client=1;send(e);}
    void ack(){Event e;e.op=Op::FenceAck;e.client=2;e.boot_id.set("test-boot");e.epoch=core.state().epoch;e.stopped=true;send(e);}
    void recover(){manage(Op::Recover);CHECK(core.state().transition_pending);ack();CHECK(core.state().safety==Safety::Ready);}
    void start(Mode mode=Mode::Program,bool dagger=false){Event e;e.op=Op::Start;e.client=1;e.mode=mode;e.enabled=dagger;send(e);CHECK(core.state().transition_pending);ack();CHECK(core.state().mode==mode);}
    Event command(Owner source=Owner::Program,Mask mask=3,std::uint64_t seq=1) {
        Event e;e.op=Op::Command;e.client=source==Owner::Program?4:source==Owner::Exo?6:7;e.source=source;e.mask=mask;
        e.seq=seq;e.ttl_ns=100000000;e.boot_id.set("test-boot");e.epoch=core.state().epoch;e.lease_id=core.state().lease_id;e.reference_epoch=core.state().reference_epoch;
        e.positions[0]=0.3;e.positions[1]=-0.3;return e;
    }
    void vr(bool left,bool right,bool tracked=true,bool new_sample=true) {
        Event e;e.op=Op::Vr;e.client=3;e.seq=++vr_seq;e.sample_seq=new_sample?vr_seq:vr_seq-1;e.sample_ns=now;e.tracked={tracked,tracked};e.grip={left,right};e.trigger={0.8,0.4};e.head_tracked=true;send(e);
    }
};
void boot_and_fencing() {
    Fixture f;CHECK(f.core.state().safety==Safety::Stopped);f.recover();f.start();
    auto old=f.command();f.send(old);CHECK(f.core.state().accepted_commands==1);
    f.manage(Op::Stop);CHECK(f.core.state().owner==Owner::None);CHECK(f.core.state().safety==Safety::Stopped);
    f.recover();f.start();f.send(old);CHECK(f.core.state().accepted_commands==1);CHECK(f.core.state().rejected_commands>=1);
    auto wrong=f.command();wrong.boot_id.set("previous-boot");f.send(wrong);CHECK(f.core.state().accepted_commands==1);
}
void partial_feedback_and_staleness() {
    Fixture f;f.recover();
    f.now+=250000000;f.feedback(1);f.core.tick(f.now);CHECK(f.core.state().safety==Safety::Ready);
    Event start;start.op=Op::Start;start.client=1;start.mode=Mode::Program;start.has_groups=true;start.requested_mask=1;f.send(start);f.ack();
    CHECK(f.core.state().leased_mask==1);
    f.advance(250000000,false);CHECK(f.core.state().safety==Safety::Fault);CHECK(f.core.state().owner==Owner::None);
}
void dagger_bundle_and_paused_return() {
    Fixture f;f.recover();f.start(Mode::Program,true);f.send(f.command());
    f.vr(false,false);f.vr(true,false);CHECK(f.core.state().requested_owner==Owner::Human);f.ack();CHECK(f.core.state().owner==Owner::Human);
    const auto epoch=f.core.state().epoch;f.vr(true,true);CHECK(f.core.state().owner==Owner::Human);CHECK(f.core.state().epoch==epoch);
    f.vr(false,true);CHECK(f.core.state().owner==Owner::Human);
    f.vr(false,false);CHECK(f.core.state().transition_pending);f.ack();CHECK(f.core.state().handback_waiting);CHECK(f.core.state().owner==Owner::None);
    f.send(f.command());CHECK(f.core.state().owner==Owner::Program);CHECK(!f.core.state().handback_waiting);
    f.manage(Op::Pause);f.ack();CHECK(f.core.state().paused);
    f.vr(false,false);f.vr(true,false);f.ack();CHECK(f.core.state().owner==Owner::Human);CHECK(f.core.state().return_paused);
    f.vr(false,false);f.ack();CHECK(f.core.state().paused);CHECK(f.core.state().owner==Owner::None);
}
void tracking_loss_is_not_release() {
    Fixture f;f.recover();f.start(Mode::Program,true);f.vr(false,false);f.vr(true,false);f.ack();
    f.vr(false,false,false);f.ack();CHECK(f.core.state().paused);CHECK(!f.core.state().handback_waiting);CHECK(f.core.state().owner==Owner::None);
    f.vr(true,false);CHECK(f.core.state().owner==Owner::None); // held on tracking recovery
    f.vr(false,false);f.vr(true,false);f.ack();CHECK(f.core.state().owner==Owner::Human);
}
void ik_older_than_latest_and_stale_rejection() {
    Fixture f;f.recover();f.start(Mode::Teleop);f.vr(false,false);f.vr(true,false);f.advance();
    IkRequest old;CHECK(f.shared->ik.consume(old));
    f.vr(true,false);f.advance();IkRequest newer;CHECK(f.shared->ik.consume(newer));CHECK(newer.seq>old.seq);
    Event e;e.op=Op::IkResult;e.client=5;e.boot_id=old.boot_id;e.epoch=old.epoch;e.lease_id=old.lease_id;e.reference_epoch=old.reference_epoch;e.seq=old.seq;e.mask=1;e.positions[0]=0.1;e.ttl_ns=100000000;
    f.send(e);CHECK(f.core.state().accepted_commands==1); // slower solver is not starved
    f.send(e);CHECK(f.core.state().accepted_commands==1);
    e.seq=newer.seq;f.vr(false,false);f.send(e);CHECK(f.core.state().accepted_commands==1); // clutch invalidates result
}
void fixed_limits_and_acceleration() {
    Fixture f;f.recover();f.start();
    auto invalid=f.command();invalid.positions[0]=20;f.send(invalid);CHECK(f.core.state().accepted_commands==0);
    double last_v=0,last_q=0;
    for(unsigned i=0;i<120;++i) {
        if(i%5==0)f.send(f.command(Owner::Program,3,i+1));
        f.advance();Output output;CHECK(f.shared->output.consume(output));
        CHECK(std::abs(output.velocities[0])<=0.6000001);
        CHECK(std::abs(output.velocities[0]-last_v)<=2.4*0.005+1e-8);
        CHECK(std::abs(output.positions[0]-last_q)<=0.6*0.005+1e-8);
        CHECK(output.positions[0]>=-2&&output.positions[0]<=2);
        last_v=output.velocities[0];last_q=output.positions[0];
    }
}
void leases_replay_home_and_overflow() {
    Fixture f;f.recover();f.start(Mode::Replay);f.send(f.command(Owner::Replay));f.advance(120000000);CHECK(f.core.state().transition_pending);f.ack();CHECK(f.core.state().paused);
    f.manage(Op::End);f.ack();CHECK(f.core.state().mode==Mode::Idle);
    f.manage(Op::Home);f.ack();f.advance();CHECK(!f.core.state().transition_pending);
    for(unsigned n=0;n<25&&!f.core.state().transition_pending;++n)f.advance();
    CHECK(f.core.state().transition_pending);f.ack();CHECK(f.core.state().mode==Mode::Idle);
    f.start(Mode::Exo);f.send(f.command(Owner::Exo));f.shared->overflow.store(true);f.advance();CHECK(f.core.state().safety==Safety::Stopped);
}
void expected_epoch_and_exclusive_backend() {
    Fixture f;f.recover();Event e;e.op=Op::Start;e.client=1;e.mode=Mode::Program;e.has_expected_epoch=true;e.expected_epoch=f.core.state().epoch-1;f.send(e);CHECK(f.core.state().mode==Mode::Idle);
    e=Event{};e.op=Op::Hello;e.client=20;e.role=Role::Backend;e.fence_ack_cap=true;f.send(e);CHECK(f.core.state().rejected_commands==2);
    e=Event{};e.op=Op::Feedback;e.client=20;e.seq=100;e.mask=1;e.ready=true;f.send(e);CHECK(f.core.state().feedback_seq==1);
}
void ownership_audit_and_recording() {
    Fixture f;f.recover();f.start();f.advance();f.manage(Op::Stop);
    Audit event;bool seen_program=false,seen_none=false;
    while(f.shared->audit.pop(event)){seen_program=seen_program||event.new_owner==Owner::Program;seen_none=seen_none||(event.old_owner==Owner::Program&&event.new_owner==Owner::None);}
    CHECK(seen_program&&seen_none);Output output;CHECK(f.shared->recording_outputs.pop(output));CHECK(output.tick>0);
}
void management_queue_and_stop_reason() {
    Fixture f;f.recover();const auto prior=f.core.state().epoch;
    Event stop;stop.op=Op::Stop;stop.client=3;stop.reason.set("VR_SAMPLE_STALE");f.send(stop);
    CHECK(f.core.state().reason.equals("VR_SAMPLE_STALE"));
    Event recover;recover.op=Op::Recover;recover.client=1;recover.ingress_epoch=prior;f.send(recover);
    CHECK(f.core.state().safety==Safety::Stopped);CHECK(!f.core.state().transition_pending);
}
void input_deadline_cannot_be_renewed() {
    Fixture f;f.recover();f.start();
    auto e=f.command();e.sample_ns=f.now-200000000;f.send(e);CHECK(f.core.state().accepted_commands==0);
    e=f.command();e.sample_ns=f.now+1;f.send(e);CHECK(f.core.state().accepted_commands==0);
    e=f.command();e.deadline_ns=f.now-1;f.send(e);CHECK(f.core.state().accepted_commands==0);
    e=f.command();e.sample_ns=f.now-20000000;e.deadline_ns=f.now+10000000;f.send(e);CHECK(f.core.state().accepted_commands==1);
    f.advance(15000000);CHECK(f.core.state().transition_pending);
}
void disabled_dagger_does_not_change_program_reference() {
    Fixture f;f.recover();f.start(Mode::Program,false);f.send(f.command());
    const auto reference=f.core.state().reference_epoch;
    f.vr(false,false);f.vr(true,false);f.vr(false,false);
    CHECK(f.core.state().owner==Owner::Program);CHECK(f.core.state().reference_epoch==reference);
    f.send(f.command(Owner::Program,3,2));CHECK(f.core.state().accepted_commands==2);
}
void configured_home_scope_and_dynamics() {
    Fixture f;f.cfg.home[0]=.3;f.cfg.home[1]=.3;f.cfg.home_mask=1;f.cfg.home_velocity=.1;f.cfg.home_acceleration=.2;
    f.recover();f.manage(Op::Home);f.ack();CHECK(f.core.state().leased_mask==1);
    double last_v=0;
    for(unsigned i=0;i<40;++i){f.advance();Output output;CHECK(f.shared->output.consume(output));
        CHECK(output.positions[1]==0);CHECK(output.velocities[0]<=.1+1e-9);
        CHECK(std::abs(output.velocities[0]-last_v)<=.2*.005+1e-9);last_v=output.velocities[0];}
}
void feedback_joint_limits_fail_closed() {
    Fixture f;f.recover();f.start();
    Event small_overshoot;small_overshoot.op=Op::Feedback;small_overshoot.client=2;small_overshoot.seq=++f.feedback_seq;
    small_overshoot.ready=true;small_overshoot.mask=1;small_overshoot.positions[0]=2.001;f.send(small_overshoot);
    f.core.tick(f.now);
    CHECK(f.core.state().safety==Safety::Ready);
    CHECK((f.core.state().fresh_feedback_mask&1u)!=0u);
    Event feedback;feedback.op=Op::Feedback;feedback.client=2;feedback.seq=++f.feedback_seq;
    feedback.ready=true;feedback.mask=1;feedback.positions[0]=2.1;f.send(feedback);
    CHECK(f.core.state().safety==Safety::Fault);CHECK(f.core.state().reason.equals("FEEDBACK_JOINT_LIMIT"));
}
void latest_buffer_concurrency() {
    struct Value {std::uint64_t seq;std::array<std::uint64_t,64> values;};
    Latest<Value> latest;std::atomic<bool> finished{false};
    std::thread producer([&]{for(std::uint64_t n=1;n<200000;++n){Value value;value.seq=n;value.values.fill(n);latest.publish(value);}finished.store(true);});
    std::uint64_t last=0;unsigned reads=0;
    while(!finished.load()){Value value;if(latest.consume(value)){CHECK(value.seq>=last);for(auto v:value.values)CHECK(v==value.seq);last=value.seq;++reads;}}
    producer.join();CHECK(reads>0);
}
void ready_hold_and_transitions_do_not_chase_gravity_sag() {
    Fixture f;
    auto push_fb=[&](double q0){
        Event e;e.op=Op::Feedback;e.client=2;e.seq=++f.feedback_seq;e.ready=true;e.mask=7;
        e.positions={q0,-0.2,0.1};e.sample_ns=f.now;f.send(e);
    };
    push_fb(0.4);
    f.recover();
    f.advance(5000000,false);
    Output out;
    CHECK(f.shared->output.consume(out));
    CHECK(out.mode==OutputMode::Hold);
    CHECK(std::abs(out.positions[0]-0.4)<1e-12);

    for(int step=0;step<10;++step){
        f.now+=5000000;
        push_fb(0.4-0.01*(step+1));
        f.core.tick(f.now);
        CHECK(f.shared->output.consume(out));
        CHECK(out.mode==OutputMode::Hold);
        CHECK(std::abs(out.positions[0]-0.4)<1e-12);
    }

    Event start_e;start_e.op=Op::Start;start_e.client=1;start_e.mode=Mode::Teleop;f.send(start_e);
    f.now+=5000000;push_fb(0.28);f.core.tick(f.now);
    CHECK(f.shared->output.consume(out));
    CHECK(out.mode==OutputMode::Stop);
    CHECK(std::abs(out.positions[0]-0.4)<1e-12);
    f.ack();
    f.now+=5000000;push_fb(0.28);f.core.tick(f.now);
    CHECK(f.shared->output.consume(out));
    CHECK(out.mode==OutputMode::Hold);
    CHECK(std::abs(out.positions[0]-0.4)<1e-12);

    f.vr(false,false);
    f.now+=5000000;push_fb(0.28);f.core.tick(f.now);
    f.vr(true,false);
    f.now+=5000000;push_fb(0.28);f.core.tick(f.now);
    IkRequest ik;
    CHECK(f.shared->ik.consume(ik));
    CHECK(std::abs(ik.reference_left[0]-0.4)<1e-12);

    f.manage(Op::Pause);f.now+=5000000;push_fb(0.25);f.core.tick(f.now);f.ack();
    f.now+=5000000;push_fb(0.25);f.core.tick(f.now);
    CHECK(f.shared->output.consume(out));
    CHECK(std::abs(out.positions[0]-0.4)<1e-12);

    f.manage(Op::Stop);
    f.now+=5000000;push_fb(0.25);f.core.tick(f.now);
    CHECK(f.shared->output.consume(out));
    CHECK(out.mode==OutputMode::Stop);
    CHECK(std::abs(out.positions[0]-0.25)<1e-12);

    f.recover();
    f.now+=5000000;push_fb(0.25);f.core.tick(f.now);
    CHECK(f.shared->output.consume(out));
    CHECK(out.mode==OutputMode::Hold);
    CHECK(std::abs(out.positions[0]-0.25)<1e-12);
}
}
int main() {
    try {
        boot_and_fencing();partial_feedback_and_staleness();dagger_bundle_and_paused_return();tracking_loss_is_not_release();
        ik_older_than_latest_and_stale_rejection();fixed_limits_and_acceleration();leases_replay_home_and_overflow();
        expected_epoch_and_exclusive_backend();ownership_audit_and_recording();management_queue_and_stop_reason();input_deadline_cannot_be_renewed();disabled_dagger_does_not_change_program_reference();configured_home_scope_and_dynamics();feedback_joint_limits_fail_closed();latest_buffer_concurrency();ready_hold_and_transitions_do_not_chase_gravity_sag();
        std::cout<<"16 control contract tests passed\n";return 0;
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
