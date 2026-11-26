// Run with the platform's actual startDesktop function, a live WebRTC peer and test poses.
window.runDeviceQA = async function () {
  const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
  const until = async fn => {
    const end = performance.now() + 18000;
    while (!fn()) {
      if (performance.now() > end) throw Error("timed out waiting for desktop state");
      await wait(10);
    }
  };
  const sid = (await (await fetch("/session")).json()).id;
  const video = document.querySelector("video");
  const pc = new RTCPeerConnection({iceServers: []});
  const life = new AbortController();
  const orig = pc.createDataChannel.bind(pc);
  let reliable, actions, ready=false, on=false, seq=0, emit=true;
  let input=null, timer=null, videoFrames=0, firstFrameAt=0;
  const rtts=[], sent=new Map();
  const t0=performance.now();
  pc.createDataChannel=(label,opts)=>{
    const ch=orig(label,opts);
    if(label==="hc-operator.v1") {
      reliable=ch;
      ch.addEventListener("message",e=>{
        const m=JSON.parse(e.data);
        if(m.type==="latency_echo" && sent.has(m.request_id)) {
          rtts.push(performance.now()-sent.get(m.request_id));
          sent.delete(m.request_id);
        }
      });
    }
    return ch;
  };
  const render=()=>{
    if(!firstFrameAt) firstFrameAt=performance.now();
    videoFrames++;
    video.requestVideoFrameCallback(render);
  };
  video.requestVideoFrameCallback(render);
  pc.ontrack=({track})=>{video.srcObject=new MediaStream([track]);void video.play();};
  pc.addTransceiver("video",{direction:"recvonly"});
  const motion=pc.createDataChannel("vr-control.v1",{ordered:false,maxRetransmits:0});
  const pose={position:[0,.2,0],quaternion:[0,0,0,1]};
  const hand={trigger:.75,grip:0,held_mask:0,primary_axis:[0,0],secondary_axis:[0,0]};
  timer=setInterval(()=>{
    if(!emit) return;
    input={sent_ms:Date.now(),age_ms:0,frame:{sequence:++seq,protocol_version:2,
      tracking:{head:true,left:true,right:true},poses:{head:pose,left:pose,right:pose},inputs:{left:hand,right:hand}}};
  },10);
  try {
    HCWeb.startDesktop(pc,motion,sid,life.signal,{video,latest:()=>input,
      operator:(a,r,e)=>{actions=a;ready=r;on=e;}},()=>{});
    await pc.setLocalDescription(await pc.createOffer());
    await until(()=>pc.iceGatheringState==="complete");
    await fetch("/offer",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(pc.localDescription)});
    let answer=null;
    const end=performance.now()+18000;
    while(!answer && performance.now()<end){await wait(100);answer=(await (await fetch("/answer")).json()).sdp;}
    if(!answer) throw Error("missing device answer");
    await pc.setRemoteDescription({type:"answer",sdp:answer});
    await until(()=>ready && videoFrames>2);
    actions.enable();
    await until(()=>on);
    for(let n=0;n<42;n++){
      const id=crypto.randomUUID();
      sent.set(id,performance.now());
      reliable.send(JSON.stringify({version:1,session_id:sid,type:"latency_probe",input_profile:"hc-pico-v2",request_id:id}));
      await wait(150);
    }
    const stats=await pc.getStats();
    const pair=[...stats.values()].find(s=>s.type==="candidate-pair" && s.state==="succeeded" && s.nominated);
    const candidate=pair && stats.get(pair.remoteCandidateId);
    emit=false;input=null;
    await until(()=>!on);
    if(rtts.length<35 || videoFrames<50 || !candidate) throw Error("insufficient measurements");
    rtts.sort((a,b)=>a-b);
    const q=p=>+rtts[Math.ceil(rtts.length*p)-1].toFixed(2);
    return {first_frame_ms:Math.round(firstFrameAt-t0),videoFrames,rtt_probes:rtts.length,
      control_rtt_ms:{p50:q(.5),p95:q(.95),p99:q(.99)},remote:candidate.address,
      candidate:candidate.candidateType,protocol:candidate.protocol,disabledAfterLoss:!on};
  } finally {
    life.abort();clearInterval(timer);pc.close();
  }
};
