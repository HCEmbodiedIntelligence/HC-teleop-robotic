// End-to-end fixture: genuine platform reader and desktop WebRTC sender, simulated camera and robot.
async () => {
  sessionStorage.setItem("hc.teleop.ticket", location.hash.split("=")[1]);
  const life = new AbortController();
  void HCWeb.readWorkstation("release-test", life.signal, state => { window.qa = state; });
  const operator = new RTCPeerConnection({iceServers: []});
  const robot = new RTCPeerConnection({iceServers: []});
  const video = document.createElement("video"); video.muted = true; document.body.append(video);
  operator.ontrack = ({track}) => { video.srcObject = new MediaStream([track]); void video.play(); };
  const canvas = document.createElement("canvas"); canvas.width = 96; canvas.height = 72;
  const ctx = canvas.getContext("2d");
  let n = 0;
  setInterval(() => { ctx.fillStyle = n++ % 2 ? "blue" : "red"; ctx.fillRect(0,0,96,72); }, 40);
  const camera = canvas.captureStream(25);
  robot.addTrack(camera.getVideoTracks()[0], camera);
  operator.addTransceiver("video",{direction:"recvonly"});
  const motion = operator.createDataChannel("vr-control.v1",{ordered:false,maxRetransmits:0});
  window.received = [];
  robot.ondatachannel = ({channel}) => {
    channel.onmessage = ({data}) => {
      const value = JSON.parse(data);
      if (channel.label === "vr-control.v1") { window.received.push(value); return; }
      if (value.type === "hello") channel.send(JSON.stringify({...value,type:"ready"}));
      if (value.type === "enable") channel.send(JSON.stringify({...value,type:"enabled"}));
      if (value.type === "disable") window.stopped = true;
    };
  };
  HCWeb.startDesktop(operator,motion,"session-test",life.signal,{
    video, latest:()=>window.qa,
    operator: (actions,ready,on) => {window.actions=actions; window.ready=ready; window.enabled=on;},
  },status=>{window.videoReady=status.video});
  async function gather(pc, desc) {
    await pc.setLocalDescription(desc);
    if (pc.iceGatheringState !== "complete") await new Promise(resolve => {
      pc.addEventListener("icegatheringstatechange",()=>{ if(pc.iceGatheringState==="complete")resolve(); });
    });
    return pc.localDescription;
  }
  await robot.setRemoteDescription(await gather(operator,await operator.createOffer()));
  await operator.setRemoteDescription(await gather(robot,await robot.createAnswer()));
}
