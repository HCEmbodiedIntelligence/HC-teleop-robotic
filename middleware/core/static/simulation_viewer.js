// Loaded only while the optional browser view is enabled. Three.js is vendored locally.
const graphics = () => Promise.all([
  import('./vendor/three/three.module.js'),
  import('./vendor/three/OrbitControls.js'),
  import('./vendor/three/STLLoader.js'),
]);

export function mountSimulationViewer({container, status, signal, loadGraphics = graphics}) {
  let disposed = false, scene, renderer, controls, camera, robot, modelAbort;
  let ws, reconnect, frameRequest, resizeObserver, modelEpoch = 0, profile = null, state;
  let THREE, STLLoader, joints = new Map(), markerObjects = new Map();
  let latestFrame = null, dirty = true, baseInertial;
  const setStatus = text => { if (!disposed) status.textContent = text; };
  const release = object => {
    object?.traverse(child => {
      child.geometry?.dispose();
      for (const material of Array.isArray(child.material) ? child.material : [child.material]) {
        if (material?.map) material.map.dispose();
        material?.dispose();
      }
      child.element?.remove();
    });
    object?.removeFromParent();
  };
  function clearModel() {
    modelEpoch++; modelAbort?.abort(); modelAbort = null;
    release(robot); robot = null; joints.clear();
    for (const marker of markerObjects.values()) release(marker);
    markerObjects.clear(); latestFrame = null; profile = null; dirty = true;
  }
  function dispose() {
    if (disposed) return;
    disposed = true; clearModel(); clearTimeout(reconnect);
    if (ws?.readyState === 1) ws.send(JSON.stringify({type:'simulation_viewer',data:{enabled:false,render_mode:'web'}}));
    ws?.close(); cancelAnimationFrame(frameRequest);
    resizeObserver?.disconnect(); controls?.dispose(); release(scene);
    renderer?.dispose(); renderer?.forceContextLoss(); container.replaceChildren();
    signal?.removeEventListener('abort', dispose);
  }
  function connect() {
    if (disposed) return;
    ws = new WebSocket(`${location.protocol === 'https:' ? 'wss:' : 'ws:'}//${location.host}/ws`);
    ws.onopen = () => {
      if (disposed) { ws.close(); return; }
      ws.send(JSON.stringify({type:'simulation_viewer',data:{enabled:true,render_mode:'web'}}));
    };
    ws.onmessage = event => {
      let message;
      try { message = JSON.parse(event.data); } catch { return; }
      if (message.type !== 'simulation_frame' || state?.state !== 'running') return;
      if (message.data?.profile_id !== profile) return;
      latestFrame = message.data;
    };
    ws.onclose = () => { if (!disposed) reconnect = setTimeout(connect, 1500); };
    ws.onerror = () => setStatus('三维连接中断，正在重连…');
  }
  const numbers = (text, fallback) => text ? text.trim().split(/\s+/).map(Number) : fallback;
  function origin(node, target) {
    target.position.fromArray(numbers(node?.getAttribute('xyz'), [0,0,0]));
    const rpy = numbers(node?.getAttribute('rpy'), [0,0,0]);
    target.quaternion.setFromEuler(new THREE.Euler(...rpy, 'ZYX'));
  }
  function pose(target, value) {
    if (!Array.isArray(value) || value.length !== 2 || !value[0] || !value[1]) return false;
    target.position.fromArray(value[0]); target.quaternion.fromArray(value[1]); return true;
  }
  async function loadModel(expected) {
    const epoch = ++modelEpoch;
    modelAbort?.abort(); modelAbort = new AbortController();
    const controller = modelAbort;
    const current = () => !disposed && epoch === modelEpoch;
    let group;
    try {
      setStatus('正在加载三维模型…');
      const response = await fetch('/api/simulation/model', {signal:controller.signal});
      if (!response.ok) throw new Error(await response.text());
      const metadata = await response.json();
      if (!current()) return;
      if (metadata.profile_id !== expected) { profile=null;setStatus('等待当前仿真模型…');return; }
      const result = await fetch(metadata.urdf_url, {signal:controller.signal});
      if (!result.ok) throw new Error('无法读取机器人模型');
      const xml = new DOMParser().parseFromString(await result.text(), 'text/xml');
      if (!current()) return;
      if (xml.querySelector('parsererror')) throw new Error('机器人模型格式错误');
      group = new THREE.Group();
      const links = new Map(), newJoints = new Map(), children = new Set();
      const root = xml.documentElement;
      for (const node of root.querySelectorAll(':scope > link')) { const link=new THREE.Group(); links.set(node.getAttribute('name'),link); group.add(link); }
      const pending = [];
      for (const node of root.querySelectorAll(':scope > link')) {
        const link = links.get(node.getAttribute('name'));
        for (const visual of node.querySelectorAll(':scope > visual')) {
          const holder = new THREE.Group(); origin(visual.querySelector(':scope > origin'), holder); link.add(holder);
          const rgba = numbers(visual.querySelector('material > color')?.getAttribute('rgba'), [0.72,0.75,0.8,1]);
          const material = new THREE.MeshStandardMaterial({color:new THREE.Color(...rgba),roughness:0.75,metalness:0.15,opacity:rgba[3],transparent:rgba[3]<1});
          const mesh = visual.querySelector('geometry > mesh');
          if (mesh) {
            const filename = mesh.getAttribute('filename');
            if (!/\.stl$/i.test(filename)) { material.dispose(); throw new Error(`暂不支持此模型格式：${filename}`); }
            let url;
            if (filename.startsWith('package://')) {
              const relative = filename.slice(10).split('/').slice(1).join('/');
              url = new URL(relative, new URL(metadata.asset_base_url, location.href));
            } else url = new URL(filename, new URL(metadata.urdf_url, location.href));
            pending.push(fetch(url, {signal:controller.signal}).then(async response => {
              if (!response.ok) throw new Error('无法读取模型网格');
              const data = await response.arrayBuffer();
              if (!current()) { material.dispose(); return; }
              const geometry = new STLLoader().parse(data);
              const object = new THREE.Mesh(geometry, material);
              object.scale.fromArray(numbers(mesh.getAttribute('scale'), [1,1,1])); holder.add(object);
            }).catch(error => { material.dispose(); throw error; }));
          } else {
            let geometry;
            const box = visual.querySelector('geometry > box'), sphere = visual.querySelector('geometry > sphere'), cylinder = visual.querySelector('geometry > cylinder');
            if (box) geometry = new THREE.BoxGeometry(...numbers(box.getAttribute('size'), [1,1,1]));
            else if (sphere) geometry = new THREE.SphereGeometry(Number(sphere.getAttribute('radius')), 20, 12);
            else if (cylinder) { geometry = new THREE.CylinderGeometry(Number(cylinder.getAttribute('radius')),Number(cylinder.getAttribute('radius')),Number(cylinder.getAttribute('length')),24); geometry.rotateX(Math.PI/2); }
            if (geometry) holder.add(new THREE.Mesh(geometry, material)); else material.dispose();
          }
        }
      }
      for (const node of root.querySelectorAll(':scope > joint')) {
        const child = links.get(node.querySelector('child')?.getAttribute('link'));
        const parent = links.get(node.querySelector('parent')?.getAttribute('link'));
        if (!parent || !child) throw new Error('模型关节缺少对应连杆');
        children.add(child); const fixed = new THREE.Group(), motion = new THREE.Group();
        origin(node.querySelector(':scope > origin'), fixed); parent.add(fixed); fixed.add(motion); motion.add(child);
        const mimic = node.querySelector('mimic');
        newJoints.set(node.getAttribute('name'), {object:motion,type:node.getAttribute('type'),axis:new THREE.Vector3(...numbers(node.querySelector('axis')?.getAttribute('xyz'),[1,0,0])).normalize(),mimic:mimic&&{joint:mimic.getAttribute('joint'),multiplier:Number(mimic.getAttribute('multiplier')??1),offset:Number(mimic.getAttribute('offset')??0)}});
      }
      for (const link of links.values()) if (!children.has(link)) group.add(link);
      // Attach before asynchronous mesh loading so cancellation disposes every allocation.
      if (!current()) { release(group); return; }
      robot = group; joints = newJoints; profile = expected;
      baseInertial = new THREE.Matrix4();
      const inertial = new THREE.Object3D();
      pose(inertial, metadata.base_inertial_pose || [[0,0,0],[0,0,0,1]]); inertial.updateMatrix(); baseInertial.copy(inertial.matrix).invert();
      scene.add(group);
      await Promise.all(pending);
      if (!current()) return;
      setStatus('三维视图已开启 · 拖动旋转，滚轮缩放'); dirty = true;
    } catch (error) {
      if (current() && error.name !== 'AbortError') { clearModel(); setStatus(`三维视图加载失败：${error.message}`); }
      else if (group && group !== robot) release(group);
    }
  }
  function marker(key) {
    if (markerObjects.has(key)) return markerObjects.get(key);
    const group = new THREE.Group(); group.add(new THREE.AxesHelper(key.includes('target') ? 0.18 : 0.12));
    const labels = {base:'底盘',ee_actual_left:'左臂末端',ee_actual_right:'右臂末端',ee_target_left:'左臂目标',ee_target_right:'右臂目标',left_actual:'左臂末端',right_actual:'右臂末端',left_target:'左臂目标',right_target:'右臂目标',head_actual:'头部相机',head_target:'相机目标',camera_actual:'头部相机',camera_target:'相机目标'};
    const canvas = document.createElement('canvas'); canvas.width=512; canvas.height=64;
    const target=key.includes('target');
    const ctx=canvas.getContext('2d'); ctx.fillStyle=target?'rgba(91,48,10,.88)':'rgba(9,51,75,.88)'; ctx.fillRect(0,0,512,64); ctx.fillStyle=target?'#ffd088':'#a5e7ff';ctx.font='30px sans-serif';ctx.textAlign='center';ctx.fillText(labels[key]||key,256,43);
    const texture = new THREE.CanvasTexture(canvas); const label = new THREE.Sprite(new THREE.SpriteMaterial({map:texture,depthTest:false})); label.scale.set(0.36,0.045,1);label.position.z=target?0.12:0.06;group.add(label);
    scene.add(group);markerObjects.set(key,group);return group;
  }
  function applyFrame(frame) {
    if (!robot) return;
    const values=frame.positions||{};
    for (const [name, joint] of joints) {
      const value=joint.mimic ? Number(values[joint.mimic.joint]||0)*joint.mimic.multiplier+joint.mimic.offset : Number(values[name]||0);
      if (!Number.isFinite(value)) continue;
      if (joint.type==='prismatic') joint.object.position.copy(joint.axis).multiplyScalar(value);
      else if (joint.type==='revolute'||joint.type==='continuous') joint.object.quaternion.setFromAxisAngle(joint.axis,value);
    }
    if (pose(robot,frame.base_pose)) { robot.updateMatrix();robot.matrix.multiply(baseInertial);robot.matrix.decompose(robot.position,robot.quaternion,robot.scale); }
    const active=new Set();
    for (const [key,value] of Object.entries(frame.markers||{})) { const object=marker(key);object.visible=pose(object,value);active.add(key); }
    for (const [key,object] of markerObjects) if (!active.has(key)) object.visible=false;
    dirty=true;
  }
  function updateState(value) {
    state=value;
    if (!scene || disposed) return;
    if (value.runtime_mode==='teleop' || value.state!=='running') {
      if (robot || profile) clearModel();
      setStatus(value.runtime_mode==='teleop'?'真机遥操模式不提供仿真视图':'等待仿真启动');return;
    }
    const expected=value.profile_id||value.active_profile;
    if (expected && expected!==profile) { clearModel();profile=expected;loadModel(expected); }
  }
  signal?.addEventListener('abort', dispose, {once:true});
  if (signal?.aborted) { dispose(); return {dispose,updateState}; }
  (async () => {
    try {
      const modules=await loadGraphics(); if (disposed) return;
      THREE=modules[0];STLLoader=modules[2].STLLoader;
      scene=new THREE.Scene();scene.background=new THREE.Color(0xeaf0f6);
      camera=new THREE.PerspectiveCamera(45,1,0.01,100);camera.up.set(0,0,1);camera.position.set(3,-3,2.3);
      renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio||1,2));container.append(renderer.domElement);
      controls=new modules[1].OrbitControls(camera,renderer.domElement);controls.target.set(0,0,0.9);controls.addEventListener('change',()=>{dirty=true;});controls.update();
      scene.add(new THREE.HemisphereLight(0xffffff,0x667788,2.5));const light=new THREE.DirectionalLight(0xffffff,2);light.position.set(3,-4,6);scene.add(light);
      const grid=new THREE.GridHelper(10,50,0x9baebe,0xc6d1db);grid.rotation.x=Math.PI/2;scene.add(grid);
      const resize=()=>{const width=Math.max(1,container.clientWidth),height=Math.max(1,container.clientHeight);renderer.setSize(width,height);camera.aspect=width/height;camera.updateProjectionMatrix();dirty=true;};
      resizeObserver=new ResizeObserver(resize);resizeObserver.observe(container);resize();
      const animate=()=>{if(disposed)return;frameRequest=requestAnimationFrame(animate);if(latestFrame){const frame=latestFrame;latestFrame=null;applyFrame(frame);}if(dirty&&container.clientWidth){renderer.render(scene,camera);dirty=false;}};
      animate();connect();if(state)updateState(state);else setStatus('等待仿真状态…');
    } catch(error) {if(!disposed){dispose();status.textContent=`无法打开三维视图：${error.message}`;}}
  })();
  return {dispose,updateState};
}
