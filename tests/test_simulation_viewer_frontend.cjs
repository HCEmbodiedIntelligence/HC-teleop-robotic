const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname,'../middleware/core/static/simulation_viewer.js'),'utf8');
const app = fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8');
function fixture() {
  const elements = new Map();
  const get = key => {
    if (!elements.has(key)) elements.set(key,{checked:false,hidden:true,textContent:'',disabled:false});
    return elements.get(key);
  };
  const imports=[], instances=[], requests=[];
  const context=vm.createContext({
    document:{querySelector:get,querySelectorAll:()=>[]},
    window:{addEventListener:()=>{}},
    AbortController, setTimeout,clearTimeout,
    importView:async()=>{imports.push(1);return {mountSimulationViewer:options=>{
      const instance={options,states:[],disposed:false,dispose(){this.disposed=true;},updateState(value){this.states.push(value);}};
      options.signal.addEventListener('abort',()=>instance.dispose());instances.push(instance);return instance;
    }};},
    fetch:async url=>{requests.push(url);return {ok:true,json:async()=>({state:'stopped',active_profile:'x1',available:true,logs:[]})};},
  });
  vm.runInContext(app.replace("import('/static/simulation_viewer.js?v=1')",'importView()').replace(/init\(\);\s*$/,''),context);
  vm.runInContext('bindSimulationControls()',context);
  return {get,context,imports,instances,requests};
}

test('default polling does not import graphics, mount a view or open a frame subscription',async()=>{
  const f=fixture();await vm.runInContext('loadSimulation()',f.context);
  assert.equal(f.get('#simulationViewEnabled').checked,false);
  assert.deepEqual(f.imports,[]);assert.deepEqual(f.instances,[]);
  assert.deepEqual(f.requests,['/api/simulation']);
});
test('independent view toggle opens and closes without saving options or restarting simulation',async()=>{
  const f=fixture();await vm.runInContext('loadSimulation()',f.context);
  f.get('#simulationViewEnabled').checked=true;await f.get('#simulationViewEnabled').onchange();
  assert.equal(f.imports.length,1);assert.equal(f.get('#simulationView').hidden,false);
  assert.equal(f.instances[0].states[0].state,'stopped');
  f.get('#simulationViewEnabled').checked=false;await f.get('#simulationViewEnabled').onchange();
  assert.equal(f.instances[0].disposed,true);assert.equal(f.get('#simulationView').hidden,true);
  assert.deepEqual(f.requests,['/api/simulation']);
});
test('closing while the module loads cannot mount a stale view',async()=>{
  const f=fixture();let resolve;
  f.context.importView=()=>new Promise(done=>resolve=done);
  f.get('#simulationViewEnabled').checked=true;const pending=f.get('#simulationViewEnabled').onchange();
  f.get('#simulationViewEnabled').checked=false;await f.get('#simulationViewEnabled').onchange();
  let mounted=false;resolve({mountSimulationViewer:()=>{mounted=true;}});await pending;
  assert.equal(mounted,false);assert.equal(f.get('#simulationView').hidden,true);
});

test('view abort during graphics import creates no canvas or websocket',async()=>{
  const context=vm.createContext({setTimeout,clearTimeout,cancelAnimationFrame:()=>{},AbortController});
  vm.runInContext(source.replace('export function mountSimulationViewer','function mountSimulationViewer'),context);
  const controller=new AbortController();let resolve;let clears=0;
  context.options={signal:controller.signal,container:{replaceChildren(){clears++;}},status:{textContent:''},loadGraphics:()=>new Promise(done=>resolve=done)};
  const view=vm.runInContext('mountSimulationViewer(options)',context);
  controller.abort();resolve([]);await Promise.resolve();
  view.dispose();assert.equal(clears,1);
});

test('active view subscribes on its own websocket and releases rendering and transport on close',async()=>{
  const sockets=[],events=[];let observeCallback;
  class Object3D {constructor(){this.children=[];this.position={set(){}};this.rotation={};this.up={set(){}};}add(child){this.children.push(child);}traverse(callback){callback(this);for(const child of this.children)child.traverse(callback);}removeFromParent(){} }
  class Renderer {constructor(){this.domElement={};}setPixelRatio(){}setSize(){}render(){events.push('render');}dispose(){events.push('renderer.dispose');}forceContextLoss(){events.push('context.release');}}
  class Controls {constructor(){this.target={set(){}};}addEventListener(){}update(){}dispose(){events.push('controls.dispose');}}
  class Camera extends Object3D {updateProjectionMatrix(){}}
  const THREE={Scene:Object3D,Color:class {},PerspectiveCamera:Camera,WebGLRenderer:Renderer,HemisphereLight:Object3D,DirectionalLight:Object3D,GridHelper:Object3D};
  const context=vm.createContext({
    setTimeout,clearTimeout,requestAnimationFrame:()=>33,cancelAnimationFrame:id=>events.push(`cancel.${id}`),
    devicePixelRatio:1,location:{protocol:'http:',host:'example:7876'},
    ResizeObserver:class {constructor(callback){observeCallback=callback;}observe(){}disconnect(){events.push('resize.disconnect');}},
    WebSocket:class {constructor(url){this.url=url;this.sent=[];this.readyState=1;sockets.push(this);}send(text){this.sent.push(JSON.parse(text));}close(){this.readyState=3;events.push('ws.close');}},
  });
  vm.runInContext(source.replace('export function mountSimulationViewer','function mountSimulationViewer'),context);
  const container={clientWidth:800,clientHeight:400,append(){events.push('canvas.append');},replaceChildren(){events.push('canvas.clear');}};
  context.options={container,status:{textContent:''},loadGraphics:async()=>[THREE,{OrbitControls:Controls},{STLLoader:class {}}]};
  const view=vm.runInContext('mountSimulationViewer(options)',context);await new Promise(resolve=>setImmediate(resolve));
  assert.equal(sockets.length,1);sockets[0].onopen();
  assert.deepEqual(sockets[0].sent,[{type:'simulation_viewer',data:{enabled:true,render_mode:'web'}}]);
  view.updateState({state:'stopped'});view.dispose();
  assert.equal(sockets[0].sent[1].data.enabled,false);
  for(const event of ['renderer.dispose','context.release','controls.dispose','resize.disconnect','ws.close','canvas.clear','cancel.33'])assert.ok(events.includes(event),event);
  assert.equal(events.filter(event=>event==='render').length,1);
});

test('URDF hierarchy, joint rotation, COM correction and markers use matching world coordinates',async()=>{
  const moduleText=fs.readFileSync(path.join(__dirname,'../middleware/core/static/vendor/three/three.module.js'),'utf8');
  const THREE=await import(`data:text/javascript;base64,${Buffer.from(moduleText).toString('base64')}`);
  let renderedScene,animate,socket;const meshes=[];
  class Scene extends THREE.Scene {constructor(){super();renderedScene=this;}}
  class Renderer {constructor(){this.domElement={};}setPixelRatio(){}setSize(){}render(){}dispose(){}forceContextLoss(){}}
  class Controls {constructor(){this.target=new THREE.Vector3();}addEventListener(){}update(){}dispose(){}}
  const node=(attributes={},queries={})=>({getAttribute:key=>attributes[key]??null,querySelector:key=>queries[key]??null,querySelectorAll:key=>queries[key]??[]});
  const visual=node({}, {
    ':scope > origin':node({xyz:'0.1 0 0',rpy:'0 0 0'}),
    'geometry > box':node({size:'0.1 0.2 0.3'}),
  });
  const links=[node({name:'root'},{':scope > visual':[]}),node({name:'tip'},{':scope > visual':[visual]})];
  const joint=node({name:'elbow',type:'revolute'},{
    child:node({link:'tip'}),parent:node({link:'root'}),
    ':scope > origin':node({xyz:'0.5 0 0',rpy:'0.1 0.2 0.3'}),axis:node({xyz:'0 0 1'}),
  });
  const root=node({}, {':scope > link':links,':scope > joint':[joint]});
  const context=vm.createContext({
    setTimeout,clearTimeout,requestAnimationFrame:callback=>{animate=callback;return 7;},cancelAnimationFrame:()=>{},devicePixelRatio:1,
    location:{protocol:'http:',host:'example',href:'http://example/'},URL,AbortController,
    DOMParser:class {parseFromString(){return {documentElement:root,querySelector:()=>null};}},
    ResizeObserver:class {observe(){}disconnect(){}},
    document:{createElement:()=>({getContext:()=>({fillRect(){},fillText(){}})})},
    WebSocket:class {constructor(){socket=this;this.readyState=1;}send(){}close(){}},
    fetch:async url=>url==='/api/simulation/model'?{ok:true,json:async()=>({profile_id:'x1',urdf_url:'/assets/x1/urdf/model.urdf',asset_base_url:'/assets/x1/',base_inertial_pose:[[0.2,0,0],[0,0,0,1]]})}:{ok:true,text:async()=>'<robot/>'},
  });
  vm.runInContext(source.replace('export function mountSimulationViewer','function mountSimulationViewer'),context);
  const module={...THREE,Scene,WebGLRenderer:Renderer};
  context.options={container:{clientWidth:800,clientHeight:400,append(){},replaceChildren(){}},status:{textContent:''},loadGraphics:async()=>[module,{OrbitControls:Controls},{STLLoader:class {}}]};
  const view=vm.runInContext('mountSimulationViewer(options)',context);
  view.updateState({state:'running',profile_id:'x1'});
  await new Promise(resolve=>setImmediate(resolve));
  const robot=renderedScene.children.find(child=>child.type==='Group');assert.ok(robot);
  const yaw=Math.PI/2,quat=[0,0,Math.sin(yaw/2),Math.cos(yaw/2)];
  socket.onmessage({data:JSON.stringify({type:'simulation_frame',data:{profile_id:'x1',positions:{elbow:0.4},base_pose:[[1,2,3],quat],markers:{ee_target_right:[[4,5,6],[0,0,0,1]]}}})});animate();
  assert.ok(robot.position.distanceTo(new THREE.Vector3(1,1.8,3))<1e-9);
  renderedScene.updateMatrixWorld(true);
  const tip=robot.children[0].children[0].children[0].children[0];
  const expected=new THREE.Matrix4().compose(new THREE.Vector3(1,1.8,3),new THREE.Quaternion(...quat),new THREE.Vector3(1,1,1))
    .multiply(new THREE.Matrix4().compose(new THREE.Vector3(0.5,0,0),new THREE.Quaternion().setFromEuler(new THREE.Euler(0.1,0.2,0.3,'ZYX')),new THREE.Vector3(1,1,1)))
    .multiply(new THREE.Matrix4().makeRotationZ(0.4));
  for(let i=0;i<16;i++)assert.ok(Math.abs(tip.matrixWorld.elements[i]-expected.elements[i])<1e-9,`matrix index ${i}`);
  const marker=renderedScene.children.find(child=>child.type==='Group'&&child!==robot);
  assert.deepEqual(marker.position.toArray(),[4,5,6]);
  view.updateState({state:'stopped',profile_id:'x1'});
  assert.equal(renderedScene.children.some(child=>child.type==='Group'),false);
  view.dispose();
});

test('the bundled X1 URDF resolves every visual mesh and assembles the complete link tree',async()=>{
  const {execFileSync}=require('node:child_process');
  const profile=path.join(__dirname,'../adapters/robots/x1');
  const xmlPath=path.join(profile,'urdf/hc_tj_robot.urdf');
  const tree=JSON.parse(execFileSync('python3',['-c',
    'import json,sys,xml.etree.ElementTree as E; f=lambda n:dict(tag=n.tag,attrs=n.attrib,children=[f(c) for c in n]); print(json.dumps(f(E.parse(sys.argv[1]).getroot())))',xmlPath],{encoding:'utf8'}));
  function dom(value) {
    const children=value.children.map(dom);
    function descend(node,tag){return node.children.flatMap(child=>[...(child.tag===tag?[child]:[]),...descend(child,tag)]);}
    return {tag:value.tag,children,getAttribute:key=>value.attrs[key]??null,
      querySelector(selector){return this.querySelectorAll(selector)[0]??null;},
      querySelectorAll(selector){
        const scoped=selector.startsWith(':scope >');
        const parts=selector.replace(/^:scope\s*>\s*/,'').split(/\s*>\s*/);
        let result=scoped?children.filter(child=>child.tag===parts[0]):descend(this,parts[0]);
        for(const part of parts.slice(1))result=result.flatMap(node=>node.children.filter(child=>child.tag===part));
        return result;
      }};
  }
  const root=dom(tree);
  const threeText=fs.readFileSync(path.join(__dirname,'../middleware/core/static/vendor/three/three.module.js'),'utf8');
  const threeUrl=`data:text/javascript;base64,${Buffer.from(threeText).toString('base64')}`;
  const THREE=await import(threeUrl);
  const stlText=fs.readFileSync(path.join(__dirname,'../middleware/core/static/vendor/three/STLLoader.js'),'utf8').replace("'./three.module.js'",JSON.stringify(threeUrl));
  const {STLLoader}=await import(`data:text/javascript;base64,${Buffer.from(stlText).toString('base64')}`);
  let scene,meshCount=0,firstGeometry;
  class Scene extends THREE.Scene {constructor(){super();scene=this;}}
  class Renderer {constructor(){this.domElement={};}setPixelRatio(){}setSize(){}render(){}dispose(){}forceContextLoss(){}}
  class Controls {constructor(){this.target=new THREE.Vector3();}addEventListener(){}update(){}dispose(){}}
  const status={textContent:''};
  const requested=[];
  const context=vm.createContext({
    setTimeout,clearTimeout,requestAnimationFrame:()=>1,cancelAnimationFrame:()=>{},devicePixelRatio:1,
    location:{protocol:'http:',host:'example',href:'http://example/'},URL,AbortController,
    DOMParser:class {parseFromString(){return {documentElement:root,querySelector:()=>null};}},
    ResizeObserver:class {observe(){}disconnect(){}},
    WebSocket:class {constructor(){this.readyState=1;}send(){}close(){}},
    fetch:async url=>{
      if(url==='/api/simulation/model')return {ok:true,json:async()=>({profile_id:'x1',urdf_url:'/assets/x1/urdf/hc_tj_robot.urdf',asset_base_url:'/assets/x1/',base_inertial_pose:[[0,0,0],[0,0,0,1]]})};
      if(typeof url==='string')return {ok:true,text:async()=>'<robot/>'};
      const file=path.join(profile,decodeURIComponent(url.pathname.replace('/assets/x1/','')));
      assert.ok(file.startsWith(profile+path.sep));assert.ok(fs.existsSync(file),file);requested.push(file);
      const buffer=fs.readFileSync(file);return {ok:true,arrayBuffer:async()=>buffer.buffer.slice(buffer.byteOffset,buffer.byteOffset+buffer.byteLength)};
    },
  });
  vm.runInContext(source.replace('export function mountSimulationViewer','function mountSimulationViewer'),context);
  class Loader {parse(data){meshCount++;if(!firstGeometry){firstGeometry=new STLLoader().parse(data);return firstGeometry;}return new THREE.BoxGeometry(0.01,0.01,0.01);}}
  context.options={container:{clientWidth:800,clientHeight:400,append(){},replaceChildren(){}},status,loadGraphics:async()=>[{...THREE,Scene,WebGLRenderer:Renderer},{OrbitControls:Controls},{STLLoader:Loader}]};
  const view=vm.runInContext('mountSimulationViewer(options)',context);view.updateState({state:'running',profile_id:'x1'});
  await new Promise(resolve=>setImmediate(resolve));
  assert.match(status.textContent,/三维视图已开启/);
  const expected=tree.children.filter(node=>node.tag==='link').reduce((count,node)=>count+node.children.filter(child=>child.tag==='visual').length,0);
  assert.equal(meshCount,expected);assert.equal(requested.length,expected);assert.ok(expected>20);
  assert.ok(firstGeometry.attributes.position.count>100);
  const robot=scene.children.find(child=>child.type==='Group');assert.ok(robot);assert.equal(robot.children.length,1);
  let visualCount=0;robot.traverse(child=>{if(child.isMesh)visualCount++;});assert.equal(visualCount,expected);
  view.dispose();
});
