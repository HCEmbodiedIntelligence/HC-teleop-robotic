const test=require('node:test');
const assert=require('node:assert/strict');
const fs=require('node:fs');
const vm=require('node:vm');
const path=require('node:path');
function setup(){
  const fields={
    '#topicRecordingDirectory':{value:'runtime/datasets'},
    '#topicRecordingFormat':{value:'both'},
    '#topicRecordingLerobotFps':{value:'15'},
    '#topicRecordingLerobotTask':{value:'搬运箱子'},
    '#recordingExportStatus':{},
  };
  const ctx=vm.createContext({document:{querySelector:s=>fields[s] || null,querySelectorAll:()=>[]}});
  vm.runInContext(fs.readFileSync(path.join(__dirname,'../middleware/core/static/app.js'),'utf8').replace(/init\(\);\s*$/,''),ctx);
  vm.runInContext('config={ros:{subscriptions:[]}}',ctx);
  return {ctx,fields};
}
test('recording settings save together without dropping subscription configuration',()=>{
  const {ctx,fields}=setup();
  const config=JSON.parse(vm.runInContext('JSON.stringify(collectConfig())',ctx));
  assert.deepEqual(config.ros.recording,{directory:'runtime/datasets',workflow:'hc',format:'both',derive_action_from_next_state:false,lerobot_fps:15,lerobot_task:'搬运箱子'});
  assert.deepEqual(config.ros.subscriptions,[]);
  fields['#topicRecordingLerobotFps'].value='0';
  assert.throws(()=>vm.runInContext('collectConfig()',ctx),/帧率/);
});
test('custom format selects the raw workflow and leaving it disables test actions',()=>{
  const {ctx,fields}=setup();
  fields['#captureConfigFile']={value:'/runtime/openarmx.yaml'};
  fields['#deriveActionFromNextState']={checked:true};
  fields['#topicRecordingFormat'].value='platform_raw';
  vm.runInContext('updateRecordingFormatFields()',ctx);
  let config=JSON.parse(vm.runInContext('JSON.stringify(collectConfig())',ctx));
  assert.equal(config.ros.recording.workflow,'platform_raw');
  assert.equal(config.ros.recording.format,'mcap');
  assert.equal(config.ros.recording.derive_action_from_next_state,true);
  assert.equal(config.ros.recording.capture_config_file,'/runtime/openarmx.yaml');
  assert.equal(fields['#topicRecordingLerobotFps'].disabled,true);
  for(const mode of ['mcap','lerobot','both']) {
    fields['#topicRecordingFormat'].value=mode;
    vm.runInContext('updateRecordingFormatFields()',ctx);
    config=JSON.parse(vm.runInContext('JSON.stringify(collectConfig())',ctx));
    assert.equal(config.ros.recording.workflow,'hc');
    assert.equal(config.ros.recording.format,mode);
    assert.equal(config.ros.recording.derive_action_from_next_state,false);
    assert.equal(fields['#deriveActionFromNextState'].disabled,true);
  }
});
test('MCAP disables dataset-only fields and switching back restores editing',()=>{
  const {ctx,fields}=setup();
  fields['#topicRecordingFormat'].value='mcap';
  vm.runInContext('updateRecordingFormatFields()',ctx);
  assert.equal(fields['#topicRecordingLerobotFps'].disabled,true);
  fields['#topicRecordingFormat'].value='lerobot';
  vm.runInContext('updateRecordingFormatFields()',ctx);
  assert.equal(fields['#topicRecordingLerobotTask'].disabled,false);
});
test('export status remains compatible with old servers and exposes failure and dataset location',()=>{
  const {ctx,fields}=setup();
  vm.runInContext('renderRecordingExport({})',ctx);
  assert.equal(fields['#recordingExportStatus'].hidden,true);
  vm.runInContext("renderRecordingExport({export_state:'completed',lerobot_path:'/datasets/episode'})",ctx);
  assert.match(fields['#recordingExportStatus'].textContent,/已生成.*\/datasets\/episode/);
  vm.runInContext("renderRecordingExport({export_state:'failed',export_error:'missing observations'})",ctx);
  assert.match(fields['#recordingExportStatus'].textContent,/导出失败.*missing observations/);
});
test('recording export reports actual stage progress and failure detail',()=>{
  const {ctx,fields}=setup();
  vm.runInContext("renderRecordingExport({export_state:'exporting',export_progress:{stage:'解码图像',percent:24,current:120,total:500,unit:'条消息'}})",ctx);
  assert.match(fields['#recordingExportStatus'].textContent,/解码图像.*24%.*120 \/ 500 条消息/);
  vm.runInContext("renderRecordingExport({export_state:'error',export_progress:{state:'error',error:'编码失败'}})",ctx);
  assert.match(fields['#recordingExportStatus'].textContent,/导出失败：编码失败/);
});
test('successful retry progress replaces cached failure state',()=>{
  const {ctx,fields}=setup();
  vm.runInContext("renderRecordingExport({export_state:'error',export_error:'old failure',export_progress:{state:'completed',stage:'转换完成',percent:100,error:''}})",ctx);
  assert.match(fields['#recordingExportStatus'].textContent,/已生成.*100%/);
  assert.doesNotMatch(fields['#recordingExportStatus'].textContent,/old failure/);
  assert.equal(fields['#recordingExportStatus'].className,'muted');
});
