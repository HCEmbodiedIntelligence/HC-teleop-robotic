"""Application service for SDK/envelope sources, sessions and background exports."""
from dataclasses import asdict
import importlib.metadata
import json
import os
from pathlib import Path
import re
import threading
import time
import uuid
import subprocess
import signal
import sys
import tempfile

from .adapters import preflight
from .catalog import read_json, session_path, summary
from .clocks import Clocks
from .config import DEFAULT, validate, config_hash
from .session import Session
from .storage import atomic_json


class CaptureHub:
    def __init__(self,state_root,ros_config):
        self.state_root=Path(state_root)
        self.config_path=self.state_root/'capture-config.json'
        self.cfg=validate(read_json(self.config_path,DEFAULT))
        self.directories_path=self.state_root/'capture-directories.json'
        self.directories=list(dict.fromkeys([str(self.directory), *read_json(self.directories_path,[])]))
        self.ros_config=ros_config
        self.clocks=Clocks()
        self.clocks.add({'id':'host_receive_v1','clock_id':'host_monotonic','epoch':0,
            'device_origin_ns':0,'common_origin_ns':0,'uncertainty_ns':None,'evidence':''})
        self.reports={}
        self.session=None
        self.ros_bridge=None
        self.adapters={}
        self.adapter_threads=[]
        self.adapter_stop=threading.Event()
        self.job={'state':'idle'}
        self.job_thread=None
        self.on_export_complete=None
        self.lock=threading.RLock()
        self.draining=[]
        self.workflow_path=self.state_root/'capture-workflow.json'
        self.workflow=read_json(self.workflow_path,{'vr_target':'aligned','package_after_stop':False})
        self.jobs_path=self.state_root/'capture-jobs.json'
        self.jobs=read_json(self.jobs_path,[])
        for job in self.jobs:
            if job['state'] not in {'completed','failed'}:job['state']='queued'
        self.jobs_stop=threading.Event()
        self.process=None
        self.job_paused=False
        self.job_resume_after=0.
        self.priority_thread=threading.Thread(target=self._prioritize_capture,name='capture-job-priority',daemon=True)
        self.directory.mkdir(parents=True,exist_ok=True)
        # A prior process's active files are not committed just because they exist.
        for path in (p for root in self.directories for p in Path(root).glob('*/manifest.json')):
            manifest=read_json(path,{})
            if manifest.get('state') in {'recording','warming_up','draining'}:
                manifest.update(state='interrupted',recovery='仅已提交清单内的文件可直接使用；活动文件需检查恢复')
                atomic_json(path,manifest)
        self.job_thread=threading.Thread(target=self._run_jobs,name='capture-jobs',daemon=True)
        self.priority_thread.start()
        self.job_thread.start()

    @property
    def directory(self):
        return self.storage_path(self.cfg['storage']['directory'])

    def storage_path(self,value):
        path=Path(value).expanduser()
        return (path if path.is_absolute() else self.state_root/path).resolve()

    @property
    def export_directory(self):
        return self.storage_path(self.cfg['storage'].get('export_directory','datasets'))

    def session_path(self,ident):
        # Only roots saved by this manager are eligible; request paths are never roots.
        found=[]
        for root in dict.fromkeys([str(self.directory),*self.directories]):
            try:found.append(session_path(root,ident))
            except ValueError:continue
        if len(found)!=1:
            raise ValueError('采集目录不存在' if not found else '多处录制目录存在同名记录，请先确认文件归属')
        return found[0]

    def busy(self):
        return bool(self.input_sessions()) or any(j['state'] not in {'completed','failed'} for j in self.jobs)

    def input_sessions(self):
        """The next recording has fresh caches; the old one only drains to its cutoff."""
        with self.lock:
            self.draining=[s for s in self.draining if not s.closed.is_set()]
            sessions=list(self.draining)
            if self.session and not self.session.closed.is_set() and self.session not in sessions:
                sessions.append(self.session)
            return tuple(sessions)

    def recording(self):
        return bool(self.session and not self.session.closed.is_set() and not self.session.stop_requested
                    and not self.session.stop_event.is_set())

    def save_workflow(self,value):
        if self.input_sessions():
            raise ValueError('请停止录制并等待本地收尾，再修改 VR 录制模式')
        if (not isinstance(value,dict) or set(value)!={'vr_target','package_after_stop'}
                or value['vr_target'] not in {'raw','aligned'}
                or type(value['package_after_stop']) is not bool):
            raise ValueError('无效的录制工作流配置')
        atomic_json(self.workflow_path,value)
        self.workflow=dict(value)
        return self.state()

    def state(self):
        return {'config':self.cfg,'etag':config_hash(self.cfg),'directory':str(self.directory),
            'export_directory':str(self.export_directory),
            'status':self.session.status() if self.session else {'state':'idle','active':False},
            'preflight':preflight(self.cfg,self.reports,self.clocks),'job':self.job,
            'workflow':self.workflow,'draining':[s.status() for s in self.draining if not s.closed.is_set()],
            'jobs':list(self.jobs),
            'available_adapters':[],'camera_architecture':'external_ros_node',
            'ros_input_error':self.ros_bridge.error if self.ros_bridge else None}

    def save_config(self,value,etag):
        if self.busy():
            raise ValueError('正在采集或处理数据，请结束后修改采集策略')
        if etag!=config_hash(self.cfg):
            raise ValueError('采集配置已变化，请重新读取')
        cfg=validate(value)
        directory=self.storage_path(cfg['storage']['directory'])
        export_directory=self.storage_path(cfg['storage'].get('export_directory','datasets'))
        for path in {directory,export_directory}:
            try:
                path.mkdir(parents=True,exist_ok=True)
                with tempfile.TemporaryFile(dir=path) as probe:
                    probe.write(b'capture-directory-check')
                    probe.flush()
            except OSError as error:
                raise ValueError(f'保存目录不可写：{path}；请检查路径、权限和磁盘空间') from error
        directories=list(dict.fromkeys([str(directory),str(self.directory),*self.directories]))
        atomic_json(self.directories_path,directories)
        atomic_json(self.config_path,cfg)
        self.close_adapters()
        self.cfg=cfg
        self.directories=directories
        self.reports={}
        return self.state()

    def register_clock(self,value):
        result=self.clocks.add(value)
        if self.session and not self.session.closed.is_set():
            self.session.storage.required('clock_model',result)
        return result

    def register_report(self,ident,report):
        if self.busy():
            raise ValueError('运行中更新能力需要先停止并重新验证配置')
        if ident not in self.cfg['sources'] or self.cfg['sources'][ident]['transport']!='envelope':
            raise ValueError('能力报告接口仅用于已配置的 envelope 驱动来源')
        if not isinstance(report,dict):
            raise ValueError('能力报告必须为对象')
        encoded=json.dumps(report,allow_nan=False)
        if len(encoded.encode())>1024*1024:
            raise ValueError('能力报告超过 1 MiB，请引用独立校准资源')
        reports={**self.reports,ident:{**json.loads(encoded),'registered_monotonic_ns':time.monotonic_ns(),'config_hash':config_hash(self.cfg)}}
        try:
            result=preflight(self.cfg,reports,self.clocks)
        except (TypeError,AttributeError) as error:
            raise ValueError('能力报告字段格式无效: '+str(error)) from error
        self.reports=reports
        return result

    def prepare_adapters(self):
        if self.input_sessions():
            raise ValueError('采集期间不能重建输入订阅')
        self.close_adapters()
        if any(s['transport'].startswith('ros_') for s in self.cfg['sources'].values()):
            if not self.ros_config['enabled']:
                raise ValueError('请启用 ROS 连接，独立相机节点与管理器须使用同一 domain_id')
            from .ros_bridge import RosSources
            self.ros_bridge=RosSources(self,self.ros_config)
            self.ros_bridge.start()
        return preflight(self.cfg,self.reports,self.clocks)

    def start(self,metadata=None):
        with self.lock:
            if self.recording():
                raise ValueError('已经在录制，请先停止当前段')
            if len(self.input_sessions())>=4:
                raise ValueError('本地写盘收尾积压达到上限，请检查磁盘和编码状态')
            if any(s['transport'].startswith('ros_') for s in self.cfg['sources'].values()) and not self.ros_bridge:
                self.prepare_adapters()
            self._set_job_paused(True)
            self.job_resume_after=time.monotonic()+1
            try:
                self.session=Session(self.directory,self.cfg,self.clocks,self.reports,metadata,raw_only=True)
                return self.session.start()
            except BaseException:
                self._update_job_priority()
                raise

    def _capture_pressure(self):
        """Current bounded queue occupancy, including prior writers still draining."""
        queues=[queue for session in self.input_sessions()
                for queue in (session.ingress,session.storage.queue)]
        queues.extend(s.originals.queue for s in self.input_sessions() if s.originals)
        return max((max((state:=queue.status())['bytes']/queue.byte_limit,
                        state['items']/queue.items_limit) for queue in queues),default=0.)

    def _update_job_priority(self):
        if not self.input_sessions():
            self._set_job_paused(False)
            return
        pressure=self._capture_pressure()
        now=time.monotonic()
        if pressure>=.5:
            self.job_resume_after=now+1
            self._set_job_paused(True)
        elif self.job_paused:
            # Require a quiet second below the low watermark; avoid oscillating
            # between stop/continue while the writer is recovering.
            if pressure>.15:self.job_resume_after=now+1
            elif now>=self.job_resume_after:self._set_job_paused(False)

    def _set_job_paused(self,paused):
        """Signal only our own encoding child, never robot/driver processes."""
        with self.lock:
            process=self.process
            if process is None or process.poll() is not None:
                self.job_paused=False
                return
            if paused==self.job_paused:return
            try:os.kill(process.pid,signal.SIGSTOP if paused else signal.SIGCONT)
            except ProcessLookupError:
                self.job_paused=False
                return
            self.job_paused=paused
            self.job={**self.job,'paused_for_capture':paused}

    def _prioritize_capture(self):
        while not self.jobs_stop.wait(.05):
            with self.lock:
                self._update_job_priority()

    def request_stop(self):
        with self.lock:
            if not self.session or self.session.closed.is_set():
                return self.state()['status']
            result=self.session.request_stop()
            if self.session not in self.draining:
                self.draining.append(self.session)
            if self.workflow['package_after_stop']:
                self._enqueue('recording',self.session.id,{'name':'recording_'+self.session.id})
            return result

    def close_adapters(self):
        # This closes subscriptions only. Camera hardware belongs to its own node.
        if self.ros_bridge:
            self.ros_bridge.stop()
            self.ros_bridge=None

    def stop(self):
        try:
            for session in self.input_sessions():
                session.request_stop()
            for session in self.input_sessions():
                session.stop()
            return self.state()['status']
        finally:
            self.close_adapters()
            if self.ros_bridge:
                self.ros_bridge.stop()
                self.ros_bridge=None

    def sessions(self):
        result=[]
        roots=dict.fromkeys([str(self.directory),*self.directories])
        for path in sorted((p for root in roots for p in Path(root).glob('*')),reverse=True):
            if path.is_dir() and not path.is_symlink() and (path/'session.json').exists():
                value=summary(path)
                result.append({key:value.get(key) for key in ('session_id','directory','recording_mode','manifest','row_count','valid_rows','persistence_gap_count')})
        return result

    def start_job(self,operation,ident,data):
        path=self.session_path(ident)
        if read_json(path/'manifest.json',{}).get('state') not in {'completed','failed','interrupted'}:
            raise ValueError('该段仍在录制或保存收尾，暂不能导出')
        return self._enqueue(operation,ident,data)

    def _enqueue(self,operation,ident,data):
        with self.lock:
            if operation not in {'export','realign','recording'}:
                raise ValueError('未知数据操作')
            if operation!='recording' and read_json(self.session_path(ident)/'session.json',{}).get('recording_mode')=='raw':
                raise ValueError('原始录制请整段上传，由平台完成对齐、切片和导出')
            for job in self.jobs:
                if job['session_id']==ident and job['operation']==operation and job['state'] not in {'completed','failed'}:
                    return dict(job)
            name=data.get('name') or 'dataset_'+uuid.uuid4().hex[:8]
            if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,100}',name):
                raise ValueError('导出名称只允许字母、数字、下划线或横线')
            job={'id':uuid.uuid4().hex,'state':'queued','operation':operation,'session_id':ident,
                 'source':str(self.session_path(ident)),
                 'output':str(self.export_directory/name),'version':data.get('version','online')}
            self.jobs.append(job)
            atomic_json(self.jobs_path,self.jobs)
            self.job=dict(job)
            return dict(job)

    def _run_jobs(self):
        while not self.jobs_stop.wait(.2):
            with self.lock:
                if (self.session and self.session.closed.is_set() and self.workflow['package_after_stop']
                        and not any(j['session_id']==self.session.id and j['operation']=='recording' for j in self.jobs)):
                    self._enqueue('recording',self.session.id,{'name':'recording_'+self.session.id})
                job=next((j for j in self.jobs if j['state']=='queued' and
                    read_json(Path(j['source'])/'manifest.json',{}).get('state') in {'completed','failed','interrupted'}),None)
                if job is None:continue
                job['state']='starting'
                atomic_json(self.jobs_path,self.jobs)
                self.job=dict(job)
            try:
                Path(job['output']).parent.mkdir(parents=True,exist_ok=True)
                result=None
                with tempfile.TemporaryFile(mode='w+') as errors:
                    with self.lock:
                        if self.jobs_stop.is_set():
                            raise ValueError('管理器停止，后台任务已保留等待恢复')
                        self.process=subprocess.Popen([sys.executable,'-m','middleware.core.capture.job_process',
                            job['operation'],job['source'],job['output'],job['version']],
                            stdout=subprocess.PIPE,stderr=errors,text=True)
                        self.job_paused=False
                        self._update_job_priority()
                    for line in self.process.stdout:
                        message=json.loads(line)
                        if message['kind']=='progress':self.job={**job,**message['value'],'paused_for_capture':self.job_paused}
                        elif message['kind']=='result':result=message['value']
                        elif message['kind']=='error':job['error']=message['value']
                    code=self.process.wait()
                    self.process=None
                if code or result is None:
                    raise ValueError(job.get('error','后台处理进程退出；原始录制保留'))
                if self.on_export_complete and result.get('completion'):
                    try:self.on_export_complete(result['completion'])
                    except Exception as error:result['notification_error']=str(error)
                job.update(state='completed',result=result)
            except Exception as error:
                job.update(state='failed',error=str(error))
            finally:
                with self.lock:
                    if self.jobs_stop.is_set() and job['state']!='completed':job['state']='queued'
                    self.job=dict(job)
                    atomic_json(self.jobs_path,self.jobs)

    def shutdown(self):
        self.jobs_stop.set()
        self.stop()
        with self.lock:
            process=self.process
            if process and process.poll() is None:
                self._set_job_paused(False)
                process.terminate()
        self.job_thread.join(10)
        self.priority_thread.join(1)
