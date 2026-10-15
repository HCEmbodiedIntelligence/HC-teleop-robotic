import math
import random
import time
from datetime import datetime
from pathlib import Path
from urllib.parse import quote

from .bundle import manifest_for
from .artifacts import ArtifactLocks, manifest_for_ready
from .common import UploadError, canonical, validate
from .http import Http


def state_for(state, processing, quality):
    if state in {"UPLOADING", "READY_TO_COMMIT"}:
        return "uploading", False, "继续上传"
    if state != "COMMITTED":
        return "needs_attention", True, "会话暂停、取消或失败；核对平台状态，暂停可显式 resume"
    if processing in {"PARTIALLY_FAILED", "FAILED"}:
        return "processing_failed", True, "保留成功 episode；在平台重试失败处理后执行本地 retry 查询，不重新上传 Raw"
    if processing == "READY":
        return {"PASS": ("ready", True, "进入平台标注；此状态不代表已审核发布"),
                "RISK": ("needs_attention", True, "在平台复核质量风险"),
                "REJECT": ("quality_rejected", True, "按源 episode 补采或复核质检"),
                "PENDING": ("needs_attention", True, "处理结果缺少质检结论，请联系平台管理员")}[quality]
    if processing in {"DISCOVERING_EPISODES", "PROCESSING"}:
        return "processing", False, "等待平台处理"
    return "raw_committed", False, "Raw 已提交，等待平台处理；无需重传"


class Worker:
    def __init__(self, queue, cfg, should_stop=None):
        self.queue, self.cfg = queue, cfg
        self.should_stop = should_stop or (lambda: False)

    def facts(self, job, response):
        validate(response, "robot-ingest-upload-response")
        upload = response["data"]
        manifest = job["manifest"]
        if (upload["client_upload_id"] != job["id"] or
                upload["authenticated_robot_id"] != manifest["robot_id"] or
                upload["request_robot_id"] != manifest["robot_id"] or
                upload["target"]["collection_task_id"] != manifest["collection_task_id"] or
                (job["upload_id"] and job["upload_id"] != upload["upload_id"])):
            raise UploadError("SERVER_IDENTITY_MISMATCH", "停止上传并联系平台管理员核对身份和任务")
        remote = upload["manifest"]
        # Pydantic responses add optional defaults: compare only our frozen fields.
        def contains(expected, actual, key=""):
            if isinstance(expected, dict):
                return isinstance(actual, dict) and all(k in actual and contains(v, actual[k], k) for k, v in expected.items())
            if isinstance(expected, list):
                return isinstance(actual, list) and len(expected) == len(actual) and all(contains(a, b, key) for a, b in zip(expected, actual))
            if key.endswith("_at") and isinstance(expected, str) and isinstance(actual, str):
                try:
                    left = datetime.fromisoformat(expected.replace("Z", "+00:00"))
                    right = datetime.fromisoformat(actual.replace("Z", "+00:00"))
                    return left.tzinfo is not None and right.tzinfo is not None and left == right
                except ValueError:
                    pass
            return expected == actual
        if not contains(manifest, remote):
            raise UploadError("SERVER_MANIFEST_CONFLICT", "核对原 UUID 与不可变 manifest；不要重建上传")
        expected = {a["asset_id"]: a for a in manifest["assets"]}
        if len(upload["assets"]) != len(expected):
            raise UploadError("SERVER_ASSET_CONFLICT")
        for asset in upload["assets"]:
            item = expected.get(asset["asset_id"])
            if not item or any(asset["expected_" + key] != item[key] for key in ("size_bytes", "sha256", "crc64")) or asset["path"] != item["path"]:
                raise UploadError("SERVER_ASSET_CONFLICT")
        state, terminal, action = state_for(upload["state"], upload["processing_status"], upload["quality_status"])
        if upload["state"] == "COMMITTED" and self.cfg["processing_endpoint"]:
            terminal = False  # A crash must not strand a terminal result without its episode receipt.
        job.update(upload_id=upload["upload_id"], raw_source_id=upload.get("raw_source_id"),
                   server_state=upload["state"], processing_status=upload["processing_status"],
                   quality_status=upload["quality_status"], state=state, terminal=terminal,
                   next_action=action, code=None, next_at=0 if terminal else time.time()+self.cfg["poll_seconds"])
        self.queue.save(job)
        return upload

    def check_cancelled(self, job):
        if self.should_stop():
            raise UploadError('SERVICE_STOPPING', '服务停止后从原上传身份恢复', retry=True)
        if self.queue is not None and self.queue.cancelled(job['id']):
            raise UploadError('UPLOAD_CANCELLED', '本地上传已取消；平台已接收部分保持原身份，不自动删除平台数据')

    def process(self, job):
        self.check_cancelled(job)
        if job.get('artifact_id'):
            with ArtifactLocks(job['frozen']['artifact_state_dir']).read(job['artifact_id']):
                self.check_cancelled(job)
                return self._process(job)
        return self._process(job)

    def _process(self, job):
        cfg = {**self.cfg, **job["frozen"]}
        http = Http(cfg)
        # Old servers cannot validate large completion seals. Negotiate before
        # creating a session or uploading any bytes; small/legacy jobs stay compatible.
        if job['manifest']['source_format']=='CAPTURE_BUNDLE' and len(job['manifest']['assets'])>256:
            self.recording_capacity(job,http)
        path = "/" + quote(job["upload_id"], safe="") if job["upload_id"] else ""
        if not job["upload_id"]:
            self.verify(job, cfg)
            response = http.api("POST", payload=job["manifest"])
        else:
            response = http.api("GET", path)
        upload = self.facts(job, response)
        path = "/" + quote(job["upload_id"], safe="")
        if upload["state"] == "PAUSED" and job.pop("resume_requested", False):
            upload = self.facts(job, http.api("POST", path + ":resume"))
        if upload["state"] == "COMMITTED":
            self.processing(job, http, path)
            return
        if job["terminal"]:
            return
        self.verify(job, cfg)
        remote = {a["asset_id"]: a for a in upload["assets"]}
        for asset in job["manifest"]["assets"]:
            self.check_cancelled(job)
            if remote[asset["asset_id"]]["state"] == "COMPLETED":
                continue
            if remote[asset["asset_id"]]["state"] != "UPLOADING":
                raise UploadError("ASSET_TERMINAL", "平台资产失败/取消，请检查平台错误")
            asset_path = path + "/assets/" + quote(asset["asset_id"], safe="")
            part_size = max(cfg["part_size_bytes"], math.ceil(asset["size_bytes"] / 10000))
            count = math.ceil(asset["size_bytes"] / part_size)
            with (Path(job["root"]) / asset["path"]).open("rb") as stream:
                for number in range(1, count+1):
                    self.check_cancelled(job)
                    grant = self.grant(http, asset_path, number, job, asset)
                    uploaded = self.parts(grant)
                    length = min(part_size, asset["size_bytes"] - (number-1)*part_size)
                    if number in uploaded:
                        if uploaded[number]["size_bytes"] != length:
                            raise UploadError("PART_SIZE_CONFLICT", "分片规划与平台不一致；检查固定队列配置")
                        job['uploaded_bytes'] = sum(a['size_bytes'] for a in job['manifest']['assets']
                            if remote[a['asset_id']]['state'] == 'COMPLETED') + min(number * part_size, asset['size_bytes'])
                        self.queue.save(job)
                        continue
                    auth = [p for p in grant["authorizations"] if p["part_number"] == number]
                    if len(auth) != 1:
                        raise UploadError("PART_AUTHORIZATION_INVALID")
                    stream.seek((number-1)*part_size)
                    http.put(auth[0]["url"], stream, length)
                    job['uploaded_bytes'] = sum(a['size_bytes'] for a in job['manifest']['assets']
                        if remote[a['asset_id']]['state'] == 'COMPLETED') + min(number * part_size, asset['size_bytes'])
                    self.queue.save(job)
            grant = self.grant(http, asset_path, 1, job, asset)
            parts = self.parts(grant)
            if sorted(parts) != list(range(1, count+1)) or any(
                    parts[n]["size_bytes"] != min(part_size, asset["size_bytes"]-(n-1)*part_size) for n in parts):
                raise UploadError("PART_LIST_INCOMPLETE", "按服务端分片事实继续恢复", retry=True)
            self.facts(job, http.api("POST", asset_path + ":complete", {"parts": [
                {"part_number": n, "etag": parts[n]["etag"]} for n in sorted(parts)]}))
            remote[asset['asset_id']]['state'] = 'COMPLETED'
        # Recheck immutable bytes before committing even if all parts already existed.
        self.verify(job, cfg)
        self.facts(job, http.api("POST", path + ":commit"))
        if job["server_state"] == "COMMITTED":
            self.processing(job, http, path)

    def verify(self, job, cfg):
        self.check_cancelled(job)
        _, _, manifest = (manifest_for_ready(job['descriptor'], cfg, job['id']) if job.get('descriptor')
                          else manifest_for(job["root"], cfg, job["id"]))
        if canonical(manifest) != canonical(job["manifest"]):
            raise UploadError("LOCAL_CONTENT_CHANGED", "保留旧队列；用新 export_id 重新导出")

    @staticmethod
    def recording_capacity(job,http):
        try:
            value=http.api('GET',':capabilities')
        except UploadError as error:
            if error.code in {'HTTP_404','HTTP_405','NOT_FOUND'}:
                raise UploadError('RECORDING_CAPACITY_UNSUPPORTED',
                    '此大包需要支持 4096 总资产契约的服务端；保留本地包和队列，先升级平台再重试') from None
            raise
        if (not isinstance(value,dict) or value.get('schema_version')!='robot-ingest-capabilities/v1'
                or value.get('robot_id')!=job['manifest']['robot_id']
                or type(value.get('max_total_assets')) is not int
                or type(value.get('max_recording_assets')) is not int
                or not 1<=value['max_total_assets']<=4096
                or not 0<=value['max_recording_assets']<value['max_total_assets']):
            raise UploadError('UPLOAD_CAPABILITIES_INVALID','检查平台版本、机器人身份和大包能力响应')
        total=len(job['manifest']['assets'])
        if total>value['max_total_assets'] or total-1>value['max_recording_assets']:
            raise UploadError('RECORDING_CAPACITY_EXCEEDED',
                '当前机器人上传策略不允许此资产数量；保留文件和队列，请管理员核对额度后重试')

    def grant(self, http, path, number, job, asset):
        value = http.api("POST", path + ":authorize-parts", {"part_numbers": [number]})
        if value.get("upload_id") != job["upload_id"] or value.get("asset_id") != asset["asset_id"]:
            raise UploadError("PART_IDENTITY_MISMATCH")
        return value

    @staticmethod
    def parts(grant):
        result = {}
        for p in grant["uploaded_parts"]:
            if not isinstance(p["part_number"], int) or not 1 <= p["part_number"] <= 10000 or not p["etag"] or p["part_number"] in result:
                raise UploadError("PART_LIST_INVALID")
            result[p["part_number"]] = p
        return result

    def processing(self, job, http, path):
        if job['manifest']['source_format']=='CAPTURE_BUNDLE':
            request_id=job.get('processing_retry_request')
            result=(http.api('POST',path+'/recording:retry',{'request_id':request_id}) if request_id
                    else http.api('GET',path+'/recording'))
            if (result.get('upload_id')!=job['upload_id'] or result.get('raw_source_id')!=job['raw_source_id']
                    or result.get('status') not in {'REGISTERING','AWAITING_SLICE','SLICED','FAILED'}
                    or type(result.get('terminal')) is not bool):
                raise UploadError('PROCESSING_IDENTITY_MISMATCH')
            complete=result['status'] in {'AWAITING_SLICE','SLICED'}
            failed=result['status']=='FAILED'
            import re
            recording=result.get('recording_id')
            if (result['terminal']!=(complete or failed) or type(result.get('poll_after_seconds')) is not int
                    or not 0<=result['poll_after_seconds']<=60
                    or (complete and (not isinstance(recording,str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}',recording)
                        or result.get('page_path')!='/recordings/'+recording+'/slice' or result['poll_after_seconds']!=0))
                    or (not complete and (recording is not None or result.get('page_path') is not None))):
                raise UploadError('PROCESSING_RESULT_INCONSISTENT')
            job.update(state='sliced' if result['status']=='SLICED' else 'awaiting_slice' if complete else 'processing_failed' if failed else 'processing',terminal=complete or failed,
                processing_status='FAILED' if failed else 'NOT_REQUESTED' if complete else 'PROCESSING',
                recording_id=result.get('recording_id'),page_path=result.get('page_path'),
                recording_status=result['status'],registration_error=result.get('error_code'),
                next_at=0 if complete or failed else time.time()+5,
                next_action=('切片已确认；请到平台查看 episode 的处理与质检结果' if result['status']=='SLICED'
                    else '到平台录制页面切片；尚未生成训练 episode' if complete
                    else '平台原始录制处理失败；修复原因后重试处理，原始数据已保留' if failed
                    else '平台登记失败，正在自动重试；保留本地数据' if result.get('error_code')
                    else '平台正在处理原始录制并准备预览'))
            job.pop('processing_retry_request',None)
            self.queue.save(job)
            return
        if not self.cfg["processing_endpoint"]:
            return
        request_id = job.get("processing_retry_request")
        result = (http.api("POST", path + ":retry-processing", {"request_id": request_id}) if request_id
                  else http.api("GET", path + "/processing"))
        validate(result, "processing-result-proposed")
        if result["upload_id"] != job["upload_id"] or result["raw_source_id"] != job["raw_source_id"]:
            raise UploadError("PROCESSING_IDENTITY_MISMATCH")
        state, terminal, action = state_for("COMMITTED", result["processing_status"], result["quality_status"])
        if terminal != result["terminal"] or (terminal and result["poll_after_seconds"] != 0):
            raise UploadError("PROCESSING_RESULT_INCONSISTENT")
        if state == "ready" and (not result["episodes"] or any(e["status"] != "READY" or e["quality_status"] != "PASS" for e in result["episodes"])):
            raise UploadError("PROCESSING_RESULT_INCONSISTENT")
        job.update(state=state, terminal=terminal, next_action=action, episodes=result["episodes"],
                   processing_status=result["processing_status"], quality_status=result["quality_status"],
                   next_at=0 if terminal else time.time()+max(1, result["poll_after_seconds"]))
        job.pop("processing_retry_request", None)
        self.queue.save(job)

    def tick(self):
        for job in self.queue.jobs(due=True):
            if self.should_stop():
                break
            try:
                self.process(job)
                job["attempts"] = 0
            except UploadError as error:
                job["attempts"] += 1
                retry = error.retry and (not self.cfg["max_attempts"] or job["attempts"] < self.cfg["max_attempts"])
                # Repeated conflicts need human action; never generate a replacement UUID.
                if error.conflict and job["attempts"] >= 3:
                    retry = False
                delay = min(self.cfg["retry_max_seconds"], self.cfg["retry_base_seconds"] * 2**min(job["attempts"]-1, 20))
                delay = max(error.after, min(self.cfg["retry_max_seconds"], delay*random.uniform(1, 1.2)))
                job.update(state="retry_wait" if retry else "needs_attention", terminal=not retry,
                           next_at=time.time()+delay if retry else 0, code=error.code, next_action=error.action)
            except (OSError, ValueError, KeyError, TypeError):
                job.update(state="needs_attention", terminal=True, next_at=0,
                           code="LOCAL_OR_PROTOCOL_ERROR", next_action="检查本地文件/磁盘及平台协议；修复后显式 retry")
            if self.queue.cancelled(job['id']):
                job.update(state='cancelled', terminal=True, next_at=0, code='UPLOAD_CANCELLED')
            self.queue.save(job)
