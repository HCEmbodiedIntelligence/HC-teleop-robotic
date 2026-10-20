import json
import ipaddress
import re
import ssl
import time
from http.client import HTTPException
from datetime import timezone
from email.utils import parsedate_to_datetime
from urllib.error import HTTPError, URLError
from urllib.request import HTTPRedirectHandler, ProxyHandler, Request, build_opener

from .common import Limiter, UploadError, canonical, credential, valid_url


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        return None


class UploadProxyHandler(ProxyHandler):
    """Local services and SSH forwards must not pass through a system proxy."""
    def proxy_open(self, req, proxy, type):
        host = valid_url(req.full_url).hostname
        try:
            local = ipaddress.ip_address(host).is_loopback
        except ValueError:
            local = host.lower() == 'localhost'
        if local:
            return None
        return super().proxy_open(req, proxy, type)


def retry_after(value):
    try:
        return max(0, float(value))
    except (ValueError, TypeError):
        try:
            dt = parsedate_to_datetime(value)
            return max(0, dt.replace(tzinfo=dt.tzinfo or timezone.utc).timestamp() - time.time())
        except (ValueError, TypeError, OverflowError):
            return 0


class PartStream:
    def __init__(self, stream, size, rate):
        self.stream, self.remaining, self.limiter = stream, size, Limiter(rate)

    def read(self, size=65536):
        data = self.stream.read(min(size, self.remaining, 65536))
        if not data and self.remaining:
            raise UploadError("LOCAL_CONTENT_CHANGED")
        self.remaining -= len(data)
        self.limiter.account(len(data))
        return data


class Http:
    def __init__(self, cfg):
        self.cfg, self.opener = cfg, build_opener(NoRedirect(), UploadProxyHandler())
        self.base = cfg["platform_base_url"] + "/api/v1/robot-ingest/uploads"

    def request(self, method, url, *, payload=None, body=None, length=None, api=True, anonymous=False):
        valid_url(url)
        headers = {}
        if api:
            headers = {"Content-Type": "application/json"}
            if not anonymous:
                headers["Authorization"] = "Bearer " + credential(self.cfg["credential_ref"])
            body = canonical(payload) if payload is not None else None
        elif length is not None:
            headers["Content-Length"] = str(length)
        try:
            with self.opener.open(Request(url, data=body, headers=headers, method=method),
                                  timeout=self.cfg["timeout_seconds"]) as response:
                if api:
                    raw = response.read(16*1024**2+1)
                    if len(raw) > 16*1024**2:
                        raise UploadError("RESPONSE_TOO_LARGE", "联系平台管理员核对响应")
                    try:
                        return json.loads(raw)
                    except ValueError:
                        raise UploadError("RESPONSE_INVALID", "响应不完整，按原 UUID 对账", retry=True) from None
                # A missing ETag is uncertain, authorize-parts will reconcile.
                if not response.headers.get("ETag"):
                    raise UploadError("PUT_RESPONSE_UNCERTAIN", retry=True)
                return None
        except HTTPError as error:
            after = retry_after(error.headers.get("Retry-After"))
            code = "HTTP_" + str(error.code)
            if api:
                try:
                    value = json.loads(error.read(65536)).get("code", "")
                    if isinstance(value, str) and re.fullmatch("[A-Z][A-Z0-9_]{0,95}", value):
                        code = value
                except (ValueError, AttributeError):
                    pass
            if error.code in {408, 429} or error.code >= 500 or (not api and error.code in {401, 403}):
                raise UploadError(code, "等待网络恢复/重新授权同一分片", retry=True, after=after) from None
            if error.code == 409 and code not in {"CLIENT_UPLOAD_ID_CONFLICT", "ASSET_SHA256_MISMATCH", "ASSET_CRC64_MISMATCH", "TASK_OWNERSHIP_CHANGED", "COLLECTION_TASK_INACTIVE", "TASK_NOT_READY", "LOCAL_ROBOT_MISMATCH", "CONNECTION_TICKET_USED", "CONNECTION_PERMISSION_CHANGED", "ROBOT_MODEL_REQUIRED", "ROBOT_MODEL_UNAVAILABLE", "ROBOT_IDENTITY_DISABLED"}:
                raise UploadError(code, "先查询服务端事实；持续冲突请核对资产", retry=True, after=after, conflict=True) from None
            action = "检查平台任务、会话及文件校验结果；修复后显式 retry"
            if error.code in {401, 403}:
                action = "更新同一凭据引用并检查权限/TLS设置，然后停止 worker 并执行 retry"
            if error.code == 410:
                action = "会话已过期；联系平台管理员确认 Raw 事实并显式重新导出，勿反复创建上传"
            actions = {
                'CONNECTION_TICKET_EXPIRED': '连接链接已过期，请在平台接入详情重新复制链接',
                'CONNECTION_RETRY_EXPIRED': '原连接的恢复期限已过，请在平台重新生成连接链接',
                'CONNECTION_TICKET_INVALID': '连接链接无效，请在平台接入详情重新复制链接',
                'CONNECTION_TICKET_USED': '连接链接已被其他请求使用，请重新生成连接链接',
                'CONNECTION_PERMISSION_CHANGED': '平台机器人权限或模型已变化，请检查后重新生成连接链接',
                'TASK_OWNERSHIP_CHANGED': '任务归属已变化，数据已保留；请联系平台管理员处理原任务，不会自动改投',
                'COLLECTION_TASK_INACTIVE': '任务已关闭或取消，数据已保留；请检查原任务状态',
                'TASK_NOT_READY': '任务的数据集、区域或接收配置尚未就绪，请在平台检查',
                'LOCAL_ROBOT_MISMATCH': '当前机器人配置与平台映射不一致，请检查机器人配置',
                'ROBOT_MODEL_REQUIRED': '请先在平台为机器人绑定已发布模型',
                'ROBOT_MODEL_UNAVAILABLE': '平台机器人模型尚未发布或资产不可用，请检查模型配置',
                'ROBOT_IDENTITY_DISABLED': '平台已停用此机器人上传身份，请联系管理员恢复',
                'ROBOT_CREDENTIAL_REVOKED': '上传凭据已撤销；使用同一机器人的新连接链接重新连接，再重试原录制',
                'SOURCE_FORMAT_NOT_ALLOWED': '机器人上传权限不允许当前录制格式，请检查平台上传规则',
            }
            action = actions.get(code, action)
            raise UploadError(code, action) from None
        except (URLError, TimeoutError, ConnectionError, OSError, ssl.SSLError, HTTPException):
            raise UploadError("NETWORK_UNCERTAIN", "检查平台公开地址、网络及证书；恢复后自动对账", retry=True) from None

    def api(self, method, suffix="", payload=None):
        return self.request(method, self.base + suffix, payload=payload)

    def put(self, url, stream, size):
        destination = valid_url(url).hostname
        origin = valid_url(self.cfg["platform_base_url"]).hostname
        def loopback(host):
            try:
                return ipaddress.ip_address(host).is_loopback
            except ValueError:
                return host.lower() == "localhost"
        if loopback(destination) and not loopback(origin):
            raise UploadError("UPLOAD_URL_NOT_EXTERNAL", "平台签发了仅本机可达的地址；由平台修正公开 origin 后重试")
        return self.request("PUT", url, api=False, length=size,
                            body=PartStream(stream, size, self.cfg["bandwidth_bytes_per_second"]))
