"""Versioned mappings; no fitted guesses or silent retiming of written samples."""
from dataclasses import asdict, dataclass
import math


@dataclass(frozen=True)
class ClockModel:
    id: str
    clock_id: str
    epoch: int
    device_origin_ns: int
    common_origin_ns: int
    scale: float = 1.
    uncertainty_ns: int | None = None
    evidence: str = ''
    valid_from_ns: int = 0
    valid_until_ns: int = 2**63-1

    def map(self, stamp):
        if not self.valid_from_ns <= stamp <= self.valid_until_ns:
            raise ValueError('设备时间超出时钟模型验证区间')
        # Subtract first to retain nanosecond precision for large device epochs.
        return self.common_origin_ns+round((stamp-self.device_origin_ns)*self.scale)


class Clocks:
    def __init__(self):
        self.models = {}

    def add(self, value):
        model = ClockModel(**value)
        if any(not isinstance(getattr(model,k),str) or len(getattr(model,k))>2048 for k in ('id','clock_id','evidence')):
            raise ValueError('时钟标识和证据必须是最多 2048 字符的字符串')
        if len(self.models)>=16384 and model.id not in self.models:
            raise ValueError('时钟模型版本达到 16384 个上限，请结束并重新连接采集器后注册当前模型')
        if not model.id or not model.clock_id or type(model.epoch) is not int or model.epoch<0:
            raise ValueError('无效时钟模型标识或代次')
        if not math.isfinite(model.scale) or model.scale<=0:
            raise ValueError('时钟漂移系数必须为有限正数')
        for key in ('device_origin_ns','common_origin_ns','valid_from_ns','valid_until_ns'):
            if type(getattr(model,key)) is not int:
                raise ValueError('时钟时间戳必须为整数纳秒')
        if model.uncertainty_ns is not None and (type(model.uncertainty_ns) is not int or model.uncertainty_ns<0 or not model.evidence):
            raise ValueError('已知时钟误差界必须为非负整数且有测量依据')
        if model.id in self.models and self.models[model.id]!=model:
            raise ValueError('已注册的时钟模型版本不可修改，请使用新 ID')
        self.models[model.id]=model
        return asdict(model)

    def stamp(self, metadata):
        m = self.models.get(metadata['clock_model_id'])
        if m is None or (m.clock_id,m.epoch)!=(metadata['clock_id'],metadata['clock_epoch']):
            raise ValueError('未注册的时钟映射或 clock_epoch 不匹配')
        capture = m.map(metadata['source_timestamp_ns'])
        if metadata.get('capture_time_ns',capture)!=capture:
            raise ValueError('capture_time_ns 与已注册时钟映射不一致')
        return capture,m


def exposure_midpoint(raw_ns, semantics, actual_us, model):
    if semantics not in {'exposure_start','exposure_end','exposure_midpoint'}:
        return None
    offset = {'exposure_start':1,'exposure_end':-1,'exposure_midpoint':0}[semantics]
    # Exposure duration uses device microseconds; map both endpoints, including drift.
    return model.map(raw_ns+round(offset*actual_us*500))


class RosClockJump(RuntimeError):
    pass


class RosSteadyMapping:
    """Local scheduling conversion only; never changes a sample's ROS stamp."""
    def __init__(self,tolerance_ns=5_000_000):
        self.tolerance_ns=tolerance_ns
        self.ros_ns=self.steady_ns=None
        self.epoch=0
        self.valid=False

    def observe(self,ros_ns,steady_ns):
        if ros_ns<=0:
            self.valid=False
            raise RosClockJump('ROS clock is uninitialized')
        if self.ros_ns is not None and abs((ros_ns-self.ros_ns)-(steady_ns-self.steady_ns))>self.tolerance_ns:
            self.epoch+=1
            self.valid=False
            raise RosClockJump('ROS clock jump: alignment epoch ended')
        self.ros_ns,self.steady_ns=ros_ns,steady_ns
        self.valid=True

    def to_steady(self,ros_ns):
        if not self.valid:
            raise RosClockJump('ROS-to-steady epoch is invalid')
        return self.steady_ns+(ros_ns-self.ros_ns)

    def to_ros(self,steady_ns):
        if not self.valid:
            raise RosClockJump('ROS-to-steady epoch is invalid')
        return self.ros_ns+(steady_ns-self.steady_ns)
