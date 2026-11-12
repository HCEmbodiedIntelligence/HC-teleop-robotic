# 启用未来的类型注解特性（PEP 563），允许在类型定义尚未完成或类自身方法内部使用类名作为注解
from __future__ import annotations

# 导入 select 模块，用于实现基于 I/O 多路复用的非阻塞 socket 监听与超时等待
import select
# 导入 socket 模块，用于底层的 UDP 套接字网络通信
import socket
# 导入 queue 模块，用于在网络接收线程和位姿消费者处理线程之间安全传递数据（跨线程解耦缓冲）
import queue
# 导入 threading 模块，用于并发多线程管理（网络接收线程、位姿消费线程、互斥锁与事件标志）
import threading
# 导入 time 模块，用于获取单调时钟（monotonic）和高精度纳秒/微秒时间戳进行超时与延迟计算
import time
# 导入类型提示注解 Any（任意类型）和 Callable（可调用对象/回调函数类型）
from typing import Any, Callable

# 从当前包的 network 模块导入针对 UDP 单播/广播的辅助处理工具：
# enable_packet_info: 开启套接字底层报文控制信息获取（如目标 IP，用于识别广播接口）
# receive_packet: 接收 UDP 数据报及其关联的接口元信息
# reply_packet: 向客户端准确回复 UDP 响应报文
from .network import enable_packet_info, receive_packet, reply_packet

# 从当前包的 protocol 模块导入协议相关常量与数据编解码工具：
from .protocol import (
    DISCOVERY_REQUEST,   # 发现请求协议标志字符串（PICO 头显局域网广播搜索服务端时发送）
    RELEASE_REQUEST,     # 释放会话请求标志字符串（PICO 头显退出应用或主动释放控制权时发送）
    PacketError,         # 解码数据包异常基类（格式非法、校验失败等）
    decode_pose_packet,  # 解码二进制/JSON VR 位姿数据包的解析函数
    encode_json_packet,  # 将事件字典编码为用于 UDP 下发的 JSON 字节流函数
    envelope,            # 构建统一事件格式信封包装函数（包含 kind, source, payload 等）
    is_sequence_newer,   # 判断新收到的序列号是否更新（处理 32 位无符号整数回绕及防重放/丢弃旧包）
    packet_loss_count,   # 计算上一包与当前包之间的丢包数量
)


class VrGateway:
    """VR 头显网关服务类。

    核心职责：
    1. 通过 UDP 双端口监听来自 VR 头显（如 PICO 4 Ultra）的服务发现请求与高频位姿数据报。
    2. 实现会话排他独占控制（Exclusive Session Lease），防止多台头显同时发送指令造成冲突。
    3. 解析位姿数据，校验数据有效性、序列号递增与时钟推进，滤除重放包、旧包与死锁静止帧。
    4. 将网络接收与数据消费分线程异步解耦，保证网络高频吞吐不被下游 ROS/Web 回调阻塞。
    5. 集成软看门狗机制，监测通信中断、追踪丢失和采样停滞，及时联动触发安全急停。
    6. 提供向 VR 头显双向通信能力，支持下发急停/恢复通知、重放进度与触觉反馈等事件。
    """

    def __init__(
        self,
        config: dict[str, Any],                                 # 配置字典，包含端口、超时、队列长度等参数
        on_pose: Callable[[Any, dict[str, Any]], None],          # 位姿接收回调：向下游控制层传入位姿对象与字典
        on_event: Callable[[dict[str, Any], list[str]], None],   # 事件广播回调：向 WebSocket 或上层总线派发事件
        on_timeout: Callable[[str], None],                       # 超时异常回调：在追踪丢失或数据停滞时触发安全急停
    ):
        # 保存外部传入的网关配置字典
        self.config = config
        # 保存位姿解析成功后的下游回调函数（通常连接至 ROS 发布与 IK 求解流程）
        self.on_pose = on_pose
        # 保存事件派发回调函数（用于广播手柄按键事件及 Web 端遥测数据）
        self.on_event = on_event
        # 保存超时/断连异常处理回调函数（联动底层控制驱动器进入安全停机态）
        self.on_timeout = on_timeout
        # 线程退出信号事件，用于通知各工作线程安全退出
        self._stop = threading.Event()
        # 网络工作线程实例（负责 UDP 套接字创建、多路复用与报文接收）
        self._thread: threading.Thread | None = None
        # 位姿消费处理线程实例（负责从队列取出位姿，执行 on_pose 与手柄按键解析）
        self._processor_thread: threading.Thread | None = None
        # 实时位姿缓冲队列：保持极短的队列深度（默认 8 帧，最小 2 帧）。
        # 设计考量：UDP 网络接收绝不能被 ROS 发布、JSON 序列化或 WebSocket 广播延迟所阻塞；
        # 在实时遥操作中，堆积的旧帧毫无意义，宁可丢弃旧采样也必须保持当前控制的极低延迟。
        self._pose_queue: queue.Queue[tuple[Any, dict[str, Any]]] = queue.Queue(
            maxsize=max(2, int(config.get("processing_queue_size", 8)))
        )
        # 是否启用排他控制模式（同一时刻仅允许一台 VR 设备拥有控制权）
        self.exclusive_mode: bool = bool(config.get("exclusive_mode", True))
        # 独占会话租约有效期（秒），超时未收到心跳则自动释放租约以允许新设备介入
        self.lease_timeout_s: float = float(config.get("lease_timeout_s", 3.0))
        # 当前持有独占租约的活跃客户端地址元组 (IP, Port)
        self._active_peer: tuple[str, int] | None = None
        # 当前租约到期时间戳（基于系统单调时钟 time.monotonic()，单位秒）
        self._lease_expires_at: float = 0.0
        # 用于向头显主动发送下行 UDP 报文的套接字（按需延迟初始化）
        self._outbound: socket.socket | None = None
        # 线程互斥锁，保护内部状态字典和对端信息的并发访问
        self._lock = threading.Lock()
        # 网关运行时状态与统计指标字典
        self._status: dict[str, Any] = {
            # 网关运行状态：未启用时为 "disabled"，否则初始状态为 "starting"
            "state": "disabled" if not config.get("enabled", True) else "starting",
            # 最近一次通信的对端地址 [IP, Port]
            "peer": None,
            # 当前获得授权的独占客户端 IP 地址
            "active_peer": None,
            # 最近一次收到数据包的墙上时间戳（time.time()）
            "last_packet_at": None,
            # 6DoF 空间追踪状态：头显 (head)、左手柄 (left)、右手柄 (right) 追踪有效性
            "tracking": {"head": False, "left": False, "right": False},
            # 控制器输入状态：按下的按键列表 (held)、扳机键开度 (trigger)、握持键开度 (grip)
            "inputs": {
                "left": {"held": [], "trigger": 0.0, "grip": 0.0},
                "right": {"held": [], "trigger": 0.0, "grip": 0.0},
            },
            # 协议版本号（从数据包头部解析提取）
            "protocol_version": None,
            # 累计捕获的手柄离散动作事件次数（如按键按下/释放）
            "controller_events": 0,
            # 累计成功接收的 UDP 数据报总数
            "received": 0,
            # 累计估算的网络丢包总数
            "lost": 0,
            # 累计因序列号过旧或重复而被丢弃的包总数
            "old": 0,
            # 累计格式异常或解码失败的无效包总数
            "invalid": 0,
            # 累计在排他模式下因未获授权而被拦截丢弃的包总数
            "unauthorized_dropped": 0,
            # 累计因消费者队列满而被丢弃的位姿帧总数
            "processing_dropped": 0,
            # 累计已交付给下游成功处理的位姿帧总数
            "processed": 0,
            # 累计主动向外发送的下行 UDP 报文总数
            "sent": 0,
            # 累计发送失败的次数
            "send_errors": 0,
            # 当前网络数据流是否处于超时断流状态
            "timeout": False,
            # 当前采样数据是否处于时间戳停滞未更新的假死状态（死帧检测）
            "sample_stale": False,
            # 最近一次记录的错误信息字符串
            "error": None,
        }

    def start(self) -> None:
        """启动 VR 网关的后台工作线程（位姿消费处理线程与 UDP 网络监听主循环线程）。"""
        # 如果配置项中明确禁用了 VR 网关，则直接退出不创建线程
        if not self.config.get("enabled", True):
            return
        # 创建位姿消费者处理线程（设为守护线程，随主进程生命周期退出）
        self._processor_thread = threading.Thread(
            target=self._process_poses, name="vr-pose-processor", daemon=True
        )
        # 启动位姿消费者线程
        self._processor_thread.start()
        # 创建 UDP 网络监听与接收主循环线程
        self._thread = threading.Thread(target=self._run, name="vr-gateway", daemon=True)
        # 启动网络工作线程
        self._thread.start()

    def stop(self) -> None:
        """优雅停止 VR 网关所有工作线程并关闭套接字资源。"""
        # 设置线程退出事件标志，通知各循环停止执行
        self._stop.set()
        # 等待网络工作线程退出，超时时间为 3 秒
        if self._thread is not None:
            self._thread.join(timeout=3)
        # 等待位姿消费者线程退出，超时时间为 3 秒
        if self._processor_thread is not None:
            self._processor_thread.join(timeout=3)
        # 如果存在出站 UDP 套接字，安全关闭并重置
        if self._outbound is not None:
            self._outbound.close()
            self._outbound = None

    def status(self) -> dict[str, Any]:
        """获取当前 VR 网关的实时运行状态快照（加锁并执行深拷贝，保证线程安全）。"""
        # 获取互斥锁，防止读取过程中被工作线程并发修改
        with self._lock:
            # 浅拷贝顶层字典
            result = dict(self._status)
            # 深拷贝 tracking 子字典
            result["tracking"] = dict(self._status["tracking"])
            # 深拷贝左右手柄输入状态子字典
            result["inputs"] = {
                side: dict(value) for side, value in self._status["inputs"].items()
            }
            # 提取当前激活的客户端 IP 字符串
            result["active_peer"] = self._active_peer[0] if self._active_peer else None
            # 计算当前排他会话是否仍处于受保护的锁定有效期内
            result["is_locked"] = (
                self.exclusive_mode
                and self._active_peer is not None
                and time.monotonic() < self._lease_expires_at
            )
            # 返回完整的状态字典
            return result

    def send_event(self, event: dict[str, Any]) -> bool:
        """通过 UDP 向目标 VR 头显下发控制事件（如急停/恢复通知、重放进度指示等）。"""
        # 优先读取配置文件中静态指定的出站目标 IP
        host = self.config.get("outbound_host") or ""
        # 若未配置静态目标，则自动匹配当前通信的动态对端
        if not host:
            # 排他模式下优先发送给持有当前租约的独占客户端 IP
            if self.exclusive_mode and self._active_peer is not None:
                host = self._active_peer[0]
            else:
                # 否则取状态中记录的最近发包客户端 IP
                current = self.status().get("peer")
                host = current[0] if current else ""
        # 若依然无法确定目标 IP，则无法发送，返回 False
        if not host:
            return False
        try:
            # 将事件字典编码为 JSON 字节串报文
            payload = encode_json_packet(event)
            # 检查报文长度是否超出单 UDP 数据报的最大有效载荷（65507 字节）
            if len(payload) > 65507:
                raise ValueError("JSON event exceeds UDP datagram limit")
            # 懒加载初始化出站 UDP 套接字
            if self._outbound is None:
                self._outbound = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 向目标主机的指定出站端口发送 UDP 数据包
            self._outbound.sendto(payload, (host, int(self.config["outbound_port"])))
            # 加锁自增发送成功计数
            with self._lock:
                self._status["sent"] += 1
            return True
        # 捕获网络系统调用或参数异常
        except (OSError, ValueError) as exc:
            # 加锁自增发送失败计数并记录错误详情
            with self._lock:
                self._status["send_errors"] += 1
                self._status["error"] = f"send failed: {exc}"
            return False

    def _run(self) -> None:
        """网络主循环线程入口：创建套接字、执行多路复用监听、处理设备发现握手及位姿报文。"""
        # 初始化位姿接收套接字和设备发现套接字变量
        pose_socket = discovery_socket = None
        try:
            # 创建位姿接收 UDP 套接字
            pose_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 开启地址复用，避免进程重启时因套接字处于 TIME_WAIT 状态导致端口绑定失败
            pose_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 设置套接字接收缓冲区为 1MB，防止因系统调度波动导致高频 UDP 数据丢弃
            pose_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
            # 绑定监听地址及位姿端口
            pose_socket.bind((self.config["listen_host"], int(self.config["pose_port"])))
            # 设置位姿套接字为非阻塞模式（配合 select 使用）
            pose_socket.setblocking(False)

            # 创建服务发现 UDP 套接字（用于响应 VR 头显的局域网广播）
            discovery_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 开启地址复用
            discovery_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # 绑定监听地址及服务发现端口
            discovery_socket.bind(
                (self.config["listen_host"], int(self.config["discovery_port"]))
            )
            # 设置发现套接字为非阻塞模式
            discovery_socket.setblocking(False)
            # 开启底层报文元信息获取，使 reply_packet 能感知接收该广播的本端网卡 IP
            discovery_packet_info = enable_packet_info(discovery_socket)
            # 预构造服务发现成功应答内容（告知头显接收端版本以及位姿接收端口）
            response = f"PICO_RECEIVER_V1|{self.config['pose_port']}".encode("ascii")
            # 更新网关运行状态为 "running"
            self._set_status(state="running", error=None)

            # 初始化上一帧序列号，用于检测丢包、重复与乱序
            last_sequence = None
            # 初始化上一帧发送端地址，用于检测对端重启或地址切换
            last_sender = None
            # 记录最近一次收到数据包的单调时间（秒）
            last_packet_time = time.monotonic()
            # 记录最近一次有效采样时间戳向前推进的时间（秒，用于死帧检测）
            last_sample_progress_time = last_packet_time
            # 记录上一帧数据包中头显的 VR 硬件时间戳
            last_vr_timestamp = None
            # 标记是否曾经收到过有效数据包
            ever_received = False
            # 标记头部追踪丢失告警是否处于激活中（避免频繁重复上报）
            head_invalid_active = False
            # 将配置的位姿超时阈值（毫秒）转换为秒
            timeout_seconds = float(self.config["pose_timeout_ms"]) / 1000.0

            # 只要未触发停止信号，就持续执行网络循环
            while not self._stop.is_set():
                # 使用 select 监听位姿套接字和发现套接字的可读就绪事件
                # 超时时间取 50ms 与超时阈值一半的较小值，保证快速响应同时避免空转打满 CPU
                readable, _, _ = select.select(
                    [pose_socket, discovery_socket], [], [], min(0.05, timeout_seconds / 2)
                )
                # 遍历所有就绪可读的套接字
                for current_socket in readable:
                    # 在做任何耗时的下游业务之前，循环排空（Drain）当前套接字中已排队的所有报文。
                    # 设计考量：保证 last_packet_time 严格对应网络层实际收包时间，不受业务回调耗时影响。
                    while True:
                        try:
                            # 若为发现套接字，调用带有元信息的 receive_packet 获取报文及来源
                            if current_socket is discovery_socket:
                                packet, sender, packet_info = receive_packet(current_socket, discovery_packet_info)
                            else:
                                # 若为位姿套接字，直接调用 recvfrom 获取最大 65535 字节的 UDP 数据包
                                packet, sender = current_socket.recvfrom(65535)
                        # 当套接字中所有可用数据均已读空时抛出 BlockingIOError，退出内层排空循环
                        except BlockingIOError:
                            break
                        # 处理发现端口接收到的命令
                        if current_socket is discovery_socket:
                            # 去除首尾空白字符
                            cmd = packet.strip()
                            # 客户端发起服务搜索探测请求
                            if cmd == DISCOVERY_REQUEST:
                                now = time.monotonic()
                                # 判断排他模式下是否正被其他活跃设备独占（且租约未过期）
                                is_busy = (
                                    self.exclusive_mode
                                    and self._active_peer is not None
                                    and now < self._lease_expires_at
                                    and sender[0] != self._active_peer[0]
                                )
                                # 若已被其他客户端独占，则回复 BUSY 繁忙消息，并告知当前占用者 IP
                                if is_busy:
                                    busy_resp = f"PICO_RECEIVER_BUSY|{self.config['pose_port']}|{self._active_peer[0]}".encode("ascii")
                                    reply_packet(discovery_socket, busy_resp, sender, packet_info)
                                else:
                                    # 否则回复正常发现握手响应，告知位姿端口
                                    reply_packet(discovery_socket, response, sender, packet_info)
                            # 客户端发来释放会话请求
                            elif cmd == RELEASE_REQUEST:
                                # 仅当发起者为当前独占会话客户端时才予以释放
                                if self._active_peer is not None and sender[0] == self._active_peer[0]:
                                    print(f"[VrGateway] Active session released by {sender[0]} (discovery)", flush=True)
                                    # 清空当前活跃对端及租约
                                    self._active_peer = None
                                    self._lease_expires_at = 0.0
                                    self._set_status(peer=None, active_peer=None)
                            # 发现端口报文处理完成，跳过后续位姿数据处理流程
                            continue

                        # 处理在位姿端口直接发来的会话释放请求
                        if packet.strip() == RELEASE_REQUEST:
                            # 仅当发起者为当前独占客户端时允许释放
                            if self._active_peer is not None and sender[0] == self._active_peer[0]:
                                print(f"[VrGateway] Active session released by {sender[0]} (pose)", flush=True)
                                # 重置租约和对端信息
                                self._active_peer = None
                                self._lease_expires_at = 0.0
                                self._set_status(peer=None, active_peer=None)
                            continue

                        # 记录接收到此帧有效数据时的单调时间戳
                        now = time.monotonic()
                        # 执行排他独占会话的鉴权与续约逻辑
                        if self.exclusive_mode:
                            # 如果当前无活跃设备，或者既有租约已超时自然过期，则允许当前设备获得独占权
                            if self._active_peer is None or now >= self._lease_expires_at:
                                if self._active_peer is None or self._active_peer[0] != sender[0]:
                                    print(f"[VrGateway] Exclusive session acquired by {sender[0]}:{sender[1]}", flush=True)
                                # 登记该发送端为当前独占对端
                                self._active_peer = sender
                                # 授予新的会话租约过期时间
                                self._lease_expires_at = now + self.lease_timeout_s
                                self._set_status(active_peer=sender[0])
                            # 若已有有效租约，但数据来自未经授权的其他设备 IP，则拦截丢弃
                            elif sender[0] != self._active_peer[0]:
                                self._increment("unauthorized_dropped")
                                continue
                            else:
                                # 授权对端发来数据，正常刷新续租时间戳
                                self._lease_expires_at = now + self.lease_timeout_s

                        # 尝试将二进制/JSON 报文解码为结构化位姿对象
                        try:
                            pose = decode_pose_packet(packet)
                        # 捕获格式错误或协议不匹配异常
                        except PacketError:
                            # 自增无效包统计计数并丢弃该包
                            self._increment("invalid")
                            continue
                        # 若检测到发送端 IP 或端口发生改变（说明客户端重连或换机），重置序列号与时钟参考基准
                        if last_sender is not None and sender != last_sender:
                            last_sequence = None
                            last_vr_timestamp = None
                        # 更新上一数据包发送端信息
                        last_sender = sender
                        # 校验序列号是否比上一帧更新（利用 32 位无符号差值算法，防御乱序和旧包重放）
                        if not is_sequence_newer(pose.sequence, last_sequence):
                            # 自增旧包计数并直接丢弃
                            self._increment("old")
                            continue
                        # 根据前后序列号差值计算网络丢包数量
                        lost = packet_loss_count(pose.sequence, last_sequence)
                        # 计算距离上一帧成功接收的时间差（毫秒）
                        gap_ms = (now-last_packet_time)*1000
                        # 若已有前序基准帧，且收包时间间隔异常偏大（>= 300ms），记录抖动现场
                        if last_sequence is not None and gap_ms >= 300:
                            # 保存一份关键事实用于排查网络卡顿，无需对每个包打日志以免刷屏
                            self._set_status(last_pose_gap={
                                'received_at': time.time(), 'receive_gap_ms': round(gap_ms, 1),
                                'missing_packets': lost, 'previous_sequence': last_sequence,
                                'sequence': pose.sequence, 'previous_vr_timestamp': last_vr_timestamp,
                                'vr_timestamp': pose.vr_timestamp})
                        # 更新当前最新序列号
                        last_sequence = pose.sequence
                        # 更新最近收到有效数据包的系统单调时间
                        last_packet_time = now
                        # 标记已成功收到过有效数据
                        ever_received = True
                        # 检查头显内部的硬件采样时间戳是否向前推移（微秒级比较）
                        sample_advanced = (
                            last_vr_timestamp is None
                            or abs(pose.vr_timestamp - last_vr_timestamp) > 1e-9
                        )
                        # 若采样时间戳产生有效推进，更新最后进展时间
                        if sample_advanced:
                            last_vr_timestamp = pose.vr_timestamp
                            last_sample_progress_time = now
                        # 将位姿对象转换为标准字典格式供下游使用
                        pose_dict = pose.as_dict()
                        # 在字典中附上接收时的系统高精度纳秒时间戳；
                        # 跨处理队列传递该时间戳，确保排队等待的手柄按键/Grip采样不会被误当作最新输入
                        pose_dict['hc_received_ns'] = time.time_ns()
                        # 提取空间追踪状态与输入字典
                        tracking = pose_dict["tracking"]
                        inputs = pose_dict["inputs"]
                        # 加锁原子更新内部状态字典
                        with self._lock:
                            self._status.update(
                                peer=[sender[0], sender[1]],
                                last_packet_at=time.time(),
                                tracking=tracking,
                                inputs=inputs,
                                protocol_version=pose.protocol_version,
                                received=self._status["received"] + 1,
                                lost=self._status["lost"] + lost,
                                timeout=False if sample_advanced else self._status["timeout"],
                                sample_stale=False if sample_advanced else self._status["sample_stale"],
                            )
                        # 将位姿数据压入消费者解耦队列
                        self._enqueue_pose(pose, pose_dict)

                        # 检测头显空间跟踪是否失效
                        if not tracking["head"] and not head_invalid_active:
                            # 首次丢失头显跟踪时，置位告警标志并触发超时/急停回调
                            head_invalid_active = True
                            self.on_timeout("head tracking invalid")
                        elif tracking["head"]:
                            # 恢复正常追踪时清除告警状态
                            head_invalid_active = False

                # 单次循环读空处理完毕后，获取当前单调时间
                loop_now = time.monotonic()
                # 检查排他会话租约是否已自然过期
                if (
                    self.exclusive_mode
                    and self._active_peer is not None
                    and loop_now >= self._lease_expires_at
                ):
                    print(
                        f"[VrGateway] Exclusive session lease expired for {self._active_peer[0]}",
                        flush=True,
                    )
                    # 清理过期租约与活跃对端状态
                    self._active_peer = None
                    self._set_status(peer=None, active_peer=None)

                # 计算自上一包网络接收以来经过的时间（秒）
                elapsed = loop_now - last_packet_time
                # 计算自上一次有效位姿采样推进以来经过的时间（秒）
                sample_elapsed = loop_now - last_sample_progress_time
                # 检查看门狗条件 1：曾连接过但已长时间未收到任何 UDP 数据包（网络彻底中断或头显休眠）
                if ever_received and elapsed > timeout_seconds and not self.status()["timeout"]:
                    # 标记网络断流超时，触发急停回调
                    self._set_status(timeout=True)
                    self.on_timeout(f"no VR pose data for {elapsed * 1000:.0f} ms")
                # 检查看门狗条件 2：持续收到 UDP 包但头显采样时间戳停滞不动（头显端线程卡死，持续发送旧死帧）
                elif (
                    ever_received
                    and sample_elapsed > timeout_seconds
                    and not self.status()["timeout"]
                ):
                    # 标记采样停滞超时，触发急停回调
                    self._set_status(timeout=True, sample_stale=True)
                    self.on_timeout(
                        f"VR pose sample stale for {sample_elapsed * 1000:.0f} ms"
                    )
        # 捕获网络底层套接字致命异常（如端口被外部占用等）
        except OSError as exc:
            self._set_status(state="error", error=f"{type(exc).__name__}: {exc}")
        # 线程退出时的终末清理逻辑
        finally:
            # 关闭位姿接收套接字
            if pose_socket is not None:
                pose_socket.close()
            # 关闭服务发现套接字
            if discovery_socket is not None:
                discovery_socket.close()
            # 若非异常错误退出，更新网关生命周期状态为停止 "stopped"
            if self.status()["state"] != "error":
                self._set_status(state="stopped")

    def _enqueue_pose(self, pose: Any, pose_dict: dict[str, Any]) -> None:
        """非阻塞式位姿入队策略：宁可丢弃旧样本，也绝不阻塞 UDP 接收网络循环。"""
        try:
            # 队列未满时直接非阻塞放入
            self._pose_queue.put_nowait((pose, pose_dict))
            return
        # 队列已满，进入挤旧迎新逻辑
        except queue.Full:
            pass

        # 在机器人实时遥操作中，队列积压造成的历史延迟远比丢掉一两帧旧位姿更具破坏性。
        # 尝试弹出队列中最旧的一项数据并自增丢弃计数，从而为最新样本腾出空间。
        try:
            self._pose_queue.get_nowait()
            self._increment("processing_dropped")
        except queue.Empty:
            pass
        try:
            # 再次尝试将最新数据放入队列
            self._pose_queue.put_nowait((pose, pose_dict))
        except queue.Full:
            # 极端并发竞争导致仍满时，自增丢弃计数
            self._increment("processing_dropped")

    def _process_poses(self) -> None:
        """位姿消费处理线程：在独立线程中执行下游 ROS 控制发布、Web 广播和手柄事件解析。"""
        # 根据配置中的 web_pose_hz（默认 30Hz）计算向前端 Web 端广播位姿的最小时间间隔
        web_interval = 1.0 / max(1.0, float(self.config.get("web_pose_hz", 30.0)))
        # 记录最近一次向 Web 端广播位姿的时间戳
        last_web_emit = 0.0
        # 消费循环：只要未收到停止信号就持续消费
        while not self._stop.is_set():
            try:
                # 从解耦队列中阻塞获取位姿数据，设置 0.1 秒超时以保证能周期性响应退出信号
                pose, pose_dict = self._pose_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                # 复用接收线程已经组装好的字典对象；
                # 避免每个 60Hz 报文在此重复创建深层嵌套的 list 和 dict 造成高内存分配开销。
                # 触发位姿下游回调函数（进行 ROS 话题发布和逆运动学 IK 求解）
                self.on_pose(pose, pose_dict)
                # 自增已处理计数
                self._increment("processed")

                now = time.monotonic()
                # 按照配置的 web_interval 进行节流，向 WebSocket 广播用于前端 3D 可视化的位姿事件
                if now - last_web_emit >= web_interval:
                    self.on_event(
                        envelope("vr_pose", "vr_udp", pose_dict), ["websocket"]
                    )
                    last_web_emit = now

                # 遍历左右手柄的按键输入，检测按键边沿变化（按下/释放）
                for side, controller_input in (
                    ("left", pose.left_input),
                    ("right", pose.right_input),
                ):
                    # 若本帧中既没有按键被按下（上升沿），也没有按键被释放（下降沿），则忽略
                    if not (
                        controller_input.pressed_mask
                        or controller_input.released_mask
                    ):
                        continue
                    # 组装手柄动作事件负载字典
                    event_payload = {
                        "side": side,
                        "sequence": pose.sequence,
                        "vr_timestamp": pose.vr_timestamp,
                        # 当前保持按下状态的按键名列表
                        "held": controller_input.decode_buttons(
                            controller_input.held_mask
                        ),
                        # 本帧新按下的按键名列表（上升沿）
                        "pressed": controller_input.decode_buttons(
                            controller_input.pressed_mask
                        ),
                        # 本帧新释放的按键名列表（下降沿）
                        "released": controller_input.decode_buttons(
                            controller_input.released_mask
                        ),
                        # 原始位掩码
                        "pressed_mask": controller_input.pressed_mask,
                        "released_mask": controller_input.released_mask,
                    }
                    # 自增手柄事件捕获计数
                    self._increment("controller_events")
                    # 通过事件回调将 vr_controller_event 信封广播到 WebSocket 客户端（供网页或录制使用）
                    self.on_event(
                        envelope("vr_controller_event", "vr_udp", event_payload),
                        ["websocket"],
                    )
            # 捕获下游业务处理过程中的任意异常，避免下游代码错误导致整个网关崩溃或看门狗失效
            except Exception as exc:
                self._set_status(error=f"pose processing failed: {type(exc).__name__}: {exc}")

    def _set_status(self, **changes: Any) -> None:
        """线程安全地批量更新网关内部状态字典。"""
        with self._lock:
            self._status.update(changes)

    def _increment(self, key: str) -> None:
        """线程安全地对网关内部状态字典中的指定数值计数器自增 1。"""
        with self._lock:
            self._status[key] += 1
