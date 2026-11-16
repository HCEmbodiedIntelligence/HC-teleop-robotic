import React, { useEffect, useRef, useState, lazy, Suspense } from "react";
import { createRoot } from "react-dom/client";
const RobotView = lazy(() => import("./RobotView"));
import "./style.css";

type J = Record<string, any>;
const tabs = [
  "状态监控",
  "话题录制",
  "数据集管理",
  "上传管理",
  "运行日志",
  "系统配置",
];
async function api(path: string, method = "GET", body?: any) {
  const headers: Record<string, string> = {};
  const token = sessionStorage.getItem("hc-token");
  if (token) headers.Authorization = "Bearer " + token;
  if (body !== undefined && !(body instanceof FormData))
    headers["Content-Type"] = "application/json";
  const response = await fetch(path, {
    method,
    headers,
    body:
      body === undefined
        ? undefined
        : body instanceof FormData
          ? body
          : JSON.stringify(body),
  });
  const value = await response.json();
  if (!response.ok)
    throw new Error(
      value.message || value.detail || value.code || response.statusText,
    );
  return value;
}
function Json({ value }: { value: any }) {
  return <pre>{JSON.stringify(value, null, 2)}</pre>;
}
function Badge({ value }: { value: any }) {
  return (
    <span
      className={
        "badge " +
        (["ready", "READY", "SUCCEEDED", "running", true].includes(value)
          ? "good"
          : ["fault", "FAILED", "stopped", false].includes(value)
            ? "warn"
            : "")
      }
    >
      {String(value ?? "等待")}
    </span>
  );
}

function App() {
  const [tab, setTab] = useState(0),
    [status, setStatus] = useState<J>({}),
    [online, setOnline] = useState(false),
    [error, setError] = useState(""),
    [notice, setNotice] = useState("");
  const [config, setConfig] = useState<J>({}),
    [info, setInfo] = useState<J>({}),
    [topics, setTopics] = useState<J>({}),
    [datasets, setDatasets] = useState<J[]>([]),
    [datasetPage, setDatasetPage] = useState(0),
    [jobs, setJobs] = useState<J[]>([]),
    [robots, setRobots] = useState<J[]>([]),
    [logs, setLogs] = useState<J[]>([]),
    [cameras, setCameras] = useState<J>({});
  const [mode, setMode] = useState("teleop"),
    [dagger, setDagger] = useState(false),
    [recordAuto, setRecordAuto] = useState(false),
    [recordMode, setRecordMode] = useState("corrections"),
    [format, setFormat] = useState("mcap"),
    [task, setTask] = useState(""),
    [showModel, setShowModel] = useState(false),
    [selected, setSelected] = useState<string[]>([]),
    [cameraSelected, setCameraSelected] = useState<string[]>([]),
    [filter, setFilter] = useState(""),
    [datasetFilter, setDatasetFilter] = useState(""),
    [level, setLevel] = useState(""),
    [detail, setDetail] = useState<any>(null),
    [busy, setBusy] = useState(false);
  const [convertOptionsDraft, setConvertOptionsDraft] = useState("{}");
  const daggerSynced = useRef(false);
  const cameraSyncedHash = useRef("");
  const [topicDraft, setTopicDraft] = useState(
    '{"topic":"/external/example","type":"std_msgs/msg/String","enabled":true,"outputs":["record","websocket"],"max_hz":30,"event_max_hz":10,"qos_reliability":"best_effort"}',
  );
  const [platformDraft, setPlatformDraft] = useState(
    '{"base_url":"https://platform.example","credential_ref":"env:HC_UPLOAD_TOKEN"}',
  );
  const [taskId, setTaskId] = useState(""),
    [platformTasks, setPlatformTasks] = useState<any[]>([]),
    [platformState, setPlatformState] = useState<any>(null);
  const [configDraft, setConfigDraft] = useState("{}"),
    [filename, setFilename] = useState("capture.mcap"),
    [saveDir, setSaveDir] = useState(""),
    [fps, setFps] = useState(30),
    [uploadFormat, setUploadFormat] = useState("mcap"),
    [autoUpload, setAutoUpload] = useState(false),
    [replay, setReplay] = useState<J>({}),
    [replaySpeed, setReplaySpeed] = useState(1),
    [replayLoop, setReplayLoop] = useState(false),
    [logSession, setLogSession] = useState(""),
    [logSessions, setLogSessions] = useState<J[]>([]),
    [subsystem, setSubsystem] = useState("");
  const control = status.control || {};
  const run = async (action: () => Promise<any>, message = "操作已提交") => {
    setBusy(true);
    setError("");
    try {
      const result = await action();
      setNotice(message);
      return result;
    } catch (e) {
      setError(String(e));
      return undefined;
    } finally {
      setBusy(false);
    }
  };
  const refresh = async () => {
    if (tab === 0) {
      setInfo(await api("/api/robot/info"));
      const [cams, camStatus] = await Promise.all([
        api("/api/cameras"),
        api("/api/camera/status").catch(() => null),
      ]);
      setCameras(cams);
      if (
        camStatus?.projection?.camera_ids &&
        cameraSyncedHash.current !== (config?.config_hash || status?.config_hash || "init")
      ) {
        cameraSyncedHash.current = config?.config_hash || status?.config_hash || "init";
        setCameraSelected(camStatus.projection.camera_ids);
      }
    }
    if (tab === 1) setTopics(await api("/api/topics"));
    if (tab === 2) {
      setDatasets(
        await api(`/api/datasets?limit=100&offset=${datasetPage * 100}`),
      );
      setReplay(await api("/api/replay/status"));
    }
    if (tab === 3) {
      setJobs(await api("/api/jobs"));
      setPlatformState(await api("/api/platform/status"));
    }
    if (tab === 4) {
      setLogs(
        await api(
          "/api/logs?" +
            new URLSearchParams({
              q: filter,
              level,
              session: logSession,
              subsystem,
            }),
        ),
      );
      setLogSessions(await api("/api/log-sessions"));
    }
    if (tab === 5) {
      setRobots(await api("/api/robots"));
      setInfo(await api("/api/robot/info"));
      setConfig((await api("/api/config")).applied);
    }
  };
  useEffect(() => {
    api("/api/config")
      .then((v) => setConfig(v.applied))
      .catch((e) => setError(String(e)));
    let socket: WebSocket,
      timer: number,
      closed = false;
    const connect = () => {
      socket = new WebSocket(
        `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}/api/ws`,
      );
      socket.onopen = () => setOnline(true);
      socket.onmessage = (e) => {
        const next = JSON.parse(e.data);
        setStatus(next);
        if (!daggerSynced.current && next?.dagger_recording) {
          daggerSynced.current = true;
          setRecordAuto(Boolean(next.dagger_recording.enabled));
          if (next.dagger_recording.mode) setRecordMode(next.dagger_recording.mode);
        }
      };
      socket.onerror = () => socket.close();
      socket.onclose = () => {
        setOnline(false);
        if (!closed) timer = window.setTimeout(connect, 1500);
      };
    };
    connect();
    return () => {
      closed = true;
      clearTimeout(timer);
      socket?.close();
    };
  }, []);
  useEffect(() => {
    refresh().catch((e) => setError(String(e)));
    const timer = setInterval(() => refresh().catch(() => {}), 3000);
    return () => clearInterval(timer);
  }, [tab, filter, level, logSession, subsystem, datasetPage]);
  const operate = (op: string, extra: J = {}) =>
    run(() =>
      api("/api/control/operations", "POST", {
        op,
        request_id: crypto.randomUUID(),
        expected_boot_id: control.boot_id,
        expected_epoch: control.epoch,
        ...extra,
      }),
    );
  const recordSpec = () => ({
    format,
    filename,
    save_subdirectory: saveDir,
    task_description: task,
    dataset_fps: fps,
    upload_format: uploadFormat,
    auto_upload: autoUpload,
    platform_task_id: taskId || undefined,
  });
  const record = () =>
    run(
      () =>
        api("/api/recordings", "POST", {
          ...recordSpec(),
          request_id: crypto.randomUUID(),
        }),
      "录制已启动",
    );
  const downloadBatch = async () => {
    const token = sessionStorage.getItem("hc-token");
    const response = await fetch("/api/datasets/download", {
      method: "POST",
      headers: {
        "Content-Type": "application/json",
        ...(token ? { Authorization: "Bearer " + token } : {}),
      },
      body: JSON.stringify({ artifact_ids: selected }),
    });
    if (!response.ok) throw new Error(await response.text());
    const url = URL.createObjectURL(await response.blob());
    const link = document.createElement("a");
    link.href = url;
    link.download = "datasets.tar";
    link.click();
    setTimeout(() => URL.revokeObjectURL(url), 30000);
  };
  const batchAction = async (action: (id: string) => Promise<any>) => {
    const results = [];
    for (const id of selected) {
      try {
        results.push({ artifact_id: id, ok: true, result: await action(id) });
      } catch (error) {
        results.push({ artifact_id: id, ok: false, error: String(error) });
      }
    }
    setDetail({ results });
    setSelected(
      results
        .filter((result) => !result.ok)
        .map((result) => result.artifact_id),
    );
    await refresh();
    return results;
  };
  const toggleSelected = (id: string) =>
    setSelected((old) =>
      old.includes(id) ? old.filter((v) => v !== id) : [...old, id],
    );
  const toggleCamera = (id: string) =>
    setCameraSelected((old) =>
      old.includes(id) ? old.filter((v) => v !== id) : [...old, id],
    );
  return (
    <div className="layout">
      <aside>
        <div className="brand">
          <b>HC</b>
          <div>
            ROBOT PLATFORM<small>机器人运行控制台</small>
          </div>
        </div>
        <nav>
          {tabs.map((name, i) => (
            <button
              key={name}
              className={tab === i ? "active" : ""}
              onClick={() => {
                setTab(i);
                setDetail(null);
                setSelected([]);
              }}
            >
              <span>{String(i + 1).padStart(2, "0")}</span>
              {name}
            </button>
          ))}
        </nav>
        <div className="aside-bottom">
          <Badge value={online ? "running" : "disconnected"} />
          <p>
            {status.robot_id || "加载机型…"} · {status.runtime_mode || "—"}
          </p>
          <small>
            ROS Domain {status.ros_domain_id ?? "—"}
            <br />
            配置 {String(status.config_hash || "").slice(0, 12)}
          </small>
        </div>
      </aside>
      <main>
        <header>
          <div>
            <span className="eyebrow">
              HC / {status.robot_id?.toUpperCase() || "ROBOT"}
            </span>
            <h1>{tabs[tab]}</h1>
          </div>
          <div className="header-actions">
            <span>
              当前控制 <strong>{control.owner || "none"}</strong>
            </span>
            <button className="stop" onClick={() => operate("stop")}>
              软件停止
            </button>
          </div>
        </header>
        {error && (
          <div className="alert error" role="alert">
            {error}
            <button onClick={() => setError("")}>关闭</button>
          </div>
        )}
        {notice && (
          <div className="alert">
            {notice}
            <button onClick={() => setNotice("")}>关闭</button>
          </div>
        )}
        {tab === 0 && (
          <>
            <div className="stats">
              {[
                ["控制状态", control.safety],
                ["控制来源", control.owner],
                ["会话模式", control.mode],
                ["VR 连接", status.vr?.is_locked ?? status.vr?.locked],
                ["控制世代", control.epoch],
                ["录制状态", status.recording?.active_session_state || "idle"],
              ].map(([label, value]) => (
                <div className="stat" key={label as string}>
                  <small>{label}</small>
                  <Badge value={value} />
                </div>
              ))}
            </div>
            <section>
              <div className="section-head">
                <h2>控制会话</h2>
                <span>
                  {control.transition_pending
                    ? "切换等待确认"
                    : control.reason || "等待操作"}
                </span>
              </div>
              <div className="row">
                <button onClick={() => operate("recover")} disabled={busy}>
                  故障复位／恢复许可
                </button>
                <select value={mode} onChange={(e) => setMode(e.target.value)}>
                  <option value="teleop">VR 人工遥操</option>
                  <option value="program">程序自动控制</option>
                  <option value="exo">外骨骼</option>
                </select>
                <label>
                  <input
                    type="checkbox"
                    checked={dagger}
                    onChange={(e) => setDagger(e.target.checked)}
                  />
                  允许 DAgger 接管
                </label>
                <button
                  className="primary"
                  disabled={
                    busy ||
                    control.safety !== "ready" ||
                    control.mode !== "idle"
                  }
                  onClick={() =>
                    operate("session_start", { mode, dagger_enabled: dagger })
                  }
                >
                  开始会话
                </button>
                <button onClick={() => operate("pause")}>暂停</button>
                <button onClick={() => operate("resume")}>继续</button>
                <button onClick={() => operate("session_end")}>结束会话</button>
                <button onClick={() => operate("home")}>回初始姿态</button>
                <button onClick={() => operate("reference_reset")}>
                  重建参考
                </button>
              </div>
              <div className="row subdued">
                <label>
                  <input
                    type="checkbox"
                    checked={recordAuto}
                    onChange={(e) => setRecordAuto(e.target.checked)}
                  />
                  自动录制 DAgger
                </label>
                <select
                  value={recordMode}
                  onChange={(e) => setRecordMode(e.target.value)}
                >
                  <option value="corrections">仅人工纠正</option>
                  <option value="full">完整自动与人工会话</option>
                </select>
                <button
                  onClick={() =>
                    run(() =>
                      api("/api/dagger/recording", "PUT", {
                        enabled: recordAuto,
                        mode: recordMode,
                        required: false,
                      }),
                    )
                  }
                >
                  保存会话录制策略
                </button>
                <small>人工接管范围为双臂和支持的夹爪；身体保持。</small>
              </div>
            </section>
            <div className="columns">
              <section>
                <div className="section-head">
                  <h2>关节反馈与命令</h2>
                  <button onClick={() => setShowModel(!showModel)}>
                    {showModel ? "关闭三维" : "显示三维"}
                  </button>
                </div>
                {showModel && (
                  <Suspense fallback={<p>正在加载三维视图…</p>}>
                    <RobotView config={config} control={control} />
                  </Suspense>
                )}
                <div className="scroll">
                  <table>
                    <thead>
                      <tr>
                        <th>关节</th>
                        <th>实际 / SI</th>
                        <th>目标 / SI</th>
                        <th>误差</th>
                      </tr>
                    </thead>
                    <tbody>
                      {(control.joint_names || config.joint_names || []).map(
                        (name: string, i: number) => {
                          const f = (control.feedback_positions ||
                              control.positions ||
                              control.feedback?.positions ||
                              [])[i],
                            t = (control.command_positions ||
                              control.commanded_positions ||
                              [])[i];
                          return (
                            <tr key={name}>
                              <td>{name}</td>
                              <td>
                                {Number.isFinite(f) ? f.toFixed(4) : "无反馈"}
                              </td>
                              <td>{Number.isFinite(t) ? t.toFixed(4) : "—"}</td>
                              <td>
                                {Number.isFinite(f) && Number.isFinite(t)
                                  ? (t - f).toFixed(4)
                                  : "—"}
                              </td>
                            </tr>
                          );
                        },
                      )}
                    </tbody>
                  </table>
                </div>
              </section>
              <section>
                <h2>运行进程</h2>
                {status.processes?.map((p: J) => (
                  <div className="process" key={p.name}>
                    <b>{p.name}</b>
                    <span>PID {p.pid}</span>
                    <Badge
                      value={p.running ? "running" : `exit ${p.exit_code}`}
                    />
                  </div>
                ))}
                <details>
                  <summary>控制与适配器诊断</summary>
                  <Json
                    value={{
                      control,
                      backend: status.backend,
                      ik: status.ik,
                      vr: status.vr,
                    }}
                  />
                </details>
              </section>
            </div>
            <section>
              <h2>相机投射</h2>
              <div className="row">
                {cameras.streams?.map((s: J) => (
                  <label key={s.id}>
                    <input
                      type="checkbox"
                      checked={cameraSelected.includes(s.id)}
                      disabled={
                        s.enabled === false ||
                        (!cameraSelected.includes(s.id) &&
                          cameraSelected.length >= 4)
                      }
                      onChange={() => toggleCamera(s.id)}
                    />
                    {s.name || s.id} <Badge value={s.source_alive} />
                  </label>
                ))}
                <button
                  disabled={!cameraSelected.length}
                  onClick={() =>
                    run(
                      () =>
                        api("/api/cameras/projection", "PUT", {
                          camera_ids: cameraSelected,
                        }),
                      "头显投射选择已保存",
                    )
                  }
                >
                  应用投射到头显
                </button>
              </div>
              <div className="camera-grid">
                {cameraSelected.map((id) => (
                  <figure key={id}>
                    <img
                      src={"/api/cameras/" + id + "/preview"}
                      alt={id + "实时预览"}
                    />
                    <figcaption>
                      {id}{" "}
                      <a
                        href={"/api/cameras/" + id + "/snapshot"}
                        target="_blank"
                      >
                        图像快照
                      </a>
                    </figcaption>
                  </figure>
                ))}
              </div>
              {cameras.reason && <p className="muted">{cameras.reason}</p>}
              <details>
                <summary>编码、传输与标定状态</summary>
                <Json
                  value={{
                    streams: cameras.streams,
                    configuration_errors: cameras.configuration_errors,
                    fec: cameras.fec,
                  }}
                />
              </details>
            </section>
          </>
        )}
        {tab === 1 && (
          <>
            <section>
              <h2>采集归属与上传</h2>
              <div className="row">
                <input
                  value={taskId}
                  onChange={(e) => setTaskId(e.target.value)}
                  placeholder="平台任务 ID（留空为本地采集）"
                />
                <label>
                  <input
                    type="checkbox"
                    checked={autoUpload}
                    onChange={(e) => setAutoUpload(e.target.checked)}
                  />
                  录制完成后上传
                </label>
                <select
                  value={uploadFormat}
                  onChange={(e) => setUploadFormat(e.target.value)}
                >
                  <option value="mcap">上传 MCAP</option>
                  <option value="lerobot">上传 LeRobot v3</option>
                </select>
              </div>
              <p className="muted">
                任务、采集方案与上传格式在开始时固定。保存根目录：
                {status.recording?.recording_root}
              </p>
              {detail && <Json value={detail} />}
            </section>
            <section>
              <h2>普通示教录制</h2>
              <div className="row">
                <input
                  aria-label="文件名"
                  value={filename}
                  onChange={(e) => setFilename(e.target.value)}
                  placeholder="MCAP 文件名"
                />
                <input
                  aria-label="保存子目录"
                  value={saveDir}
                  onChange={(e) => setSaveDir(e.target.value)}
                  placeholder="保存子目录（数据根目录内）"
                />
                <label>
                  数据集 Hz{" "}
                  <input
                    type="number"
                    min="1"
                    max="120"
                    value={fps}
                    onChange={(e) => setFps(Number(e.target.value))}
                  />
                </label>
                <button
                  onClick={() =>
                    run(async () =>
                      setDetail(
                        await api(
                          "/api/recordings/preflight",
                          "POST",
                          recordSpec(),
                        ),
                      ),
                    )
                  }
                >
                  录制前检查
                </button>
              </div>
              <div className="row">
                <select
                  value={format}
                  onChange={(e) => setFormat(e.target.value)}
                >
                  <option value="mcap">MCAP</option>
                  <option value="mcap_mp4">MCAP＋MP4</option>
                  <option value="lerobot">LeRobot v3</option>
                  <option value="mcap_lerobot">MCAP＋LeRobot</option>
                </select>
                <input
                  value={task}
                  onChange={(e) => setTask(e.target.value)}
                  placeholder="任务描述"
                />
                <button
                  className="primary"
                  disabled={busy || !!status.recording?.active_session_id}
                  onClick={record}
                >
                  开始录制
                </button>
                <button
                  disabled={!status.recording?.active_session_id}
                  onClick={() => run(() => api("/api/recordings/stop", "POST"))}
                >
                  停止并收尾
                </button>
                <button
                  disabled={!status.recording?.active_session_id}
                  onClick={() =>
                    run(() =>
                      api("/api/recordings/mark", "POST", {
                        label: "operator_mark",
                      }),
                    )
                  }
                >
                  打标
                </button>
              </div>
              <p className="muted">
                录制前固定话题、机型、格式与任务。采集标记不代表任务成功。
              </p>
              <Json value={status.recording} />
            </section>
            <section>
              <h2>录制与展示话题</h2>
              <div className="scroll">
                <table>
                  <thead>
                    <tr>
                      <th>录制</th>
                      <th>话题</th>
                      <th>消息类型</th>
                      <th>接收 Hz</th>
                      <th>操作</th>
                    </tr>
                  </thead>
                  <tbody>
                    {topics.configured?.map((t: J) => (
                      <tr key={t.topic}>
                        <td>
                          <input
                            type="checkbox"
                            checked={
                              t.outputs?.includes("record") &&
                              t.enabled !== false
                            }
                            onChange={(e) =>
                              run(() =>
                                api("/api/topics", "PUT", {
                                  ...t,
                                  enabled: true,
                                  outputs: e.target.checked
                                    ? [
                                        ...new Set([
                                          ...(t.outputs || []),
                                          "record",
                                        ]),
                                      ]
                                    : t.outputs.filter(
                                        (v: string) => v !== "record",
                                      ),
                                }),
                              )
                            }
                          />
                        </td>
                        <td>{t.topic}</td>
                        <td>{t.type}</td>
                        <td>
                          {topics.metrics?.[t.topic]?.fps?.toFixed(1) || "—"}{" "}
                          <Badge value={topics.metrics?.[t.topic]?.state} />
                        </td>
                        <td>
                          <button
                            onClick={() =>
                              setTopicDraft(JSON.stringify(t, null, 2))
                            }
                          >
                            编辑
                          </button>
                          <button
                            onClick={() =>
                              run(() =>
                                api(
                                  "/api/topics?topic=" +
                                    encodeURIComponent(t.topic),
                                  "DELETE",
                                ),
                              )
                            }
                          >
                            删除
                          </button>
                        </td>
                      </tr>
                    ))}
                  </tbody>
                </table>
              </div>
              <details>
                <summary>发现的话题</summary>
                <Json value={topics.discovered} />
              </details>
            </section>
            <section>
              <h2>添加／编辑话题</h2>
              <button
                onClick={() =>
                  run(async () => setDetail(await api("/api/topics/catalog")))
                }
              >
                查看标准话题目录
              </button>
              <textarea
                rows={8}
                value={topicDraft}
                onChange={(e) => setTopicDraft(e.target.value)}
              />
              <button
                onClick={() =>
                  run(() => api("/api/topics", "PUT", JSON.parse(topicDraft)))
                }
              >
                校验并保存
              </button>
            </section>
          </>
        )}
        {tab === 2 && (
          <>
            <section>
              <div className="section-head">
                <h2>本地数据集</h2>
                <div className="row">
                  <input
                    placeholder="筛选当前页（名称／格式）"
                    value={datasetFilter}
                    onChange={(e) => setDatasetFilter(e.target.value)}
                  />
                  <input
                    placeholder="上传任务 ID（批量上传用）"
                    value={taskId}
                    onChange={(e) => setTaskId(e.target.value)}
                  />
                  <input
                    placeholder='转换选项 JSON（可选，如 {"fps":30}）'
                    value={convertOptionsDraft}
                    onChange={(e) => setConvertOptionsDraft(e.target.value)}
                  />
                  <label className="button">
                    导入 MCAP
                    <input
                      type="file"
                      accept=".mcap"
                      hidden
                      onChange={(e) => {
                        const file = e.target.files?.[0];
                        if (file) {
                          const form = new FormData();
                          form.append("file", file);
                          run(() => api("/api/datasets/import", "POST", form));
                        }
                      }}
                    />
                  </label>
                </div>
              </div>
              <div className="row">
                <button
                  disabled={datasetPage === 0}
                  onClick={() => {
                    setDatasetPage(datasetPage - 1);
                    setSelected([]);
                  }}
                >
                  上一页
                </button>
                <span>第 {datasetPage + 1} 页 · 每页 100 条</span>
                <button
                  disabled={datasets.length < 100}
                  onClick={() => {
                    setDatasetPage(datasetPage + 1);
                    setSelected([]);
                  }}
                >
                  下一页
                </button>
                <button
                  disabled={!selected.length}
                  onClick={() => run(downloadBatch)}
                >
                  批量打包下载
                </button>
                <button
                  disabled={!selected.length}
                  onClick={() =>
                    run(() => {
                      const options = JSON.parse(
                        convertOptionsDraft.trim() || "{}",
                      );
                      return batchAction((id) =>
                        api(`/api/datasets/${id}/convert`, "POST", {
                          request_id: crypto.randomUUID(),
                          ...(Object.keys(options).length ? { options } : {}),
                        }),
                      );
                    })
                  }
                >
                  批量转换
                </button>
                <button
                  disabled={!selected.length}
                  onClick={() =>
                    run(() =>
                      batchAction((id) =>
                        api(`/api/datasets/${id}/upload`, "POST", {
                          request_id: crypto.randomUUID(),
                          target: { task_id: taskId },
                        }),
                      ),
                    )
                  }
                >
                  批量上传
                </button>
                <button
                  disabled={!selected.length}
                  onClick={() => {
                    if (
                      confirm(
                        "删除所选本地数据集？正在使用的产物会被拒绝删除。",
                      )
                    )
                      run(() =>
                        batchAction((id) =>
                          api(`/api/datasets/${id}`, "DELETE"),
                        ),
                      );
                  }}
                >
                  批量删除
                </button>
              </div>
              <div className="scroll">
                <table>
                  <thead>
                    <tr>
                      <th></th>
                      <th>数据集</th>
                      <th>格式</th>
                      <th>状态</th>
                      <th>机器人／时长</th>
                      <th>消息／帧／相机</th>
                      <th>大小</th>
                      <th>操作</th>
                    </tr>
                  </thead>
                  <tbody>
                    {datasets
                      .filter((d) =>
                        JSON.stringify(d)
                          .toLowerCase()
                          .includes(datasetFilter.toLowerCase()),
                      )
                      .map((d) => (
                        <tr key={d.id}>
                          <td>
                            <input
                              type="checkbox"
                              checked={selected.includes(d.id)}
                              onChange={() => toggleSelected(d.id)}
                            />
                          </td>
                          <td>
                            <code>{d.id.slice(0, 12)}</code>
                          </td>
                          <td>{d.format}</td>
                          <td>
                            <Badge value={d.state} />
                          </td>
                          <td>
                            {d.summary?.robot_id || "未知"} /{" "}
                            {(d.summary?.duration_s || 0).toFixed(1)} s
                          </td>
                          <td>
                            {d.summary?.message_count ?? "—"} /{" "}
                            {d.summary?.frame_count ?? "—"} /{" "}
                            {d.summary?.camera_count ?? "—"}
                          </td>
                          <td>
                            {((d.summary?.size_bytes || 0) / 1048576).toFixed(
                              1,
                            )}{" "}
                            MB
                          </td>
                          <td>
                            <button
                              onClick={() =>
                                run(async () =>
                                  setDetail(await api("/api/datasets/" + d.id)),
                                )
                              }
                            >
                              详情
                            </button>
                            <a
                              className="button"
                              href={`/api/datasets/${d.id}/download`}
                            >
                              下载
                            </a>
                            <button
                              onClick={() =>
                                run(() =>
                                  api(`/api/replay/start`, "POST", {
                                    artifact_id: d.id,
                                    mode: "observe",
                                    speed: replaySpeed,
                                    loop: replayLoop,
                                  }),
                                )
                              }
                            >
                              观测回放
                            </button>
                            <button
                              onClick={() =>
                                run(() =>
                                  api("/api/replay/start", "POST", {
                                    artifact_id: d.id,
                                    mode: "drive",
                                    speed: replaySpeed,
                                    loop: replayLoop,
                                  }),
                                )
                              }
                            >
                              驱动回放
                            </button>
                          </td>
                        </tr>
                      ))}
                  </tbody>
                </table>
              </div>
            </section>
            {detail && (
              <section>
                <h2>数据集详情</h2>
                {detail.video_assets?.map((path: string) => (
                  <video
                    key={path}
                    controls
                    preload="metadata"
                    src={`/api/datasets/${detail.id}/asset?path=${encodeURIComponent(path)}`}
                  />
                ))}
                <Json value={detail} />
              </section>
            )}
            <section>
              <h2>
                回放 <Badge value={replay.phase} />
              </h2>
              <div className="row">
                <label>
                  倍速{" "}
                  <input
                    type="number"
                    min="0.1"
                    max="4"
                    step="0.1"
                    value={replaySpeed}
                    onChange={(e) => setReplaySpeed(Number(e.target.value))}
                  />
                </label>
                <label>
                  <input
                    type="checkbox"
                    checked={replayLoop}
                    onChange={(e) => setReplayLoop(e.target.checked)}
                  />
                  循环
                </label>
                <button
                  onClick={() =>
                    run(() =>
                      api("/api/replay/speed", "POST", { speed: replaySpeed }),
                    )
                  }
                >
                  应用倍速
                </button>
                {["pause", "resume", "stop"].map((action) => (
                  <button
                    key={action}
                    onClick={() =>
                      run(() => api("/api/replay/" + action, "POST"))
                    }
                  >
                    {action}
                  </button>
                ))}
              </div>
              <p>
                {(replay.elapsed_s || 0).toFixed(1)} /{" "}
                {(replay.duration_s || 0).toFixed(1)} s · {replay.messages || 0}{" "}
                / {replay.total_messages || 0} 条
              </p>
              {replay.error && <Json value={replay.error} />}
              <p className="muted">
                驱动回放通过同一控制出口；结束后需要明确恢复。
              </p>
            </section>
          </>
        )}
        {tab === 3 && (
          <>
            <section>
              <h2>平台连接与采集任务</h2>
              <textarea
                rows={4}
                value={platformDraft}
                onChange={(e) => setPlatformDraft(e.target.value)}
              />
              <div className="row">
                <button
                  onClick={() =>
                    run(() =>
                      api(
                        "/api/platform/config",
                        "PUT",
                        JSON.parse(platformDraft),
                      ),
                    )
                  }
                >
                  保存连接
                </button>
                <input
                  value={taskId}
                  onChange={(e) => setTaskId(e.target.value)}
                  placeholder="目标任务 ID"
                />
                <button
                  onClick={() =>
                    run(async () => {
                      const value = await api(
                        "/api/platform/tasks?q=" + encodeURIComponent(taskId),
                      );
                      setPlatformTasks(
                        Array.isArray(value)
                          ? value
                          : value.items || value.tasks || [],
                      );
                    })
                  }
                >
                  搜索任务
                </button>
              </div>
              {platformTasks.map((t: J) => (
                <button
                  key={t.collection_task_id || t.id}
                  onClick={() => setTaskId(t.collection_task_id || t.id)}
                >
                  {t.name || t.collection_task_id || t.id}
                </button>
              ))}
              <details>
                <summary>连接状态</summary>
                <Json value={platformState} />
              </details>
            </section>
            <section>
              <h2>持久作业队列</h2>
              <table>
                <thead>
                  <tr>
                    <th>作业</th>
                    <th>类型</th>
                    <th>状态</th>
                    <th>进度／错误</th>
                    <th>操作</th>
                  </tr>
                </thead>
                <tbody>
                  {jobs.map((j) => (
                    <tr key={j.id}>
                      <td>
                        <code>{j.id.slice(0, 12)}</code>
                      </td>
                      <td>{j.kind}</td>
                      <td>
                        <Badge value={j.state} />
                      </td>
                      <td>
                        {JSON.stringify(
                          (j.result && Object.keys(j.result).length
                            ? j.result
                            : j.error) || {},
                        )}
                      </td>
                      <td>
                        <button
                          onClick={() =>
                            run(() => api(`/api/jobs/${j.id}/cancel`, "POST"))
                          }
                        >
                          取消
                        </button>
                        <button
                          onClick={() =>
                            run(() => api(`/api/jobs/${j.id}/retry`, "POST"))
                          }
                        >
                          重试
                        </button>
                        <button onClick={() => setDetail(j)}>详情</button>
                        {j.kind === "upload" && (
                          <button
                            onClick={() =>
                              run(() =>
                                api(`/api/jobs/${j.id}/refresh`, "POST"),
                              )
                            }
                          >
                            更新平台处理状态
                          </button>
                        )}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
              {detail && <Json value={detail} />}
            </section>
          </>
        )}
        {tab === 4 && (
          <section>
            <div className="section-head">
              <h2>运行日志</h2>
              <a
                className="button"
                href={
                  "/api/logs/export?" +
                  new URLSearchParams({
                    session: logSession,
                    subsystem,
                    q: filter,
                    level,
                  })
                }
              >
                导出 JSONL
              </a>
            </div>
            <div className="row">
              <input
                value={filter}
                onChange={(e) => setFilter(e.target.value)}
                placeholder="关键词"
              />
              <select value={level} onChange={(e) => setLevel(e.target.value)}>
                <option value="">全部等级</option>
                <option>INFO</option>
                <option>ERROR</option>
                <option>WARN</option>
              </select>
            </div>
            <div className="row">
              <select
                aria-label="日志会话"
                value={logSession}
                onChange={(e) => setLogSession(e.target.value)}
              >
                <option value="">当前会话</option>
                {logSessions.map((s) => (
                  <option key={s.id}>{s.id}</option>
                ))}
              </select>
              <input
                value={subsystem}
                onChange={(e) => setSubsystem(e.target.value)}
                placeholder="子系统"
              />
            </div>
            <div className="logs">
              {logs.map((l) => (
                <div key={l.seq}>
                  <time>{new Date(l.time * 1000).toLocaleTimeString()}</time>
                  <b>{l.level}</b>
                  <span>
                    [{l.subsystem}] {l.message}
                  </span>
                </div>
              ))}
            </div>
          </section>
        )}
        {tab === 5 && (
          <>
            <section>
              <h2>连接授权</h2>
              <div className="row">
                <input
                  type="password"
                  placeholder="远程操作令牌（仅保存于当前浏览器会话）"
                  onChange={(e) =>
                    sessionStorage.setItem("hc-token", e.target.value)
                  }
                />
                <span className="muted">
                  本机控制默认可用；远程控制需 HC_PLATFORM_TOKEN。
                </span>
              </div>
            </section>
            <section>
              <h2>机器人配置包</h2>
              <div className="cards">
                {robots.map((r) => (
                  <article key={r.id}>
                    <h3>{r.id}</h3>
                    <Badge value={r.valid} />
                    <p>真机验证：{r.verification?.hardware || "未验证"}</p>
                    {r.errors && (
                      <p className="error-text">{r.errors.join("; ")}</p>
                    )}
                    <button
                      disabled={!r.valid || busy}
                      onClick={() =>
                        run(async () => {
                          const result = await api(
                            `/api/robots/${r.id}/apply`,
                            "POST",
                          );
                          setConfig((await api("/api/config")).applied);
                          return result;
                        })
                      }
                    >
                      应用并保持停止
                    </button>
                    <a className="button" href={`/api/robots/${r.id}/export`}>
                      导出
                    </a>
                    {!r.builtin && (
                      <button
                        onClick={() =>
                          run(() => api(`/api/robots/${r.id}`, "DELETE"))
                        }
                      >
                        删除
                      </button>
                    )}
                  </article>
                ))}
              </div>
              <label className="button">
                导入配置 ZIP
                <input
                  type="file"
                  accept=".zip"
                  hidden
                  onChange={(e) => {
                    const file = e.target.files?.[0];
                    if (file) {
                      const id = prompt("新机型 ID（已有配置请使用新版本 ID）");
                      if (id) {
                        const form = new FormData();
                        form.append("file", file);
                        run(() =>
                          api(
                            `/api/robots/${encodeURIComponent(id)}/import`,
                            "POST",
                            form,
                          ),
                        );
                      }
                    }
                  }}
                />
              </label>
            </section>
            <section>
              <h2>修改运行配置</h2>
              <p className="muted">
                填写覆盖配置
                JSON。校验通过后会停止控制，重启相关进程并保存；需要明确恢复。URDF、关节拓扑与手部标定通过新配置包版本管理。
              </p>
              <textarea
                aria-label="运行配置 JSON"
                rows={12}
                value={configDraft}
                onChange={(e) => setConfigDraft(e.target.value)}
              />
              <div className="row">
                <button
                  onClick={() =>
                    run(async () =>
                      setConfigDraft(
                        JSON.stringify(
                          (await api("/api/config")).overrides,
                          null,
                          2,
                        ),
                      ),
                    )
                  }
                >
                  读取已保存配置
                </button>
                <button
                  onClick={() =>
                    run(async () =>
                      setDetail(
                        await api(
                          "/api/config/validate",
                          "POST",
                          JSON.parse(configDraft),
                        ),
                      ),
                    )
                  }
                >
                  仅校验
                </button>
                <button
                  className="primary"
                  onClick={() =>
                    run(async () => {
                      const value = await api(
                        "/api/config",
                        "PUT",
                        JSON.parse(configDraft),
                      );
                      setConfig((await api("/api/config")).applied);
                      return value;
                    })
                  }
                >
                  停止并应用配置
                </button>
                <button
                  onClick={() =>
                    run(async () => setDetail(await api("/api/diagnostics")))
                  }
                >
                  系统诊断
                </button>
              </div>
              {detail && <Json value={detail} />}
            </section>
            <section>
              <h2>运行配置与能力</h2>
              <p>
                {status.runtime_mode} · ROS Domain {status.ros_domain_id} · 配置{" "}
                {status.config_hash}
              </p>
              <div className="row">
                <button
                  onClick={() => run(() => api("/api/runtime/restart", "POST"))}
                >
                  停止并重启控制组件
                </button>
                {status.runtime_mode === "sim" && (
                  <>
                    <button
                      onClick={() =>
                        run(() => api("/api/runtime/simulation/stop", "POST"))
                      }
                    >
                      停止仿真
                    </button>
                    <button
                      onClick={() =>
                        run(() => api("/api/runtime/simulation/start", "POST"))
                      }
                    >
                      启动仿真
                    </button>
                  </>
                )}
              </div>
              <table>
                <thead>
                  <tr>
                    <th>能力</th>
                    <th>已配置</th>
                    <th>后端支持</th>
                    <th>当前就绪</th>
                    <th>原因</th>
                  </tr>
                </thead>
                <tbody>
                  {Object.entries(info.capabilities || {}).map(
                    ([name, value]: [string, any]) => (
                      <tr key={name}>
                        <td>{name}</td>
                        <td>
                          <Badge value={value.configured} />
                        </td>
                        <td>
                          <Badge value={value.backend_supported} />
                        </td>
                        <td>
                          <Badge value={value.runtime_ready} />
                        </td>
                        <td>{value.blocking_reasons?.join(", ")}</td>
                      </tr>
                    ),
                  )}
                </tbody>
              </table>
              <details>
                <summary>完整已应用配置</summary>
                <Json value={config} />
              </details>
            </section>
          </>
        )}
        <footer>软件控制保护 · 物理急停与底层执行保护由硬件系统提供</footer>
      </main>
    </div>
  );
}
createRoot(document.getElementById("root")!).render(<App />);
