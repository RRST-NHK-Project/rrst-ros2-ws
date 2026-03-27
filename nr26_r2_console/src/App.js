import React, { useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";
import "./App.css";

const PACKET_INDEX_LABELS = [
  "DEBUG",
  "MD1", "MD2", "MD3", "MD4", "MD5", "MD6", "MD7", "MD8",
  "SERVO1", "SERVO2", "SERVO3", "SERVO4", "SERVO5", "SERVO6", "SERVO7", "SERVO8",
  "TR1", "TR2", "TR3", "TR4", "TR5", "TR6", "TR7",
];

const DEFAULT_PACKET_COUNT = 24;
const SERIAL_BRIDGE_MIN_ELEMENTS = 24;

const describeActuator = (label) => {
  if (label === "DEBUG") return "デバッグ (0/1)";
  if (label.startsWith("MD")) return "モータ出力 (-255〜255)";
  if (label.startsWith("SERVO")) return "サーボ角度 (0〜270)";
  if (label.startsWith("TR")) return "デジタル出力 (0/1)";
  return "予備";
};

function App() {
  const rosRef = useRef(null);
  const commandRef = useRef(null);
  const joyRef = useRef(null);
  const odomRef = useRef(null);
  const autoDriveCmdRef = useRef(null);
  const odomResetCmdRef = useRef(null);
  const rosTopicsServiceRef = useRef(null);
  const rosTopicTypeServiceRef = useRef(null);
  const topicEchoSubRef = useRef(null);
  const serialPeriodicTimerRef = useRef(null);
  const serialBridgeLogBoxRef = useRef(null);
  const defaultRosHost = window.location.hostname || "localhost";
  const wsScheme = window.location.protocol === "https:" ? "wss" : "ws";
  const commandValueRef = useRef(0);
  const buttonsRef = useRef(Array(14).fill(0));
  const axesRef = useRef(Array(8).fill(0));

  const [status, setStatus] = useState("接続中...");
  const [rosHostInput, setRosHostInput] = useState(defaultRosHost);
  const [rosPortInput, setRosPortInput] = useState("9090");
  const [rosEndpoint, setRosEndpoint] = useState({
    host: defaultRosHost,
    port: "9090",
  });
  const [commandValue, setCommandValue] = useState(0);
  const [buttons, setButtons] = useState(Array(14).fill(0));
  const [axes, setAxes] = useState(Array(8).fill(0));
  const [operationArmed, setOperationArmed] = useState(false);
  const [frontendForceStopped, setFrontendForceStopped] = useState(false);
  const [controllerEnabled, setControllerEnabled] = useState(false);
  const [controllerFullscreen, setControllerFullscreen] = useState(false);
  const [activePage, setActivePage] = useState("controller");
  const [joyTopicName, setJoyTopicName] = useState("/joy_9");
  const [joyTopicInput, setJoyTopicInput] = useState("/joy_9");
  const [serialTargetIdInput, setSerialTargetIdInput] = useState("1");
  const [serialElementCount, setSerialElementCount] = useState(DEFAULT_PACKET_COUNT);
  const [serialValues, setSerialValues] = useState(Array(DEFAULT_PACKET_COUNT).fill(0));
  const [serialPublishInfo, setSerialPublishInfo] = useState("未送信");
  const [serialPeriodicHz, setSerialPeriodicHz] = useState("10");
  const [serialPeriodicEnabled, setSerialPeriodicEnabled] = useState(false);
  const [poseX, setPoseX] = useState(0);
  const [poseY, setPoseY] = useState(0);
  const [poseYaw, setPoseYaw] = useState(0);
  const [targetXInput, setTargetXInput] = useState("0.0");
  const [targetYInput, setTargetYInput] = useState("0.0");
  const [targetYawInput, setTargetYawInput] = useState("0.0");
  const [targetXStep, setTargetXStep] = useState("0.1");
  const [targetYStep, setTargetYStep] = useState("0.1");
  const [targetYawStep, setTargetYawStep] = useState("5");
  const [savedPose, setSavedPose] = useState(null);
  const [savedPosesList, setSavedPosesList] = useState([]);
  const [autoDriveCmdInfo, setAutoDriveCmdInfo] = useState("未送信");
  const [topicList, setTopicList] = useState([]);
  const [topicListLoading, setTopicListLoading] = useState(false);
  const [topicListError, setTopicListError] = useState("");
  const [selectedEchoTopic, setSelectedEchoTopic] = useState("");
  const [selectedEchoType, setSelectedEchoType] = useState("");
  const [topicEchoInfo, setTopicEchoInfo] = useState("未開始");
  const [topicEchoMessages, setTopicEchoMessages] = useState([]);
  const [topicEchoRunning, setTopicEchoRunning] = useState(false);
  const [serialBridgePorts, setSerialBridgePorts] = useState([]);
  const [serialBridgeIds, setSerialBridgeIds] = useState([]);
  const [serialBridgeRunning, setSerialBridgeRunning] = useState(false);
  const [serialBridgePid, setSerialBridgePid] = useState("");
  const [serialBridgeInfo, setSerialBridgeInfo] = useState("未取得");
  const [serialBridgeLoading, setSerialBridgeLoading] = useState(false);
  const [serialBridgeLogs, setSerialBridgeLogs] = useState([]);
  const [serialBridgeLogLinesInput, setSerialBridgeLogLinesInput] = useState("200");
  const [serialBridgeLogLoading, setSerialBridgeLogLoading] = useState(false);
  const [serialBridgeLogRealtimeEnabled, setSerialBridgeLogRealtimeEnabled] = useState(false);

  const backendBaseUrl = `${window.location.protocol}//${window.location.hostname}:3031`;

  const rosUrl = `${wsScheme}://${rosEndpoint.host}:${rosEndpoint.port}`;

  const applyRosEndpoint = () => {
    const nextHost = rosHostInput.trim() || defaultRosHost;
    const nextPort = rosPortInput.trim() || "9090";

    setRosEndpoint({ host: nextHost, port: nextPort });
  };

  const applyJoyTopicName = () => {
    const nextTopic = joyTopicInput.trim() || "/joy_9";
    setJoyTopicName(nextTopic);
    console.log("Joy topic name updated to:", nextTopic);
  };

  const callRosService = (serviceRef, requestData) =>
    new Promise((resolve, reject) => {
      if (!serviceRef.current) {
        reject(new Error("サービスが初期化されていません"));
        return;
      }

      const request = requestData || {};
      serviceRef.current.callService(
        request,
        (response) => resolve(response),
        (error) => reject(error || new Error("サービス呼び出しに失敗しました"))
      );
    });

  const stopTopicEcho = () => {
    if (topicEchoSubRef.current) {
      try {
        topicEchoSubRef.current.unsubscribe();
      } catch (error) {
        console.warn("Error unsubscribing topic echo:", error);
      }
      topicEchoSubRef.current = null;
    }

    setTopicEchoRunning(false);
  };

  const extractSerialBridgeIds = (topics) => {
    const idSet = new Set();
    topics.forEach((item) => {
      const name = item?.name || "";
      const match = name.match(/^serial_(?:rx|tx)_(\d+)$/);
      if (match) {
        idSet.add(Number.parseInt(match[1], 10));
      }
    });

    return Array.from(idSet).sort((a, b) => a - b);
  };

  const addTimestampToLogLines = (lines) => {
    const fetchedAt = new Date().toLocaleTimeString("ja-JP");
    return lines.map((line) => {
      if (/^\[\d{1,2}:\d{2}:\d{2}\]/.test(line)) {
        return line;
      }
      return `[${fetchedAt}] ${line}`;
    });
  };

  const parsedLogLineLimit = Number.parseInt(serialBridgeLogLinesInput, 10);
  const serialBridgeLogLineLimit = Number.isFinite(parsedLogLineLimit)
    ? Math.max(10, Math.min(1000, parsedLogLineLimit))
    : 200;

  const refreshTopicList = async () => {
    if (!rosRef.current || !rosTopicsServiceRef.current) {
      setTopicListError("ROS未接続のため取得できません");
      return [];
    }

    setTopicListLoading(true);
    setTopicListError("");
    try {
      const response = await callRosService(rosTopicsServiceRef, {});
      const topics = Array.isArray(response?.topics) ? response.topics : [];
      const types = Array.isArray(response?.types) ? response.types : [];
      const mapped = topics.map((name, index) => ({
        name,
        type: types[index] || "",
      }));

      mapped.sort((a, b) => a.name.localeCompare(b.name));
      setTopicList(mapped);
      setSerialBridgeIds(extractSerialBridgeIds(mapped));

      if (mapped.length === 0) {
        setTopicListError("トピックが見つかりません");
      }
      return mapped;
    } catch (error) {
      console.error("Failed to fetch topic list:", error);
      setTopicListError("トピック一覧の取得に失敗しました");
      return [];
    } finally {
      setTopicListLoading(false);
    }
  };

  const refreshSerialBridgeStatus = async () => {
    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/status`);
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgePorts(Array.isArray(data?.ports) ? data.ports : []);
      setSerialBridgeRunning(Boolean(data?.running));
      setSerialBridgePid(data?.pid ? String(data.pid) : "");
      setSerialBridgeInfo("状態を更新しました");
    } catch (error) {
      console.error("Failed to fetch serial bridge status:", error);
      setSerialBridgeInfo("状態取得に失敗しました (backend未起動の可能性)");
      setSerialBridgeRunning(false);
      setSerialBridgePid("");
      setSerialBridgePorts([]);
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const refreshSerialBridgeLogs = async () => {
    setSerialBridgeLogLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/logs?lines=${serialBridgeLogLineLimit}`);
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      const lines = Array.isArray(data?.lines) ? data.lines : [];
      setSerialBridgeLogs(addTimestampToLogLines(lines));
    } catch (error) {
      console.error("Failed to fetch serial bridge logs:", error);
      setSerialBridgeLogs(["ログ取得に失敗しました"]);
    } finally {
      setSerialBridgeLogLoading(false);
    }
  };

  const startSerialBridgeFromConsole = async () => {
    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/start`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgeRunning(Boolean(data?.running));
      setSerialBridgePid(data?.pid ? String(data.pid) : "");
      setSerialBridgeInfo(data?.message || "serial_bridge を起動しました");
      await refreshSerialBridgeStatus();
      await refreshTopicList();
      await refreshSerialBridgeLogs();
    } catch (error) {
      console.error("Failed to start serial bridge:", error);
      setSerialBridgeInfo("serial_bridge の起動に失敗しました");
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const applySettingsValues = async () => {
    applyRosEndpoint();
    applyJoyTopicName();
    await refreshSerialBridgeLogs();
    setSerialBridgeInfo("設定を適用しました");
  };

  const stopSerialBridgeFromConsole = async () => {
    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/serial-bridge/stop`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgeRunning(Boolean(data?.running));
      setSerialBridgeInfo(data?.message || "serial_bridge を停止しました");
      await refreshSerialBridgeStatus();
      await refreshTopicList();
      await refreshSerialBridgeLogs();
    } catch (error) {
      console.error("Failed to stop serial bridge:", error);
      setSerialBridgeInfo("serial_bridge の停止に失敗しました");
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const forceShutdownBackendFromConsole = async () => {
    const confirmed = window.confirm(
      "console backend を強制シャットダウンします。\nこの後、状態取得や起動/停止機能は再起動まで使えません。続行しますか？"
    );
    if (!confirmed) {
      return;
    }

    setSerialBridgeLoading(true);
    try {
      const response = await fetch(`${backendBaseUrl}/api/backend/force-shutdown`, {
        method: "POST",
      });
      if (!response.ok) {
        throw new Error(`status ${response.status}`);
      }
      const data = await response.json();
      setSerialBridgeInfo(data?.message || "console backend を強制シャットダウンしました");
      setSerialBridgeRunning(false);
      setSerialBridgePid("");
      setSerialBridgePorts([]);
    } catch (error) {
      console.error("Failed to force shutdown backend:", error);
      setSerialBridgeInfo("console backend の強制シャットダウンに失敗しました");
    } finally {
      setSerialBridgeLoading(false);
    }
  };

  const forceShutdownFrontendFromConsole = () => {
    const confirmed = window.confirm(
      "フロントエンドを強制シャットダウンします。\n画面操作は停止し、再読み込みまで復帰できません。続行しますか？"
    );
    if (!confirmed) {
      return;
    }

    setOperationArmed(false);
    setControllerEnabled(false);
    setSerialPeriodicEnabled(false);
    setSerialBridgeLogRealtimeEnabled(false);
    setActivePage("shutdown");
    resetAllControls();
    stopTopicEcho();
    if (rosRef.current) {
      rosRef.current.close();
    }
    setStatus("切断");
    setFrontendForceStopped(true);
  };

  useEffect(() => {
    if (!serialBridgeLogRealtimeEnabled || activePage !== "serial-bridge") {
      return undefined;
    }

    const pollLogs = async () => {
      setSerialBridgeLogLoading(true);
      try {
        const response = await fetch(`${backendBaseUrl}/api/serial-bridge/logs?lines=${serialBridgeLogLineLimit}`);
        if (!response.ok) {
          throw new Error(`status ${response.status}`);
        }
        const data = await response.json();
        const lines = Array.isArray(data?.lines) ? data.lines : [];
        setSerialBridgeLogs(addTimestampToLogLines(lines));
      } catch (error) {
        console.error("Failed to fetch serial bridge logs:", error);
        setSerialBridgeLogs(["ログ取得に失敗しました"]);
      } finally {
        setSerialBridgeLogLoading(false);
      }
    };

    const timer = setInterval(() => {
      pollLogs();
    }, 1000);

    pollLogs();

    return () => {
      clearInterval(timer);
    };
  }, [serialBridgeLogRealtimeEnabled, activePage, backendBaseUrl, serialBridgeLogLineLimit]);

  useEffect(() => {
    if (activePage !== "serial-bridge") {
      return;
    }

    const logBox = serialBridgeLogBoxRef.current;
    if (!logBox) {
      return;
    }

    logBox.scrollTop = logBox.scrollHeight;
  }, [serialBridgeLogs, serialBridgeLogLoading, activePage]);

  const startTopicEcho = async () => {
    const topicName = selectedEchoTopic.trim();
    if (!topicName) {
      setTopicEchoInfo("トピック名を選択してください");
      return;
    }

    if (!rosRef.current) {
      setTopicEchoInfo("ROS未接続のため開始できません");
      return;
    }

    let topicType = selectedEchoType;
    try {
      if (!topicType) {
        const response = await callRosService(rosTopicTypeServiceRef, { topic: topicName });
        topicType = response?.type || "";
      }
    } catch (error) {
      console.error("Failed to resolve topic type:", error);
    }

    if (!topicType) {
      setTopicEchoInfo("トピック型の取得に失敗しました");
      return;
    }

    stopTopicEcho();
    setTopicEchoMessages([]);

    const echoTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: topicType,
    });

    echoTopic.subscribe((msg) => {
      const payload = JSON.stringify(msg, null, 2);
      const row = {
        id: Date.now() + Math.random(),
        at: new Date().toLocaleTimeString("ja-JP"),
        payload,
      };
      setTopicEchoMessages((prev) => [row, ...prev].slice(0, 30));
    });

    topicEchoSubRef.current = echoTopic;
    setTopicEchoRunning(true);
    setTopicEchoInfo(`${topicName} (${topicType}) を監視中`);
  };

  const serialTopicName = `serial_tx_${Math.max(0, Number.parseInt(serialTargetIdInput, 10) || 0)}`;

  const parseFloatSafe = (value, fallback = 0) => {
    const parsed = Number.parseFloat(value);
    return Number.isFinite(parsed) ? parsed : fallback;
  };

  const updateSerialElementCount = (nextCountRaw) => {
    const parsed = Number.parseInt(nextCountRaw, 10);
    const nextCount = Number.isFinite(parsed)
      ? Math.max(1, Math.min(64, parsed))
      : DEFAULT_PACKET_COUNT;

    setSerialElementCount(nextCount);
    setSerialValues((prev) => {
      const next = [...prev];
      if (next.length < nextCount) {
        while (next.length < nextCount) next.push(0);
      }
      if (next.length > nextCount) {
        next.length = nextCount;
      }
      return next;
    });
  };

  const updateSerialValue = (index, rawValue) => {
    const parsed = Number.parseInt(rawValue, 10);
    const safeValue = Number.isFinite(parsed)
      ? Math.max(-32768, Math.min(32767, parsed))
      : 0;

    setSerialValues((prev) => {
      const next = [...prev];
      next[index] = safeValue;
      return next;
    });
  };

  const renderSerialInputItem = (index) => {
    const label = PACKET_INDEX_LABELS[index] || `CH${index}`;
    const value = serialValues[index] ?? 0;
    return (
      <label className="serial-item" key={`${label}-${index}`}>
        <span className="serial-item-name">[{index}] {label}</span>
        <span className="serial-item-desc">{describeActuator(label)}</span>
        <input
          className="connection-input"
          type="number"
          value={value}
          onChange={(e) => updateSerialValue(index, e.target.value)}
        />
      </label>
    );
  };

  const buildSerialPacketPayload = () => {
    const payload = serialValues.slice(0, serialElementCount);
    const sendPayload = [...payload];
    if (sendPayload.length < SERIAL_BRIDGE_MIN_ELEMENTS) {
      while (sendPayload.length < SERIAL_BRIDGE_MIN_ELEMENTS) {
        sendPayload.push(0);
      }
    }
    return {
      payload,
      sendPayload,
    };
  };

  const publishSerialPacket = (showStatus = true) => {
    if (!operationArmed) {
      if (showStatus) {
        setSerialPublishInfo("操作許可がOFFのため送信できません");
      }
      return false;
    }

    if (!rosRef.current) {
      if (showStatus) {
        setSerialPublishInfo("ROS未接続のため送信できません");
      }
      return false;
    }

    const targetId = Math.max(0, Number.parseInt(serialTargetIdInput, 10) || 0);
    const topicName = `serial_tx_${targetId}`;
    const { payload, sendPayload } = buildSerialPacketPayload();

    const serialTopic = new ROSLIB.Topic({
      ros: rosRef.current,
      name: topicName,
      messageType: "std_msgs/msg/Int16MultiArray",
    });

    serialTopic.publish({ data: sendPayload });
    if (showStatus) {
      setSerialPublishInfo(
        `${topicName} に ${sendPayload.length} 要素を送信 (入力 ${payload.length} 要素)`
      );
    }
    console.log("Serial TX publish:", topicName, sendPayload);
    return true;
  };

  const clearSerialPacket = () => {
    setSerialValues((prev) => prev.map(() => 0));
    setSerialPublishInfo("配列をクリアしました");
  };

  const applyAutoDriveFromCurrentPose = () => {
    setTargetXInput(poseX.toFixed(3));
    setTargetYInput(poseY.toFixed(3));
    setTargetYawInput((poseYaw * 180 / Math.PI).toFixed(1));
  };

  const publishOdomReset = () => {
    if (!operationArmed) {
      setAutoDriveCmdInfo("操作許可がOFFのためオドメトリリセット送信できません");
      return;
    }

    if (!odomResetCmdRef.current) {
      setAutoDriveCmdInfo("ROS未接続のためオドメトリリセット送信できません");
      return;
    }

    odomResetCmdRef.current.publish({
      data: true,
    });
    setAutoDriveCmdInfo("odom_reset にリセット要求を送信しました");
  };

  const saveTargetPose = () => {
    const x = parseFloatSafe(targetXInput);
    const y = parseFloatSafe(targetYInput);
    const yawDeg = parseFloatSafe(targetYawInput);
    const yawRad = yawDeg * Math.PI / 180;

    const newPose = {
      id: Date.now(),
      x: x,
      y: y,
      yaw: yawRad,
      yawDeg: yawDeg,
      timestamp: new Date().toLocaleTimeString('ja-JP'),
      label: `目標${savedPosesList.length + 1}`,
    };

    setSavedPosesList([newPose, ...savedPosesList]);
    setAutoDriveCmdInfo(`目標値を保存しました: (${x.toFixed(3)}, ${y.toFixed(3)}, ${yawDeg.toFixed(1)}°)`);
  };

  const applySavedTargetPose = (pose) => {
    if (!autoDriveCmdRef.current) {
      setAutoDriveCmdInfo("ROS未接続のため送信できません");
      return;
    }

    setTargetXInput(pose.x.toFixed(3));
    setTargetYInput(pose.y.toFixed(3));
    setTargetYawInput(pose.yawDeg.toFixed(1));

    autoDriveCmdRef.current.publish({
      data: [pose.x, pose.y, pose.yaw],
    });

    setAutoDriveCmdInfo(`保存値 "${pose.label}" を送信しました`);
  };

  const deleteSavedPose = (id) => {
    setSavedPosesList(savedPosesList.filter((pose) => pose.id !== id));
  };

  const clearAllSavedPoses = () => {
    setSavedPosesList([]);
    setAutoDriveCmdInfo("すべての保存目標値をクリアしました");
  };

  const incrementTarget = (setter, currentValue, step) => {
    const current = parseFloatSafe(currentValue);
    const stepValue = parseFloatSafe(step);
    setter((current + stepValue).toFixed(1));
  };

  const decrementTarget = (setter, currentValue, step) => {
    const current = parseFloatSafe(currentValue);
    const stepValue = parseFloatSafe(step);
    setter((current - stepValue).toFixed(1));
  };

  const publishAutoDriveCommand = () => {
    if (!operationArmed) {
      setAutoDriveCmdInfo("操作許可がOFFのため送信できません");
      return;
    }

    if (!autoDriveCmdRef.current) {
      setAutoDriveCmdInfo("ROS未接続のため送信できません");
      return;
    }

    const tx = parseFloatSafe(targetXInput);
    const ty = parseFloatSafe(targetYInput);
    const tyawDeg = parseFloatSafe(targetYawInput);
    const tyawRad = tyawDeg * Math.PI / 180;

    autoDriveCmdRef.current.publish({
      data: [tx, ty, tyawRad],
    });

    setAutoDriveCmdInfo("r2_autodrive_cmd に目標座標を送信しました");
  };

  useEffect(() => {
    if (!operationArmed && serialPeriodicEnabled) {
      setSerialPeriodicEnabled(false);
      setSerialPublishInfo("操作許可がOFFのため定期送信を停止しました");
    }
  }, [operationArmed, serialPeriodicEnabled]);

  useEffect(() => {
    if (serialPeriodicTimerRef.current) {
      clearInterval(serialPeriodicTimerRef.current);
      serialPeriodicTimerRef.current = null;
    }

    if (!serialPeriodicEnabled || !operationArmed) {
      return;
    }

    const parsedHz = Number.parseFloat(serialPeriodicHz);
    const safeHz = Number.isFinite(parsedHz)
      ? Math.max(0.5, Math.min(100, parsedHz))
      : 10;
    const intervalMs = Math.max(10, Math.round(1000 / safeHz));

    setSerialPublishInfo(`定期送信中: ${safeHz} Hz`);
    serialPeriodicTimerRef.current = setInterval(() => {
      publishSerialPacket(false);
    }, intervalMs);

    return () => {
      if (serialPeriodicTimerRef.current) {
        clearInterval(serialPeriodicTimerRef.current);
        serialPeriodicTimerRef.current = null;
      }
    };
  }, [serialPeriodicEnabled, serialPeriodicHz, serialTargetIdInput, serialElementCount, serialValues, operationArmed]);

  const updateCommand = (value) => {
    commandValueRef.current = value;
    setCommandValue(value);
  };

  const resetAllControls = () => {
    updateCommand(0);

    const nextButtons = Array(14).fill(0);
    const nextAxes = Array(8).fill(0);

    buttonsRef.current = nextButtons;
    axesRef.current = nextAxes;
    setButtons(nextButtons);
    setAxes(nextAxes);
  };

  const getHoldHandlers = (onPress, onRelease) => ({
    onPointerDown: (event) => {
      console.log("Pointer DOWN:", event.pointerId);
      event.preventDefault();
      event.stopPropagation();
      if (event.currentTarget.setPointerCapture) {
        event.currentTarget.setPointerCapture(event.pointerId);
      }
      onPress();
    },
    onPointerUp: (event) => {
      console.log("Pointer UP:", event.pointerId);
      event.preventDefault();
      event.stopPropagation();
      onRelease();
    },
    onPointerCancel: (event) => {
      console.log("Pointer CANCEL:", event.pointerId);
      event.preventDefault();
      event.stopPropagation();
      onRelease();
    },
    onLostPointerCapture: (event) => {
      console.log("Lost Pointer Capture:", event.pointerId);
      event.preventDefault();
      onRelease();
    },
    onContextMenu: (event) => event.preventDefault(),
  });

  const setPsButton = (index, isPressed) => {
    const nextButtons = [...buttonsRef.current];
    nextButtons[index] = isPressed ? 1 : 0;
    buttonsRef.current = nextButtons;
    setButtons(nextButtons);
  };

  const setPsAxis = (index, value) => {
    const nextAxes = [...axesRef.current];
    nextAxes[index] = value;
    axesRef.current = nextAxes;
    setAxes(nextAxes);
  };

  const getButtonPressProps = (index) =>
    getHoldHandlers(
      () => setPsButton(index, true),
      () => setPsButton(index, false)
    );

  const getAxisPressProps = (index, value) =>
    getHoldHandlers(
      () => setPsAxis(index, value),
      () => setPsAxis(index, 0)
    );

  useEffect(() => {
    const resetOnBlur = () => {
      resetAllControls();
    };

    const resetOnHidden = () => {
      if (document.visibilityState === "hidden") {
        resetAllControls();
      }
    };

    window.addEventListener("blur", resetOnBlur);
    window.addEventListener("visibilitychange", resetOnHidden);

    return () => {
      window.removeEventListener("blur", resetOnBlur);
      window.removeEventListener("visibilitychange", resetOnHidden);
    };
  }, []);

  useEffect(() => {
    setStatus("接続中...");

    // ROS接続
    rosRef.current = new ROSLIB.Ros({
      url: rosUrl,
    });

    rosRef.current.on("connection", () => {
      setStatus("接続OK");
      console.log("Connected to ROS:", rosUrl);
      refreshTopicList();
      refreshSerialBridgeStatus();
      refreshSerialBridgeLogs();
    });

    rosRef.current.on("error", (error) => {
      setStatus("エラー");
      console.log("Error:", error);
    });

    rosRef.current.on("close", () => {
      setStatus("切断");
      console.log("Connection closed");
    });

    // 古いjoyトピックをクリーンアップ
    if (joyRef.current) {
      try {
        joyRef.current.unsubscribe?.();
        console.log("Old joy topic unsubscribed");
      } catch (error) {
        console.warn("Error unsubscribing old joy topic:", error);
      }
    }

    // Topic定義
    commandRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/command",
      messageType: "std_msgs/msg/Int32MultiArray",
    });

    joyRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: joyTopicName,
      messageType: "sensor_msgs/msg/Joy",
    });
    console.log("Joy topic created:", joyTopicName);

    odomRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "odom_xy_yaw",
      messageType: "std_msgs/msg/Float32MultiArray",
    });
    odomRef.current.subscribe((msg) => {
      if (!msg?.data || msg.data.length < 3) return;
      setPoseX(Number(msg.data[0]) || 0);
      setPoseY(Number(msg.data[1]) || 0);
      setPoseYaw(Number(msg.data[2]) || 0);
    });

    autoDriveCmdRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "r2_autodrive_cmd",
      messageType: "std_msgs/msg/Float32MultiArray",
    });

    odomResetCmdRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "odom_reset",
      messageType: "std_msgs/msg/Bool",
    });

    rosTopicsServiceRef.current = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/rosapi/topics",
      serviceType: "rosapi_msgs/srv/Topics",
    });

    rosTopicTypeServiceRef.current = new ROSLIB.Service({
      ros: rosRef.current,
      name: "/rosapi/topic_type",
      serviceType: "rosapi_msgs/srv/TopicType",
    });

    // 10Hzで送信
    const interval = setInterval(() => {
      if (!controllerEnabled || !operationArmed) return;

      if (commandRef.current) {
        commandRef.current.publish({
          data: [commandValueRef.current],
        });
      }

      if (joyRef.current) {
        joyRef.current.publish({
          axes: axesRef.current,
          buttons: buttonsRef.current,
        });
      }
    }, 100);

    return () => {
      clearInterval(interval);
      // Cleanup joy topic explicitly before closing ROS
      if (joyRef.current) {
        try {
          joyRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing joy topic:", error);
        }
      }
      if (odomRef.current) {
        try {
          odomRef.current.unsubscribe?.();
        } catch (error) {
          console.warn("Error unsubscribing odom topic:", error);
        }
      }
      stopTopicEcho();
      if (rosRef.current) rosRef.current.close();
    };
  }, [rosUrl, joyTopicName]);

  if (frontendForceStopped) {
    return (
      <div className="console-page">
        <div className="console-bg-shape console-bg-shape-a" />
        <div className="console-bg-shape console-bg-shape-b" />

        <main className="console-card">
          <header className="console-header">
            <img src="/logo.svg" alt="NR26 Logo" className="console-logo" />
            <div>
              <h1>R2 Console</h1>
              <p>Frontend Forced Shutdown</p>
            </div>
          </header>

          <section className="disabled-notice">
            フロントエンドを強制停止しました。復帰するには再読み込みしてください。
          </section>

          <section className="control-toggle-row">
            <button className="connection-button btn-connect" onClick={() => window.location.reload()}>
              再読み込み
            </button>
          </section>
        </main>
      </div>
    );
  }

  if (controllerFullscreen) {
    return (
      <div className="console-page console-page-fullscreen">
        <main className="console-card console-card-fullscreen-controller">
          <button
            className="fullscreen-close-button-top"
            onClick={() => setControllerFullscreen(false)}
          >
            ✕ Back to Normal
          </button>

          <div className="ps4-panel-fullscreen">
            <div className="ps-shoulder-row">
              <div className="ps-shoulder-side">
                <button className={`ps-button ps-shoulder ${buttons[6] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(6)}>
                  L2
                </button>
                <button className={`ps-button ps-shoulder ${buttons[4] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(4)}>
                  L1
                </button>
              </div>
              <div className="ps-shoulder-side ps-shoulder-side-right">
                <button className={`ps-button ps-shoulder ${buttons[5] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(5)}>
                  R1
                </button>
                <button className={`ps-button ps-shoulder ${buttons[7] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(7)}>
                  R2
                </button>
              </div>
            </div>

            <div className="ps-main-row-fullscreen">
              <div className="dpad-grid-fullscreen">
                <button className={`dpad-button ${axes[7] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(7, 1)}>
                  ↑
                </button>
                <button className={`dpad-button ${axes[6] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(6, -1)}>
                  ←
                </button>
                <button className={`dpad-button ${axes[6] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(6, 1)}>
                  →
                </button>
                <button className={`dpad-button ${axes[7] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(7, -1)}>
                  ↓
                </button>
              </div>

              <div className="face-grid-fullscreen">
                <button className={`ps-button ps-face ps-triangle ${buttons[2] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(2)}>
                  △
                </button>
                <button className={`ps-button ps-face ps-square ${buttons[3] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(3)}>
                  □
                </button>
                <button className={`ps-button ps-face ps-circle ${buttons[1] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(1)}>
                  ○
                </button>
                <button className={`ps-button ps-face ps-cross ${buttons[0] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(0)}>
                  ×
                </button>
              </div>
            </div>

            <div className="ps-system-row-fullscreen">
              <button className={`ps-button ps-system ${buttons[8] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(8)}>
                SHARE
              </button>
              <button className={`ps-button ps-system ps-home ${buttons[12] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(12)}>
                PS
              </button>
              <button className={`ps-button ps-system ${buttons[9] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(9)}>
                OPTIONS
              </button>
            </div>

            <div className="ps-stick-row-fullscreen">
              <button className={`ps-button ps-stick ${buttons[10] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(10)}>
                L3
              </button>
              <button className={`ps-button ps-stick ${buttons[11] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(11)}>
                R3
              </button>
            </div>
          </div>
        </main>
      </div>
    );
  }

  return (
    <div className="console-page">
      <div className="console-bg-shape console-bg-shape-a" />
      <div className="console-bg-shape console-bg-shape-b" />

      <main className="console-card">
        <header className="console-header">
          <img src="/logo.svg" alt="NR26 Logo" className="console-logo" />
          <div>
            <h1>R2 Console</h1>
            <p>ROS2 Teleoperation Panel</p>
          </div>
        </header>

        <section className="connection-row">
          <input
            className="connection-input"
            value={rosHostInput}
            onChange={(e) => setRosHostInput(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") applyRosEndpoint();
            }}
            placeholder="ROS IP (例: 192.168.1.10)"
          />
          <input
            className="connection-input connection-port"
            value={rosPortInput}
            onChange={(e) => setRosPortInput(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") applyRosEndpoint();
            }}
            placeholder="9090"
          />
          <button className="connection-button btn-connect" onClick={applyRosEndpoint}>
            接続
          </button>
        </section>

        <p className="connection-hint">現在の接続先: {rosUrl}</p>

        <section className="status-row">
          <span className="status-label">状態</span>
          <span
            className={`status-pill ${status === "接続OK"
              ? "status-ok"
              : status === "接続中..."
                ? "status-pending"
                : "status-bad"
              }`}
          >
            {status}
          </span>
        </section>

        <section className="control-toggle-row">
          <button
            className={`toggle-button ${operationArmed ? "toggle-on" : "toggle-off"}`}
            onClick={() => {
              const next = !operationArmed;
              setOperationArmed(next);
              if (!next) {
                resetAllControls();
              }
            }}
          >
            {operationArmed ? "操作ロック: OFF" : "操作ロック: ON"}
          </button>
          <span className="connection-hint">
            {operationArmed
              ? "送信系機能が有効です。注意して操作してください。"
              : "ロック中: 送信系機能は無効化されています"}
          </span>
        </section>

        <section className="page-switch-row">
          <button
            className={`page-switch-button ${activePage === "controller" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("controller")}
          >
            コントローラ操作
          </button>
          <button
            className={`page-switch-button ${activePage === "sequence" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("sequence")}
          >
            シーケンス操作
          </button>
          <button
            className={`page-switch-button ${activePage === "pose" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("pose")}
          >
            座標・姿勢管理
          </button>
          <button
            className={`page-switch-button ${activePage === "actuator" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("actuator")}
          >
            アクチュエータ送信
          </button>
          <button
            className={`page-switch-button ${activePage === "topic" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("topic")}
          >
            トピック監視
          </button>
          <button
            className={`page-switch-button ${activePage === "serial-bridge" ? "page-switch-active" : ""}`}
            onClick={() => {
              setActivePage("serial-bridge");
              refreshSerialBridgeStatus();
              refreshTopicList();
              refreshSerialBridgeLogs();
            }}
          >
            Serial Bridge
          </button>
          <button
            className={`page-switch-button ${activePage === "shutdown" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("shutdown")}
          >
            強制停止
          </button>
          <button
            className={`page-switch-button ${activePage === "settings" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("settings")}
          >
            設定
          </button>
        </section>

        {activePage === "controller" && (
          <>
            <section className="joy-topic-row">
              <input
                className="connection-input"
                value={joyTopicInput}
                onChange={(e) => setJoyTopicInput(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === "Enter") applyJoyTopicName();
                }}
                placeholder="Joy Topic Name (例: /joy_9)"
              />
              <button className="connection-button btn-connect" onClick={applyJoyTopicName}>
                更新
              </button>
            </section>

            <p className="connection-hint">現在のJoyトピック: {joyTopicName}</p>

            <section className="control-toggle-row">
              <button
                className={`toggle-button ${controllerEnabled ? "toggle-on" : "toggle-off"}`}
                onClick={() => setControllerEnabled(!controllerEnabled)}
              >
                {controllerEnabled ? "コントローラー: ON" : "コントローラー: OFF"}
              </button>
              <button
                className="fullscreen-toggle-button"
                onClick={() => setControllerFullscreen(true)}
              >
                全画面操作
              </button>
            </section>

            {controllerEnabled && (
              <div className="controller-layout">
                <section className="quick-controls-panel">
                  <div className="quick-controls-row">
                    <div className="control-group">
                      <button
                        className="control-button control-button-emergency emergency-stop-shape"
                        onClick={() => {
                          updateCommand(0);
                        }}
                      >
                        緊急停止
                      </button>
                    </div>
                  </div>

                  <p className="velocity-readout">
                    command: {commandValue}
                  </p>
                </section>

                <section className="ps4-panel">
                  <h2>PS4 Controller Buttons</h2>

                  <div className="ps-shoulder-row">
                    <div className="ps-shoulder-side">
                      <button className={`ps-button ps-shoulder ${buttons[6] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(6)}>
                        L2
                      </button>
                      <button className={`ps-button ps-shoulder ${buttons[4] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(4)}>
                        L1
                      </button>
                    </div>
                    <div className="ps-shoulder-side ps-shoulder-side-right">
                      <button className={`ps-button ps-shoulder ${buttons[5] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(5)}>
                        R1
                      </button>
                      <button className={`ps-button ps-shoulder ${buttons[7] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(7)}>
                        R2
                      </button>
                    </div>
                  </div>

                  <div className="ps-main-row">
                    <div className="dpad-grid">
                      <button className={`dpad-button ${axes[7] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(7, 1)}>
                        ↑
                      </button>
                      <button className={`dpad-button ${axes[6] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(6, -1)}>
                        ←
                      </button>
                      <button className={`dpad-button ${axes[6] === 1 ? "ps-active" : ""}`} {...getAxisPressProps(6, 1)}>
                        →
                      </button>
                      <button className={`dpad-button ${axes[7] === -1 ? "ps-active" : ""}`} {...getAxisPressProps(7, -1)}>
                        ↓
                      </button>
                    </div>

                    <div className="ps-system-row">
                      <button className={`ps-button ps-system ${buttons[8] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(8)}>
                        SHARE
                      </button>
                      <button className={`ps-button ps-system ps-home ${buttons[12] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(12)}>
                        PS
                      </button>
                      <button className={`ps-button ps-system ${buttons[9] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(9)}>
                        OPTIONS
                      </button>
                    </div>

                    <div className="face-grid">
                      <button className={`ps-button ps-face ps-triangle ${buttons[2] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(2)}>
                        △
                      </button>
                      <button className={`ps-button ps-face ps-square ${buttons[3] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(3)}>
                        □
                      </button>
                      <button className={`ps-button ps-face ps-circle ${buttons[1] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(1)}>
                        ○
                      </button>
                      <button className={`ps-button ps-face ps-cross ${buttons[0] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(0)}>
                        ×
                      </button>
                    </div>
                  </div>

                  <div className="ps-stick-row">
                    <button className={`ps-button ps-stick ${buttons[10] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(10)}>
                      L3
                    </button>
                    <button className={`ps-button ps-stick ${buttons[11] === 1 ? "ps-active" : ""}`} {...getButtonPressProps(11)}>
                      R3
                    </button>
                  </div>
                </section>
              </div>
            )}

            {!controllerEnabled && (
              <div className="disabled-notice">
                コントローラーは無効化されています
              </div>
            )}
          </>
        )}

        {activePage === "sequence" && (
          <section className="quick-controls-panel">
            <h2 className="serial-packet-title">シーケンス操作</h2>
            <div className="quick-controls-row sequence-controls-row">
              <div className="control-group">
                <button
                  className="control-button control-button-step-up"
                  {...getHoldHandlers(
                    () => updateCommand(1),
                    () => updateCommand(0)
                  )}
                >
                  段差上り
                </button>
              </div>

              <div className="control-group">
                <button
                  className="control-button control-button-step-down"
                  {...getHoldHandlers(
                    () => updateCommand(-1),
                    () => updateCommand(0)
                  )}
                >
                  段差下り
                </button>
              </div>
            </div>

            <p className="velocity-readout">
              command: {commandValue}
            </p>
          </section>
        )}

        {activePage === "pose" && (
          <section className="pose-panel">
            <h2 className="serial-packet-title">座標・姿勢管理</h2>
            <p className="serial-packet-hint">
              現在座標と目標座標を管理します。（Yaw: 度数法）
            </p>

            <div className="pose-current-grid">
              <div className="pose-current-item">
                <span>現在X</span>
                <strong>{poseX.toFixed(3)}</strong>
              </div>
              <div className="pose-current-item">
                <span>現在Y</span>
                <strong>{poseY.toFixed(3)}</strong>
              </div>
              <div className="pose-current-item">
                <span>現在Yaw (°)</span>
                <strong>{(poseYaw * 180 / Math.PI).toFixed(1)}</strong>
              </div>
            </div>

            <div className="pose-step-grid">
              <label className="serial-packet-label">
                X ステップ
                <input
                  className="connection-input"
                  type="number"
                  step="0.1"
                  value={targetXStep}
                  onChange={(e) => setTargetXStep(e.target.value)}
                />
              </label>
              <label className="serial-packet-label">
                Y ステップ
                <input
                  className="connection-input"
                  type="number"
                  step="0.1"
                  value={targetYStep}
                  onChange={(e) => setTargetYStep(e.target.value)}
                />
              </label>
              <label className="serial-packet-label">
                Yaw ステップ (°)
                <input
                  className="connection-input"
                  type="number"
                  step="1"
                  value={targetYawStep}
                  onChange={(e) => setTargetYawStep(e.target.value)}
                />
              </label>
            </div>

            <div className="pose-input-grid">
              <div className="pose-input-item">
                <label className="serial-packet-label">
                  目標X
                  <input className="connection-input" value={targetXInput} onChange={(e) => setTargetXInput(e.target.value)} />
                </label>
                <div className="pose-button-group">
                  <button className="pose-pm-button" onClick={() => decrementTarget(setTargetXInput, targetXInput, targetXStep)}>−</button>
                  <button className="pose-pm-button" onClick={() => incrementTarget(setTargetXInput, targetXInput, targetXStep)}>+</button>
                </div>
              </div>

              <div className="pose-input-item">
                <label className="serial-packet-label">
                  目標Y
                  <input className="connection-input" value={targetYInput} onChange={(e) => setTargetYInput(e.target.value)} />
                </label>
                <div className="pose-button-group">
                  <button className="pose-pm-button" onClick={() => decrementTarget(setTargetYInput, targetYInput, targetYStep)}>−</button>
                  <button className="pose-pm-button" onClick={() => incrementTarget(setTargetYInput, targetYInput, targetYStep)}>+</button>
                </div>
              </div>

              <div className="pose-input-item">
                <label className="serial-packet-label">
                  目標Yaw (°)
                  <input className="connection-input" value={targetYawInput} onChange={(e) => setTargetYawInput(e.target.value)} />
                </label>
                <div className="pose-button-group">
                  <button className="pose-pm-button" onClick={() => decrementTarget(setTargetYawInput, targetYawInput, targetYawStep)}>−</button>
                  <button className="pose-pm-button" onClick={() => incrementTarget(setTargetYawInput, targetYawInput, targetYawStep)}>+</button>
                </div>
              </div>
            </div>

            <div className="pose-actions-row">
              <button className="connection-button btn-neutral" onClick={applyAutoDriveFromCurrentPose}>現在値を目標へ</button>
              <button className="connection-button btn-neutral" onClick={publishOdomReset} disabled={!operationArmed}>オドメトリをリセット</button>
              <button className="connection-button serial-send-button btn-send" onClick={publishAutoDriveCommand} disabled={!operationArmed}>目標座標を送信</button>
            </div>

            <p className="connection-hint">{autoDriveCmdInfo}</p>

            <div className="pose-target-save-panel">
              <button className="connection-button serial-send-button btn-save" onClick={saveTargetPose}>
                目標値を保存
              </button>
              {savedPosesList.length > 0 && (
                <button className="serial-clear-button" onClick={clearAllSavedPoses}>
                  すべてクリア
                </button>
              )}
            </div>

            {savedPosesList.length > 0 && (
              <div className="pose-saved-list-panel">
                <h3 className="pose-saved-list-title">保存済み目標値 ({savedPosesList.length})</h3>
                <div className="pose-saved-list">
                  {savedPosesList.map((pose) => (
                    <div key={pose.id} className="pose-saved-item">
                      <div className="pose-saved-item-info">
                        <span className="pose-saved-item-label">{pose.label}</span>
                        <span className="pose-saved-item-values">
                          X: {pose.x.toFixed(3)}, Y: {pose.y.toFixed(3)}, Yaw: {pose.yawDeg.toFixed(1)}°
                        </span>
                        <span className="pose-saved-item-time">{pose.timestamp}</span>
                      </div>
                      <div className="pose-saved-item-actions">
                        <button
                          className="connection-button pose-item-button btn-restore"
                          onClick={() => applySavedTargetPose(pose)}
                        >
                          復元&送信
                        </button>
                        <button
                          className="serial-clear-button pose-item-button"
                          onClick={() => deleteSavedPose(pose.id)}
                        >
                          削除
                        </button>
                      </div>
                    </div>
                  ))}
                </div>
              </div>
            )}
          </section>
        )}

        {activePage === "actuator" && (
          <section className="serial-packet-section">
            <h2 className="serial-packet-title">アクチュエータ送信 (Int16MultiArray)</h2>
            <p className="serial-packet-hint">
              IDで送信先を切替えることができます。トピック名: <strong>{serialTopicName}</strong>
              <br />
              入力値のチェックは行っていません。注意して入力してください。
            </p>

            <div className="serial-packet-controls">
              <label className="serial-packet-label">
                ID
                <input
                  className="connection-input"
                  type="number"
                  min="0"
                  max="255"
                  value={serialTargetIdInput}
                  onChange={(e) => setSerialTargetIdInput(e.target.value)}
                />
              </label>
              <label className="serial-packet-label">
                要素数
                <input
                  className="connection-input"
                  type="number"
                  min="1"
                  max="64"
                  value={serialElementCount}
                  onChange={(e) => updateSerialElementCount(e.target.value)}
                />
              </label>
              <label className="serial-packet-label">
                DEBUG
                <input
                  className="connection-input"
                  type="number"
                  min="0"
                  max="1"
                  value={serialValues[0] ?? 0}
                  onChange={(e) => updateSerialValue(0, e.target.value)}
                />
              </label>
            </div>

            <div className="actuator-groups">
              <section className="actuator-group">
                <h3 className="actuator-group-title">モータ (MD1 - MD8)</h3>
                <div className="serial-packet-grid">
                  {Array.from({ length: 8 }, (_, i) => i + 1)
                    .filter((index) => index < serialElementCount)
                    .map((index) => renderSerialInputItem(index))}
                </div>
              </section>

              <section className="actuator-group">
                <h3 className="actuator-group-title">サーボ (SERVO1 - SERVO8)</h3>
                <div className="serial-packet-grid">
                  {Array.from({ length: 8 }, (_, i) => i + 9)
                    .filter((index) => index < serialElementCount)
                    .map((index) => renderSerialInputItem(index))}
                </div>
              </section>

              <section className="actuator-group">
                <h3 className="actuator-group-title">TR出力 (TR1 - TR7)</h3>
                <div className="serial-packet-grid">
                  {Array.from({ length: 7 }, (_, i) => i + 17)
                    .filter((index) => index < serialElementCount)
                    .map((index) => renderSerialInputItem(index))}
                </div>
              </section>

              <section className="actuator-group actuator-actions-group">
                <h3 className="actuator-group-title">配列操作</h3>
                <div className="serial-packet-actions">
                  <button className="connection-button serial-send-button btn-send" onClick={() => publishSerialPacket(true)} disabled={!operationArmed}>
                    配列送信
                  </button>
                  <button className="serial-clear-button" onClick={clearSerialPacket}>
                    配列クリア
                  </button>

                  <label className="serial-packet-label">
                    定期送信 Hz
                    <select
                      className="connection-input"
                      value={serialPeriodicHz}
                      onChange={(e) => setSerialPeriodicHz(e.target.value)}
                    >
                      <option value="1">1</option>
                      <option value="2">2</option>
                      <option value="5">5</option>
                      <option value="10">10</option>
                      <option value="20">20</option>
                      <option value="30">30</option>
                      <option value="50">50</option>
                    </select>
                  </label>

                  <button
                    className={`serial-periodic-button ${serialPeriodicEnabled ? "serial-periodic-on" : ""}`}
                    disabled={!operationArmed}
                    onClick={() => setSerialPeriodicEnabled((prev) => !prev)}
                  >
                    {serialPeriodicEnabled ? "定期送信: ON" : "定期送信: OFF"}
                  </button>
                </div>
              </section>

              {serialElementCount > 24 && (
                <section className="actuator-group">
                  <h3 className="actuator-group-title">追加チャネル (CH24+)</h3>
                  <div className="serial-packet-grid">
                    {Array.from({ length: serialElementCount - 24 }, (_, i) => i + 24)
                      .map((index) => renderSerialInputItem(index))}
                  </div>
                </section>
              )}
            </div>

            <p className="connection-hint">{serialPublishInfo}</p>
          </section>
        )}

        {activePage === "topic" && (
          <section className="topic-panel">
            <h2 className="serial-packet-title">ROS2 トピック監視</h2>
            <p className="serial-packet-hint">
              トピック一覧を取得し、選択したトピックを subscribe して内容を確認できます。
            </p>

            <div className="topic-toolbar">
              <button className="connection-button btn-connect" onClick={refreshTopicList}>
                トピック一覧を更新
              </button>
              <span className="connection-hint">
                {topicListLoading ? "取得中..." : `件数: ${topicList.length}`}
              </span>
            </div>

            {topicListError && <p className="connection-hint topic-error">{topicListError}</p>}

            <div className="topic-select-row">
              <select
                className="connection-input"
                value={selectedEchoTopic}
                onChange={(e) => {
                  const topicName = e.target.value;
                  setSelectedEchoTopic(topicName);
                  const matched = topicList.find((item) => item.name === topicName);
                  setSelectedEchoType(matched?.type || "");
                }}
              >
                <option value="">監視するトピックを選択</option>
                {topicList.map((item) => (
                  <option key={item.name} value={item.name}>
                    {item.name} {item.type ? `(${item.type})` : ""}
                  </option>
                ))}
              </select>

              <button className="connection-button btn-send" onClick={startTopicEcho}>
                Echo開始
              </button>
              <button className="serial-clear-button" onClick={stopTopicEcho}>
                Echo停止
              </button>
            </div>

            <p className="connection-hint">
              {topicEchoInfo} {topicEchoRunning ? "(受信中)" : ""}
            </p>

            <div className="topic-list-box">
              {topicList.map((item) => (
                <div key={item.name} className="topic-list-row">
                  <span className="topic-list-name">{item.name}</span>
                  <span className="topic-list-type">{item.type || "型不明"}</span>
                </div>
              ))}
            </div>

            <h3 className="topic-echo-title">Echoログ (最新30件)</h3>
            <div className="topic-echo-box">
              {topicEchoMessages.length === 0 && (
                <p className="connection-hint">まだ受信していません</p>
              )}
              {topicEchoMessages.map((row) => (
                <article key={row.id} className="topic-echo-item">
                  <header className="topic-echo-meta">{row.at}</header>
                  <pre className="topic-echo-pre">{row.payload}</pre>
                </article>
              ))}
            </div>
          </section>
        )}

        {activePage === "serial-bridge" && (
          <section className="serial-bridge-panel">
            <h2 className="serial-packet-title">Serial Bridge 管理</h2>
            <p className="serial-packet-hint">
              検出ポート、トピック由来のDevice ID、実行状態を確認し、ここから serial_bridge を手動起動できます。
              <br />
              serial_bridge はコンソール起動時には自動起動しません。
            </p>

            <div className="serial-bridge-toolbar">
              <button className="connection-button btn-connect" onClick={refreshSerialBridgeStatus}>
                状態更新
              </button>
              <button className="connection-button btn-send" onClick={startSerialBridgeFromConsole}>
                serial_bridge 起動
              </button>
              <button className="connection-button btn-neutral" onClick={stopSerialBridgeFromConsole}>
                serial_bridge 停止
              </button>
              <button className="connection-button btn-connect" onClick={refreshSerialBridgeLogs}>
                ログ更新
              </button>
              <button
                className={`serial-periodic-button ${serialBridgeLogRealtimeEnabled ? "serial-periodic-on" : ""}`}
                onClick={() => setSerialBridgeLogRealtimeEnabled((prev) => !prev)}
              >
                {serialBridgeLogRealtimeEnabled ? "ログ自動更新: ON" : "ログ自動更新: OFF"}
              </button>
              <span className="connection-hint">{serialBridgeLoading ? "処理中..." : serialBridgeInfo}</span>
            </div>

            <div className="serial-bridge-grid">
              <section className="serial-bridge-card">
                <h3 className="serial-bridge-title">実行状態</h3>
                <p className="connection-hint">running: {serialBridgeRunning ? "ON" : "OFF"}</p>
                <p className="connection-hint">pid: {serialBridgePid || "-"}</p>
              </section>

              <section className="serial-bridge-card">
                <h3 className="serial-bridge-title">検出ポート</h3>
                <div className="serial-bridge-list">
                  {serialBridgePorts.length === 0 && <p className="connection-hint">ポート未検出</p>}
                  {serialBridgePorts.map((port) => (
                    <div className="serial-bridge-list-row" key={port}>{port}</div>
                  ))}
                </div>
              </section>

              <section className="serial-bridge-card">
                <h3 className="serial-bridge-title">検出ID (serial_rx/tx)</h3>
                <div className="serial-bridge-list">
                  {serialBridgeIds.length === 0 && <p className="connection-hint">ID未検出</p>}
                  {serialBridgeIds.map((id) => (
                    <div className="serial-bridge-list-row" key={`id-${id}`}>ID: {id}</div>
                  ))}
                </div>
              </section>

              <section className="serial-bridge-card serial-bridge-card-log">
                <h3 className="serial-bridge-title">serial_bridge ログ </h3>
                <div className="serial-bridge-log-box" ref={serialBridgeLogBoxRef}>
                  {serialBridgeLogLoading && <p className="connection-hint">ログ取得中...</p>}
                  {!serialBridgeLogLoading && serialBridgeLogs.length === 0 && (
                    <p className="connection-hint">ログはまだありません</p>
                  )}
                  {!serialBridgeLogLoading && serialBridgeLogs.length > 0 && (
                    <pre className="serial-bridge-log-pre">{serialBridgeLogs.join("\n")}</pre>
                  )}
                </div>
              </section>
            </div>
          </section>
        )}

        {activePage === "shutdown" && (
          <section className="serial-bridge-panel">
            <h2 className="serial-packet-title">強制停止</h2>
            <p className="serial-packet-hint">
              フロントエンドとバックエンドの強制停止操作を行います。実行すると復帰に再起動/再読み込みが必要です。強制停止後の機体の動作は保証できません。
            </p>

            <div className="serial-bridge-toolbar">
              <button className="connection-button btn-neutral" onClick={forceShutdownFrontendFromConsole}>
                frontend 強制停止
              </button>
              <button className="connection-button btn-neutral" onClick={forceShutdownBackendFromConsole}>
                backend 強制停止
              </button>
              <span className="connection-hint">{serialBridgeLoading ? "処理中..." : serialBridgeInfo}</span>
            </div>
          </section>
        )}

        {activePage === "settings" && (
          <section className="serial-bridge-panel">
            <h2 className="serial-packet-title">設定</h2>
            <p className="serial-packet-hint">
              接続先、操作ロック、送信設定、ログ設定を一括管理します。
            </p>

            <div className="serial-bridge-grid">
              <section className="serial-bridge-card">
                <h3 className="serial-bridge-title">接続設定</h3>
                <div className="serial-bridge-list">
                  <label className="serial-packet-label">
                    ROS Host
                    <input
                      className="connection-input"
                      value={rosHostInput}
                      onChange={(e) => setRosHostInput(e.target.value)}
                    />
                  </label>
                  <label className="serial-packet-label">
                    ROS Port
                    <input
                      className="connection-input"
                      value={rosPortInput}
                      onChange={(e) => setRosPortInput(e.target.value)}
                    />
                  </label>
                  <label className="serial-packet-label">
                    Joy Topic
                    <input
                      className="connection-input"
                      value={joyTopicInput}
                      onChange={(e) => setJoyTopicInput(e.target.value)}
                    />
                  </label>
                </div>
              </section>

              <section className="serial-bridge-card">
                <h3 className="serial-bridge-title">操作設定</h3>
                <div className="serial-bridge-list">
                  <button
                    className={`toggle-button ${operationArmed ? "toggle-on" : "toggle-off"}`}
                    onClick={() => {
                      const next = !operationArmed;
                      setOperationArmed(next);
                      if (!next) {
                        resetAllControls();
                      }
                    }}
                  >
                    {operationArmed ? "操作ロック: OFF" : "操作ロック: ON"}
                  </button>
                  <button
                    className={`toggle-button ${controllerEnabled ? "toggle-on" : "toggle-off"}`}
                    onClick={() => setControllerEnabled((prev) => !prev)}
                  >
                    {controllerEnabled ? "コントローラー: ON" : "コントローラー: OFF"}
                  </button>
                  <button
                    className={`serial-periodic-button ${serialBridgeLogRealtimeEnabled ? "serial-periodic-on" : ""}`}
                    onClick={() => setSerialBridgeLogRealtimeEnabled((prev) => !prev)}
                  >
                    {serialBridgeLogRealtimeEnabled ? "ログ自動更新: ON" : "ログ自動更新: OFF"}
                  </button>
                </div>
              </section>

              <section className="serial-bridge-card">
                <h3 className="serial-bridge-title">送信設定</h3>
                <div className="serial-bridge-list">
                  <label className="serial-packet-label">
                    Serial Target ID
                    <input
                      className="connection-input"
                      type="number"
                      min="0"
                      max="255"
                      value={serialTargetIdInput}
                      onChange={(e) => setSerialTargetIdInput(e.target.value)}
                    />
                  </label>
                  <label className="serial-packet-label">
                    配列要素数
                    <input
                      className="connection-input"
                      type="number"
                      min="1"
                      max="64"
                      value={serialElementCount}
                      onChange={(e) => updateSerialElementCount(e.target.value)}
                    />
                  </label>
                  <label className="serial-packet-label">
                    定期送信Hz
                    <select
                      className="connection-input"
                      value={serialPeriodicHz}
                      onChange={(e) => setSerialPeriodicHz(e.target.value)}
                    >
                      <option value="1">1</option>
                      <option value="2">2</option>
                      <option value="5">5</option>
                      <option value="10">10</option>
                      <option value="20">20</option>
                      <option value="30">30</option>
                      <option value="50">50</option>
                    </select>
                  </label>
                </div>
              </section>

              <section className="serial-bridge-card">
                <h3 className="serial-bridge-title">ログ設定</h3>
                <div className="serial-bridge-list">
                  <label className="serial-packet-label">
                    取得行数 (10-1000)
                    <input
                      className="connection-input"
                      type="number"
                      min="10"
                      max="1000"
                      value={serialBridgeLogLinesInput}
                      onChange={(e) => setSerialBridgeLogLinesInput(e.target.value)}
                    />
                  </label>
                  <p className="connection-hint">現在の取得行数: {serialBridgeLogLineLimit} 行</p>
                </div>
              </section>
            </div>

            <div className="serial-bridge-toolbar">
              <button className="connection-button btn-connect" onClick={applySettingsValues}>
                設定を適用
              </button>
              <button className="connection-button btn-neutral" onClick={refreshSerialBridgeStatus}>
                状態を再取得
              </button>
              <span className="connection-hint">{serialBridgeLoading ? "処理中..." : serialBridgeInfo}</span>
            </div>
          </section>
        )}
      </main>
    </div>
  );
}

export default App;