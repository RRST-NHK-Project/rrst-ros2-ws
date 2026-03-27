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
  if (label === "DEBUG") return "デバッグフラグ (0/1)";
  if (label.startsWith("MD")) return "モータ出力 (-255〜255 推奨)";
  if (label.startsWith("SERVO")) return "サーボ角度 (0〜270 推奨)";
  if (label.startsWith("TR")) return "デジタル出力 (0/1)";
  return "予備";
};

function App() {
  const rosRef = useRef(null);
  const commandRef = useRef(null);
  const joyRef = useRef(null);
  const odomRef = useRef(null);
  const autoDriveCmdRef = useRef(null);
  const serialPeriodicTimerRef = useRef(null);
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
  const [controllerEnabled, setControllerEnabled] = useState(true);
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
  const [autoDriveCmdInfo, setAutoDriveCmdInfo] = useState("未送信");

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
    setTargetYawInput(poseYaw.toFixed(3));
  };

  const publishAutoDriveCommand = () => {
    if (!autoDriveCmdRef.current) {
      setAutoDriveCmdInfo("ROS未接続のため送信できません");
      return;
    }

    const tx = parseFloatSafe(targetXInput);
    const ty = parseFloatSafe(targetYInput);
    const tyaw = parseFloatSafe(targetYawInput);

    autoDriveCmdRef.current.publish({
      data: [tx, ty, tyaw],
    });

    setAutoDriveCmdInfo("r2_autodrive_cmd に目標座標を送信しました");
  };

  useEffect(() => {
    if (serialPeriodicTimerRef.current) {
      clearInterval(serialPeriodicTimerRef.current);
      serialPeriodicTimerRef.current = null;
    }

    if (!serialPeriodicEnabled) {
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
  }, [serialPeriodicEnabled, serialPeriodicHz, serialTargetIdInput, serialElementCount, serialValues]);

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

    // 10Hzで送信
    const interval = setInterval(() => {
      if (!controllerEnabled) return;

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
      if (rosRef.current) rosRef.current.close();
    };
  }, [rosUrl, joyTopicName]);

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
          <button className="connection-button" onClick={applyRosEndpoint}>
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
            座標・姿勢速度管理
          </button>
          <button
            className={`page-switch-button ${activePage === "actuator" ? "page-switch-active" : ""}`}
            onClick={() => setActivePage("actuator")}
          >
            アクチュエータ送信
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
              <button className="connection-button" onClick={applyJoyTopicName}>
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
              現在座標と目標座標を管理します。
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
                <span>現在Yaw</span>
                <strong>{poseYaw.toFixed(3)}</strong>
              </div>
            </div>

            <div className="pose-input-grid">
              <label className="serial-packet-label">
                目標X
                <input className="connection-input" value={targetXInput} onChange={(e) => setTargetXInput(e.target.value)} />
              </label>
              <label className="serial-packet-label">
                目標Y
                <input className="connection-input" value={targetYInput} onChange={(e) => setTargetYInput(e.target.value)} />
              </label>
              <label className="serial-packet-label">
                目標Yaw
                <input className="connection-input" value={targetYawInput} onChange={(e) => setTargetYawInput(e.target.value)} />
              </label>
            </div>

            <div className="pose-actions-row">
              <button className="connection-button" onClick={applyAutoDriveFromCurrentPose}>現在値を目標へ</button>
              <button className="connection-button serial-send-button" onClick={publishAutoDriveCommand}>目標座標を送信</button>
            </div>

            <p className="connection-hint">{autoDriveCmdInfo}</p>
          </section>
        )}

        {activePage === "actuator" && (
          <section className="serial-packet-section">
            <h2 className="serial-packet-title">アクチュエータ送信 (Int16MultiArray)</h2>
            <p className="serial-packet-hint">
              IDで送信先を切替え、トピック名は <strong>{serialTopicName}</strong> になります。
              serial_bridge都合で24未満は0埋めして送信します。
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
                  <button className="connection-button serial-send-button" onClick={() => publishSerialPacket(true)}>
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
      </main>
    </div>
  );
}

export default App;