import React, { useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";
import "./App.css";

function App() {
  const rosRef = useRef(null);
  const cmdVelRef = useRef(null);
  const joyRef = useRef(null);
  const defaultRosHost = window.location.hostname || "localhost";
  const wsScheme = window.location.protocol === "https:" ? "wss" : "ws";
  const linearRef = useRef(0);
  const angularRef = useRef(0);
  const buttonsRef = useRef(Array(14).fill(0));
  const axesRef = useRef(Array(8).fill(0));

  const [status, setStatus] = useState("接続中...");
  const [rosHostInput, setRosHostInput] = useState(defaultRosHost);
  const [rosPortInput, setRosPortInput] = useState("9090");
  const [rosEndpoint, setRosEndpoint] = useState({
    host: defaultRosHost,
    port: "9090",
  });
  const [linear, setLinear] = useState(0);
  const [angular, setAngular] = useState(0);
  const [buttons, setButtons] = useState(Array(14).fill(0));
  const [axes, setAxes] = useState(Array(8).fill(0));

  const rosUrl = `${wsScheme}://${rosEndpoint.host}:${rosEndpoint.port}`;

  const applyRosEndpoint = () => {
    const nextHost = rosHostInput.trim() || defaultRosHost;
    const nextPort = rosPortInput.trim() || "9090";

    setRosEndpoint({ host: nextHost, port: nextPort });
  };

  const updateLinear = (value) => {
    linearRef.current = value;
    setLinear(value);
  };

  const updateAngular = (value) => {
    angularRef.current = value;
    setAngular(value);
  };

  const resetAllControls = () => {
    updateLinear(0);
    updateAngular(0);

    const nextButtons = Array(14).fill(0);
    const nextAxes = Array(8).fill(0);

    buttonsRef.current = nextButtons;
    axesRef.current = nextAxes;
    setButtons(nextButtons);
    setAxes(nextAxes);
  };

  const getHoldHandlers = (onPress, onRelease) => ({
    onPointerDown: (event) => {
      event.preventDefault();
      if (event.currentTarget.setPointerCapture) {
        event.currentTarget.setPointerCapture(event.pointerId);
      }
      onPress();
    },
    onPointerUp: onRelease,
    onPointerCancel: onRelease,
    onLostPointerCapture: onRelease,
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

    // Topic定義
    cmdVelRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/cmd_vel",
      messageType: "geometry_msgs/msg/Twist",
    });

    joyRef.current = new ROSLIB.Topic({
      ros: rosRef.current,
      name: "/joy",
      messageType: "sensor_msgs/msg/Joy",
    });

    // 10Hzで送信
    const interval = setInterval(() => {
      if (cmdVelRef.current) {
        cmdVelRef.current.publish({
          linear: { x: linearRef.current, y: 0, z: 0 },
          angular: { x: 0, y: 0, z: angularRef.current },
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
      if (rosRef.current) rosRef.current.close();
    };
  }, [rosUrl]);

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

        {/* 前進 */}
        <div className="control-group">
          <button
            className="control-button control-button-forward"
            {...getHoldHandlers(
              () => updateLinear(0.5),
              () => updateLinear(0)
            )}
          >
            前進
          </button>
        </div>

        {/* 旋回 */}
        <div className="control-group control-turn-group">
          <button
            className="control-button control-button-turn"
            {...getHoldHandlers(
              () => updateAngular(1.0),
              () => updateAngular(0)
            )}
          >
            左旋回
          </button>

          <button
            className="control-button control-button-turn"
            {...getHoldHandlers(
              () => updateAngular(-1.0),
              () => updateAngular(0)
            )}
          >
            右旋回
          </button>
        </div>

        {/* 停止 */}
        <div className="control-group">
          <button
            className="control-button control-button-stop"
            onClick={() => {
              updateLinear(0);
              updateAngular(0);
            }}
          >
            停止
          </button>
        </div>

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

        <p className="velocity-readout">
          linear: {linear.toFixed(1)} | angular: {angular.toFixed(1)}
        </p>
      </main>
    </div>
  );
}

export default App;