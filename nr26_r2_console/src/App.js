import React, { useEffect, useRef, useState } from "react";
import * as ROSLIB from "roslib";
import "./App.css";

function App() {
  const rosRef = useRef(null);
  const cmdVelRef = useRef(null);

  const [status, setStatus] = useState("接続中...");
  const [linear, setLinear] = useState(0);
  const [angular, setAngular] = useState(0);

  useEffect(() => {
    // ROS接続
    rosRef.current = new ROSLIB.Ros({
      url: "ws://localhost:9090",
    });

    rosRef.current.on("connection", () => {
      setStatus("接続OK");
      console.log("Connected to ROS");
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

    // 🔥 10Hzで送信（重要）
    const interval = setInterval(() => {
      if (!cmdVelRef.current) return;

      const msg = {
        linear: { x: linear, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: angular },
      };

      cmdVelRef.current.publish(msg);
    }, 100);

    return () => {
      clearInterval(interval);
      if (rosRef.current) rosRef.current.close();
    };
  }, [linear, angular]);

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
            onMouseDown={() => setLinear(0.5)}
            onMouseUp={() => setLinear(0)}
            onMouseLeave={() => setLinear(0)}
            onTouchStart={() => setLinear(0.5)}
            onTouchEnd={() => setLinear(0)}
            onTouchCancel={() => setLinear(0)}
          >
            前進
          </button>
        </div>

        {/* 旋回 */}
        <div className="control-group control-turn-group">
          <button
            className="control-button control-button-turn"
            onMouseDown={() => setAngular(1.0)}
            onMouseUp={() => setAngular(0)}
            onMouseLeave={() => setAngular(0)}
            onTouchStart={() => setAngular(1.0)}
            onTouchEnd={() => setAngular(0)}
            onTouchCancel={() => setAngular(0)}
          >
            左旋回
          </button>

          <button
            className="control-button control-button-turn"
            onMouseDown={() => setAngular(-1.0)}
            onMouseUp={() => setAngular(0)}
            onMouseLeave={() => setAngular(0)}
            onTouchStart={() => setAngular(-1.0)}
            onTouchEnd={() => setAngular(0)}
            onTouchCancel={() => setAngular(0)}
          >
            右旋回
          </button>
        </div>

        {/* 停止 */}
        <div className="control-group">
          <button
            className="control-button control-button-stop"
            onClick={() => {
              setLinear(0);
              setAngular(0);
            }}
          >
            停止
          </button>
        </div>

        <p className="velocity-readout">
          linear: {linear.toFixed(1)} | angular: {angular.toFixed(1)}
        </p>
      </main>
    </div>
  );
}

export default App;