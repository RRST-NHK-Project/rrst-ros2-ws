const http = require("http");
const fs = require("fs");
const { execSync, spawn } = require("child_process");
const path = require("path");

const BACKEND_PORT = Number.parseInt(process.env.CONSOLE_BACKEND_PORT || "3031", 10);
const CONSOLE_DIR = path.resolve(__dirname, "..");
const SRC_DIR = path.resolve(CONSOLE_DIR, "..");
const WS_DIR = path.resolve(SRC_DIR, "..");
const SERIAL_LOG_PATH = "/tmp/r2_console_serial_bridge.log";

const jsonResponse = (res, statusCode, payload) => {
    res.writeHead(statusCode, {
        "Content-Type": "application/json; charset=utf-8",
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "GET,POST,OPTIONS",
        "Access-Control-Allow-Headers": "Content-Type",
    });
    res.end(JSON.stringify(payload));
};

const listPortsUsedByProcess = (pid) => {
    if (!Number.isInteger(pid) || pid <= 0) {
        return [];
    }

    const fdDirPath = `/proc/${pid}/fd`;
    let fdEntries = [];
    try {
        fdEntries = fs.readdirSync(fdDirPath);
    } catch (error) {
        return [];
    }

    const ports = new Set();
    fdEntries.forEach((fdName) => {
        try {
            const target = fs.readlinkSync(`${fdDirPath}/${fdName}`);
            if (/^\/dev\/tty(USB|ACM|AMA|S)\d+$/.test(target)) {
                ports.add(target);
            }
        } catch (error) {
            // FD close timing races can happen; ignore transient errors.
        }
    });

    return Array.from(ports).sort();
};

const getRunningInfo = () => {
    try {
        const output = execSync("ps -eo pid,args | grep -E '[s]erial_bridge_node' || true", {
            encoding: "utf8",
            stdio: ["ignore", "pipe", "ignore"],
        }).trim();

        if (!output) {
            return { running: false, pid: null, command: "" };
        }

        const line = output
            .split("\n")
            .map((row) => row.trim())
            .filter((row) => row && !row.includes("console_backend.js"))[0];

        if (!line) {
            return { running: false, pid: null, command: "" };
        }

        const firstSpace = line.indexOf(" ");
        const pid = firstSpace > 0 ? Number.parseInt(line.slice(0, firstSpace), 10) : null;
        const command = firstSpace > 0 ? line.slice(firstSpace + 1) : line;
        return {
            running: Number.isInteger(pid),
            pid: Number.isInteger(pid) ? pid : null,
            command,
        };
    } catch (error) {
        return { running: false, pid: null, command: "" };
    }
};

const startSerialBridge = () => {
    const runningInfo = getRunningInfo();
    if (runningInfo.running) {
        return {
            started: false,
            running: true,
            pid: runningInfo.pid,
            message: "serial_bridge は既に起動中です",
        };
    }

    const command = [
        "set -e",
        "source /opt/ros/jazzy/setup.bash",
        `[ -f '${WS_DIR}/install/setup.bash' ] && source '${WS_DIR}/install/setup.bash' || true`,
        "exec ros2 run serial_bridge serial_bridge_node",
    ].join("; ");

    const child = spawn("bash", ["-lc", command], {
        cwd: WS_DIR,
        detached: true,
        stdio: ["ignore", fs.openSync(SERIAL_LOG_PATH, "a"), fs.openSync(SERIAL_LOG_PATH, "a")],
    });
    child.unref();

    return {
        started: true,
        running: true,
        pid: child.pid,
        message: "serial_bridge を起動しました",
    };
};

const server = http.createServer((req, res) => {
    const url = new URL(req.url, `http://${req.headers.host}`);

    if (req.method === "OPTIONS") {
        jsonResponse(res, 200, { ok: true });
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/serial-bridge/status") {
        const runningInfo = getRunningInfo();
        const usedPorts = listPortsUsedByProcess(runningInfo.pid);
        jsonResponse(res, 200, {
            running: runningInfo.running,
            pid: runningInfo.pid,
            command: runningInfo.command,
            ports: usedPorts,
            logPath: SERIAL_LOG_PATH,
            workspace: WS_DIR,
        });
        return;
    }

    if (req.method === "POST" && url.pathname === "/api/serial-bridge/start") {
        const startInfo = startSerialBridge();
        jsonResponse(res, 200, startInfo);
        return;
    }

    jsonResponse(res, 404, { message: "not found" });
});

server.listen(BACKEND_PORT, "0.0.0.0", () => {
    console.log(`Console backend listening on :${BACKEND_PORT}`);
});
