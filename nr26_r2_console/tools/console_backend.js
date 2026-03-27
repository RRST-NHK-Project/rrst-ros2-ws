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

const stopSerialBridge = () => {
    const runningInfo = getRunningInfo();
    if (!runningInfo.running || !Number.isInteger(runningInfo.pid)) {
        return {
            stopped: false,
            running: false,
            pid: null,
            message: "serial_bridge は起動していません",
        };
    }

    try {
        process.kill(runningInfo.pid, "SIGTERM");
        return {
            stopped: true,
            running: false,
            pid: runningInfo.pid,
            message: "serial_bridge を停止しました",
        };
    } catch (error) {
        return {
            stopped: false,
            running: true,
            pid: runningInfo.pid,
            message: `serial_bridge の停止に失敗しました: ${error.message}`,
        };
    }
};

const readLogTail = (lineLimit = 200) => {
    let fileText = "";
    try {
        fileText = fs.readFileSync(SERIAL_LOG_PATH, "utf8");
    } catch (error) {
        if (error && error.code === "ENOENT") {
            return [];
        }
        throw error;
    }

    const lines = fileText.split(/\r?\n/);
    if (lines.length > 0 && lines[lines.length - 1] === "") {
        lines.pop();
    }
    return lines.slice(-lineLimit);
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

    if (req.method === "POST" && url.pathname === "/api/serial-bridge/stop") {
        const stopInfo = stopSerialBridge();
        jsonResponse(res, 200, stopInfo);
        return;
    }

    if (req.method === "GET" && url.pathname === "/api/serial-bridge/logs") {
        const rawLines = url.searchParams.get("lines");
        const parsedLines = Number.parseInt(rawLines || "200", 10);
        const lineLimit = Number.isFinite(parsedLines)
            ? Math.max(10, Math.min(1000, parsedLines))
            : 200;

        try {
            const lines = readLogTail(lineLimit);
            jsonResponse(res, 200, {
                logPath: SERIAL_LOG_PATH,
                lines,
                lineCount: lines.length,
            });
        } catch (error) {
            jsonResponse(res, 500, {
                message: "ログ取得に失敗しました",
                error: String(error?.message || error),
            });
        }
        return;
    }

    jsonResponse(res, 404, { message: "not found" });
});

server.listen(BACKEND_PORT, "0.0.0.0", () => {
    console.log(`Console backend listening on :${BACKEND_PORT}`);
});
