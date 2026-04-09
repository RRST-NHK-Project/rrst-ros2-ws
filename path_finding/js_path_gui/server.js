const fs = require("fs");
const path = require("path");
const http = require("http");

const PORT = Number(process.env.PORT || 3010);
const rootDir = path.join(__dirname, "public");

const mimeByExt = {
  ".html": "text/html; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".js": "application/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
};

function send(res, status, body, contentType = "text/plain; charset=utf-8") {
  res.writeHead(status, { "Content-Type": contentType });
  res.end(body);
}

http.createServer((req, res) => {
  if (req.url === "/health") {
    send(res, 200, JSON.stringify({ ok: true, service: "js_path_gui" }), "application/json; charset=utf-8");
    return;
  }

  const rawPath = req.url === "/" ? "/index.html" : req.url;
  const cleanPath = path.normalize(rawPath).replace(/^\/+/, "").replace(/^\.+/, "");
  const filePath = path.join(rootDir, cleanPath);

  if (!filePath.startsWith(rootDir)) {
    send(res, 404, "Not Found");
    return;
  }

  fs.readFile(filePath, (err, data) => {
    if (err) {
      send(res, err.code === "ENOENT" ? 404 : 500, err.code === "ENOENT" ? "Not Found" : "Server Error");
      return;
    }
    const ext = path.extname(filePath).toLowerCase();
    send(res, 200, data, mimeByExt[ext] || "application/octet-stream");
  });
}).listen(PORT, () => {
  console.log(`js_path_gui started: http://localhost:${PORT}`);
});
