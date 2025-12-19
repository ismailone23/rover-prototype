let port, writer;

// 1. Serial Connection
async function connectSerial() {
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });
    const encoder = new TextEncoderStream();
    encoder.readable.pipeTo(port.writable);
    writer = encoder.writable.getWriter();
    document.getElementById("status").innerText = "ONLINE";
    document.getElementById("status").style.color = "#00ffcc";
    readStream();
  } catch (e) {
    console.error(e);
  }
}

async function sendCommand(cmd) {
  if (writer) {
    await writer.write(cmd + "\n");
    logToConsole("Sent: " + cmd);
  }
}

// 2. Keyboard Listeners for Drive
const keyMap = {
  w: "MOVE_F",
  a: "MOVE_L",
  s: "MOVE_S",
  d: "MOVE_R",
  " ": "STOP",
};
window.addEventListener("keydown", (e) => {
  const cmd = keyMap[e.key.toLowerCase()];
  if (cmd) {
    sendCommand(cmd);
    const btn = document.getElementById(
      `btn-${e.key === " " ? "Space" : e.key.toUpperCase()}`
    );
    if (btn) {
      btn.classList.add("active-btn");
      logToConsole(`Command: ${e.key}`);
    }
  }
});
window.addEventListener("keyup", (e) => {
  const btn = document.getElementById(
    `btn-${e.key === " " ? "Space" : e.key.toUpperCase()}`
  );
  if (btn) btn.classList.remove("active-btn");
});

const manager = nipplejs.create({
  zone: document.getElementById("joy-container"),
  mode: "static",
  position: { left: "50%", top: "50%" },
  color: "#00ffcc",
});

manager.on("move", (evt, data) => {
  if (data.angle) {
    logToConsole(`Radian: ${data.angle.radian}`);

    // Sends AXIS:Angle:Distance (e.g., AXIS:180:50)
    sendCommand(
      `AXIS:${Math.round(data.angle.degree)}:${Math.round(data.distance)}`
    );
  }
});

manager.on("end", () => sendCommand("AXIS:CENTER"));

// 4. GPS & Telemetry
function sendGPS() {
  const val = document.getElementById("gpsInput").value;
  sendCommand(`GOTO:${val}`);
}

async function readStream() {
  const decoder = new TextDecoderStream();
  port.readable.pipeTo(decoder.writable);
  const reader = decoder.readable.getReader();
  while (true) {
    const { value, done } = await reader.read();
    if (done) break;
    if (value) logToConsole("[ROVER]: " + value);
  }
}

function logToConsole(msg) {
  const c = document.getElementById("console");
  c.innerHTML += `<div>${msg}</div>`;
  c.scrollTop = c.scrollHeight;
}
