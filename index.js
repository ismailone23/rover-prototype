let port, writer, reader;
let currentSpeed = 75;
let currentAngle = 90;
let map, vehicleMarker, destinationMarker;
let pathPolyline, destinationLine;
let currentLat = null;
let currentLon = null;
let currentHeading = null;
let isSatelliteView = false;
let isSettingDestination = false;
let pathCoordinates = [];

// Initialize Leaflet map
function initMap() {
  // Create map centered on Dhaka, Bangladesh
  map = L.map("map").setView([23.838766, 90.35869], 18);

  // Add OpenStreetMap tile layer
  L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    attribution: "© OpenStreetMap contributors",
    maxZoom: 19,
  }).addTo(map);

  // Create custom vehicle icon (arrow)
  const vehicleIcon = L.divIcon({
    className: "vehicle-icon",
    html: `<div style="
          width: 30px;
          height: 30px;
          background: #4285F4;
          border: 3px solid white;
          border-radius: 50%;
          box-shadow: 0 2px 8px rgba(0,0,0,0.3);
          display: flex;
          align-items: center;
          justify-content: center;
          font-size: 16px;
          transform: rotate(0deg);
        ">🚗</div>`,
    iconSize: [30, 30],
    iconAnchor: [15, 15],
  });

  vehicleMarker = L.marker([23.838766, 90.35869], {
    icon: vehicleIcon,
    title: "vehicle Position",
  });

  pathPolyline = L.polyline([], {
    color: "#4285F4",
    weight: 3,
    opacity: 0.8,
  }).addTo(map);

  log("Map initialized - FREE OpenStreetMap!");
}

// Update vehicle position on map
function updateMapPosition(lat, lon, heading) {
  if (!map) return;

  const position = [parseFloat(lat), parseFloat(lon)];

  if (!map.hasLayer(vehicleMarker)) {
    vehicleMarker.addTo(map);
  }

  vehicleMarker.setLatLng(position);

  if (heading !== null) {
    const icon = vehicleMarker.getElement();
    if (icon) {
      const innerDiv = icon.querySelector("div");
      if (innerDiv) {
        innerDiv.style.transform = `rotate(${heading}deg)`;
      }
    }
  }

  pathCoordinates.push(position);
  pathPolyline.setLatLngs(pathCoordinates);

  vehicleMarker.bindPopup(`
        <b>vehicle Position</b><br>
        Lat: ${lat.toFixed(6)}<br>
        Lon: ${lon.toFixed(6)}<br>
        Heading: ${heading ? heading.toFixed(1) + "°" : "N/A"}
      `);
}

function centerOnvehicle() {
  if (currentLat && currentLon) {
    map.setView([parseFloat(currentLat), parseFloat(currentLon)], 18);
    log("Centered on vehicle");
  } else {
    alert("No GPS data available yet");
  }
}

function setDestinationFromMap() {
  if (isSettingDestination) {
    isSettingDestination = false;
    map.getContainer().style.cursor = "";
    log("Destination mode cancelled");
    return;
  }

  isSettingDestination = true;
  map.getContainer().style.cursor = "crosshair";
  log("Click on map to set destination...");

  const clickHandler = (e) => {
    const lat = e.latlng.lat;
    const lon = e.latlng.lng;

    document.getElementById("targetLat").value = lat.toFixed(6);
    document.getElementById("targetLon").value = lon.toFixed(6);

    if (destinationMarker) {
      map.removeLayer(destinationMarker);
    }

    const destIcon = L.divIcon({
      className: "dest-icon",
      html: `<div style="
            width: 30px;
            height: 30px;
            border: 3px solid white;
            border-radius: 50%;
            box-shadow: 0 2px 8px rgba(0,0,0,0.3);
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 16px;
          ">📍</div>`,
      iconSize: [30, 30],
      iconAnchor: [15, 15],
    });

    destinationMarker = L.marker([lat, lon], { icon: destIcon }).addTo(map)
      .bindPopup(`
            <b>Destination</b><br>
            Lat: ${lat.toFixed(6)}<br>
            Lon: ${lon.toFixed(6)}
          `);

    if (currentLat && currentLon) {
      if (destinationLine) {
        map.removeLayer(destinationLine);
      }
      destinationLine = L.polyline(
        [
          [parseFloat(currentLat), parseFloat(currentLon)],
          [lat, lon],
        ],
        {
          color: "#34A853",
          weight: 2,
          opacity: 0.6,
          dashArray: "10, 10",
        }
      ).addTo(map);
    }

    log(`Destination set: ${lat.toFixed(6)}, ${lon.toFixed(6)}`);
    isSettingDestination = false;
    map.getContainer().style.cursor = "";
    map.off("click", clickHandler);
  };

  map.once("click", clickHandler);
}

function clearPath() {
  pathCoordinates = [];
  pathPolyline.setLatLngs([]);

  if (destinationMarker) {
    map.removeLayer(destinationMarker);
    destinationMarker = null;
  }

  if (destinationLine) {
    map.removeLayer(destinationLine);
    destinationLine = null;
  }

  log("Path and destination cleared");
}

function toggleSatellite() {
  map.eachLayer((layer) => {
    if (layer instanceof L.TileLayer) {
      map.removeLayer(layer);
    }
  });

  isSatelliteView = !isSatelliteView;

  if (isSatelliteView) {
    L.tileLayer(
      "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      {
        attribution: "Tiles © Esri",
        maxZoom: 19,
      }
    ).addTo(map);
    log("Switched to satellite view");
  } else {
    L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
      attribution: "© OpenStreetMap contributors",
      maxZoom: 19,
    }).addTo(map);
    log("Switched to map view");
  }
}

// Serial connection
async function connectSerial() {
  if (!navigator.serial)
    return alert("Web Serial not supported in this browser.");
  try {
    port = await navigator.serial.requestPort();
    await port.open({ baudRate: 115200 });

    const textEncoder = new TextEncoderStream();
    const writableStreamClosed = textEncoder.readable.pipeTo(port.writable);
    writer = textEncoder.writable.getWriter();

    document.getElementById("connectBtn").innerText = "✅ Connected";
    document.getElementById("connectBtn").style.background = "#28a745";

    readLoop();
  } catch (err) {
    console.error(err);
    alert("Connection failed");
  }
}

async function sendJson(obj) {
  if (!writer) return;
  const str = JSON.stringify(obj) + "\n";
  await writer.write(str);
  if (obj.cmd !== "servo") {
    log("TX: " + str);
  }
}

function startMove(dir) {
  sendJson({ cmd: dir, spd: parseInt(currentSpeed) });
}

function stopMove() {
  sendJson({ cmd: "x" });
}

function updateSpeed(val) {
  currentSpeed = val;
  document.getElementById("speedVal").innerText = val;
}

function sendServo(angle) {
  currentAngle = parseInt(angle);
  sendJson({ cmd: "servo", ang: currentAngle });
  updateSteeringWheel(currentAngle);
}

function updateSteeringWheel(angle) {
  const wheel = document.getElementById("steeringWheel");
  const indicator = document.getElementById("indicator");
  const s1Val = document.getElementById("s1Val");

  if (!wheel) return;

  const rotation = (angle - 90) * 2;

  wheel.style.transform = `rotate(${rotation}deg)`;
  s1Val.textContent = angle;
}

function steerLeft() {
  let newAngle = Math.max(45, currentAngle - 15);
  sendServo(newAngle);
}

function steerCenter() {
  sendServo(90);
}

function steerRight() {
  let newAngle = Math.min(135, currentAngle + 15);
  sendServo(newAngle);
}

function startAutoDrive() {
  const lat = parseFloat(document.getElementById("targetLat").value);
  const lon = parseFloat(document.getElementById("targetLon").value);

  if (isNaN(lat) || isNaN(lon)) {
    alert("Please enter valid coordinates");
    return;
  }

  sendJson({ cmd: "auto", lat: lat, lon: lon });
  log("Auto-drive started to: " + lat.toFixed(6) + ", " + lon.toFixed(6));
}

function stopAutoDrive() {
  sendJson({ cmd: "stop_auto" });
  log("Auto-drive stopped");
}

async function readLoop() {
  const textDecoder = new TextDecoderStream();
  const readableStreamClosed = port.readable.pipeTo(textDecoder.writable);
  const reader = textDecoder.readable.getReader();

  try {
    while (true) {
      const { value, done } = await reader.read();
      if (done) break;
      if (value) handleData(value);
    }
  } catch (error) {
    console.error(error);
  }
}

let buffer = "";
function handleData(chunk) {
  buffer += chunk;
  let lines = buffer.split("\n");
  buffer = lines.pop();

  for (let line of lines) {
    try {
      if (line.includes("{")) {
        const data = JSON.parse(line);

        // Update sensors
        if (data.d !== undefined)
          document.getElementById("dist").innerText = data.d;
        if (data.t !== undefined)
          document.getElementById("temp").innerText = data.t;
        if (data.h !== undefined)
          document.getElementById("humid").innerText = data.h;

        // Update GPS
        if (data.lat !== undefined && data.lat !== null) {
          currentLat = data.lat;
          document.getElementById("latitude").innerText = data.lat.toFixed(6);
        }
        if (data.lon !== undefined && data.lon !== null) {
          currentLon = data.lon;
          document.getElementById("longitude").innerText = data.lon.toFixed(6);
        }

        // Update heading
        if (data.hdg !== undefined && data.hdg !== null) {
          currentHeading = data.hdg;
          document.getElementById("heading").innerText = data.hdg.toFixed(1);
        }

        // Update auto status
        if (data.auto !== undefined) {
          const statusEl = document.getElementById("autoStatus");
          if (data.auto) {
            statusEl.innerHTML =
              '<span class="status-indicator active"></span>ON';
          } else {
            statusEl.innerHTML =
              '<span class="status-indicator inactive"></span>OFF';
          }
        }

        // Update map
        if (currentLat && currentLon) {
          updateMapPosition(currentLat, currentLon, currentHeading);
        }
      }
    } catch (e) {
      console.log("Parse error:", e);
    }
  }
}

function log(msg) {
  const logDiv = document.getElementById("log");
  if (!logDiv) return;
  const time = new Date().toLocaleTimeString();
  logDiv.innerHTML += `<div>[${time}] ${msg}</div>`;
  logDiv.scrollTop = logDiv.scrollHeight;
}

document.addEventListener("keydown", (e) => {
  if (e.repeat) return;
  const k = e.key.toLowerCase();

  if (k === "w" || k === "s") startMove(k);
  if (k === "a" || k === "d") startMove(k);
  if (k === " ") {
    e.preventDefault();
    stopMove();
  }

  if (e.key === "ArrowLeft") {
    e.preventDefault();
    steerLeft();
  }
  if (e.key === "ArrowRight") {
    e.preventDefault();
    steerRight();
  }
  if (e.key.toLowerCase() === "c") {
    steerCenter();
  }
});

document.addEventListener("keyup", (e) => {
  const k = e.key.toLowerCase();
  if (["w", "a", "s", "d"].includes(k)) stopMove();
});

window.addEventListener("DOMContentLoaded", function () {
  setTimeout(function () {
    if (typeof L !== "undefined") {
      initMap();
    } else {
      console.error("Leaflet failed to load");
      alert("Map library failed to load. Please refresh the page.");
    }
  }, 100);
});
