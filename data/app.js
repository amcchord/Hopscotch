(function() {
    "use strict";

    var ws = null;
    var reconnectTimer = null;
    var settingsCache = null;

    // Motor role mapping: index -> element id
    var motorElements = [
        {id: "motor-fr", role: "FrontRight"},
        {id: "motor-br", role: "BackRight"},
        {id: "motor-bl", role: "BackLeft"},
        {id: "motor-fl", role: "FrontLeft"},
        {id: "motor-la", role: "ArmLeft"},
        {id: "motor-ra", role: "ArmRight"}
    ];

    function connectWebSocket() {
        var host = window.location.hostname || "192.168.1.1";
        var url = "ws://" + host + "/ws";
        ws = new WebSocket(url);

        ws.onopen = function() {
            document.getElementById("connection-status").className = "status-dot connected";
            if (reconnectTimer) {
                clearInterval(reconnectTimer);
                reconnectTimer = null;
            }
        };

        ws.onclose = function() {
            document.getElementById("connection-status").className = "status-dot disconnected";
            scheduleReconnect();
        };

        ws.onerror = function() {
            ws.close();
        };

        ws.onmessage = function(event) {
            try {
                var data = JSON.parse(event.data);
                updateTelemetry(data);
            } catch(e) {
                // ignore
            }
        };
    }

    function scheduleReconnect() {
        if (reconnectTimer) return;
        reconnectTimer = setInterval(function() {
            if (!ws || ws.readyState === WebSocket.CLOSED) {
                connectWebSocket();
            }
        }, 3000);
    }

    function updateTelemetry(data) {
        // Link / RSSI / LQ
        var linkEl = document.getElementById("link-status");
        if (data.link_up) {
            linkEl.textContent = "UP";
            linkEl.style.color = "var(--green)";
        } else {
            linkEl.textContent = "DOWN";
            linkEl.style.color = "var(--red)";
        }

        document.getElementById("rssi").textContent = data.rssi + " dBm";
        document.getElementById("lq").textContent = data.lq + "%";

        // Armed status
        var driveEl = document.getElementById("drive-armed");
        driveEl.textContent = data.drive_armed ? "ARMED" : "DISARMED";
        driveEl.className = "value " + (data.drive_armed ? "armed" : "disarmed");

        var armEl = document.getElementById("arm-armed");
        armEl.textContent = data.arm_armed ? "ARMED" : "DISARMED";
        armEl.className = "value " + (data.arm_armed ? "armed" : "disarmed");

        // Motors
        if (data.motors) {
            updateMotors(data.motors);
        }

        // Channels
        if (data.channels) {
            updateChannels(data.channels);
        }
    }

    function updateMotors(motors) {
        var tbody = document.querySelector("#motor-details tbody");
        tbody.innerHTML = "";

        for (var i = 0; i < motors.length; i++) {
            var m = motors[i];

            // Update diagram boxes
            if (i < motorElements.length) {
                var el = document.getElementById(motorElements[i].id);
                if (el) {
                    el.className = "motor-box";
                    if (!m.online) {
                        el.classList.add("offline");
                    } else if (m.error > 0) {
                        el.classList.add("fault");
                    } else if (!m.enabled) {
                        el.classList.add("disabled");
                    } else {
                        el.classList.add("online");
                    }
                    el.querySelector(".motor-id").textContent = "ID:" + m.id;
                }
            }

            // Table row
            var tr = document.createElement("tr");
            tr.innerHTML =
                "<td>" + m.id + "</td>" +
                "<td>" + m.pos + "</td>" +
                "<td>" + m.vel + "</td>" +
                "<td>" + m.torque + "</td>" +
                "<td>" + m.temp + "</td>" +
                "<td>" + (m.error > 0 ? "0x" + m.error.toString(16) : "OK") + "</td>";
            tbody.appendChild(tr);
        }
    }

    function updateChannels(channels) {
        var container = document.getElementById("channel-bars");
        // Build bars on first call, then just update
        if (container.children.length === 0) {
            for (var i = 0; i < channels.length; i++) {
                var row = document.createElement("div");
                row.className = "channel-bar";
                row.innerHTML =
                    '<span class="ch-label">CH' + (i+1) + '</span>' +
                    '<div class="bar-track">' +
                        '<div class="bar-center"></div>' +
                        '<div class="bar-fill" id="bar-fill-' + i + '"></div>' +
                    '</div>' +
                    '<span class="ch-value" id="ch-val-' + i + '"></span>';
                container.appendChild(row);
            }
        }

        for (var j = 0; j < channels.length; j++) {
            var raw = channels[j];
            var norm = (raw - 992) / (1811 - 992);
            if (norm < -1) norm = -1;
            if (norm > 1) norm = 1;

            var fill = document.getElementById("bar-fill-" + j);
            var valEl = document.getElementById("ch-val-" + j);

            if (fill) {
                var pct = Math.abs(norm) * 50;
                if (norm >= 0) {
                    fill.style.left = "50%";
                    fill.style.width = pct + "%";
                } else {
                    fill.style.left = (50 - pct) + "%";
                    fill.style.width = pct + "%";
                }
            }
            if (valEl) {
                valEl.textContent = raw;
            }
        }
    }

    // Settings
    function loadSettings() {
        fetch("/api/settings")
            .then(function(r) { return r.json(); })
            .then(function(data) {
                settingsCache = data;
                populateSettingsForm(data);
            })
            .catch(function(e) { console.error("Failed to load settings:", e); });
    }

    function populateSettingsForm(s) {
        document.getElementById("set-wifi-ssid").value = s.wifi_ssid || "";
        document.getElementById("set-wifi-pass").value = s.wifi_password || "";

        var ch = s.channel_map || {};
        document.getElementById("set-ch-steering").value = ch.steering || 0;
        document.getElementById("set-ch-throttle").value = ch.throttle || 0;
        document.getElementById("set-ch-arm-drive").value = ch.arm_disarm_drive || 0;
        document.getElementById("set-ch-arm-arms").value = ch.arm_disarm_arms || 0;
        document.getElementById("set-ch-arm-left").value = ch.arm_left || 0;
        document.getElementById("set-ch-arm-right").value = ch.arm_right || 0;

        document.getElementById("set-deadband").value = s.deadband || 50;
        document.getElementById("set-max-speed").value = s.max_drive_speed || 10;
        document.getElementById("set-horizon").value = s.position_horizon_sec || 2;
        document.getElementById("set-arm-speed").value = s.max_arm_speed || 5;

        var mi = s.motor_ids || {};
        document.getElementById("set-mid-fr").value = mi.front_right || 10;
        document.getElementById("set-mid-br").value = mi.back_right || 20;
        document.getElementById("set-mid-bl").value = mi.back_left || 30;
        document.getElementById("set-mid-fl").value = mi.front_left || 40;
        document.getElementById("set-mid-la").value = mi.arm_left || 1;
        document.getElementById("set-mid-ra").value = mi.arm_right || 2;
    }

    function saveSettings() {
        var payload = {
            wifi_ssid: document.getElementById("set-wifi-ssid").value,
            wifi_password: document.getElementById("set-wifi-pass").value,
            channel_map: {
                steering: parseInt(document.getElementById("set-ch-steering").value),
                throttle: parseInt(document.getElementById("set-ch-throttle").value),
                arm_disarm_drive: parseInt(document.getElementById("set-ch-arm-drive").value),
                arm_disarm_arms: parseInt(document.getElementById("set-ch-arm-arms").value),
                arm_left: parseInt(document.getElementById("set-ch-arm-left").value),
                arm_right: parseInt(document.getElementById("set-ch-arm-right").value)
            },
            deadband: parseInt(document.getElementById("set-deadband").value),
            max_drive_speed: parseFloat(document.getElementById("set-max-speed").value),
            position_horizon_sec: parseFloat(document.getElementById("set-horizon").value),
            max_arm_speed: parseFloat(document.getElementById("set-arm-speed").value),
            motor_ids: {
                front_right: parseInt(document.getElementById("set-mid-fr").value),
                back_right: parseInt(document.getElementById("set-mid-br").value),
                back_left: parseInt(document.getElementById("set-mid-bl").value),
                front_left: parseInt(document.getElementById("set-mid-fl").value),
                arm_left: parseInt(document.getElementById("set-mid-la").value),
                arm_right: parseInt(document.getElementById("set-mid-ra").value)
            }
        };

        fetch("/api/settings", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify(payload)
        })
        .then(function(r) { return r.json(); })
        .then(function(data) {
            alert("Settings saved! Some changes may require reboot.");
        })
        .catch(function(e) { alert("Failed to save settings: " + e); });
    }

    function disarmAll() {
        if (!confirm("Disarm all motors?")) return;
        fetch("/api/disarm", {method: "POST"})
            .then(function(r) { return r.json(); })
            .then(function() { /* telemetry will update */ })
            .catch(function(e) { alert("Disarm failed: " + e); });
    }

    function changeCanId() {
        var oldId = parseInt(document.getElementById("canid-old").value);
        var newId = parseInt(document.getElementById("canid-new").value);
        if (!oldId || !newId) {
            alert("Enter both old and new CAN IDs");
            return;
        }
        if (!confirm("Change motor CAN ID from " + oldId + " to " + newId + "? Motor must be power-cycled after.")) return;

        fetch("/api/change-can-id", {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({oldId: oldId, newId: newId})
        })
        .then(function(r) { return r.json(); })
        .then(function(data) {
            if (data.status === "ok") {
                alert("CAN ID changed. Power-cycle the motor now.");
            } else {
                alert("Failed: " + (data.error || "unknown"));
            }
        })
        .catch(function(e) { alert("Error: " + e); });
    }

    function resetSettings() {
        if (!confirm("Reset all settings to factory defaults?")) return;
        fetch("/api/reset-settings", {method: "POST"})
            .then(function() { loadSettings(); alert("Settings reset."); })
            .catch(function(e) { alert("Error: " + e); });
    }

    // Init
    document.addEventListener("DOMContentLoaded", function() {
        connectWebSocket();
        loadSettings();

        document.getElementById("btn-disarm").addEventListener("click", disarmAll);
        document.getElementById("btn-save-settings").addEventListener("click", saveSettings);
        document.getElementById("btn-reset-settings").addEventListener("click", resetSettings);
        document.getElementById("btn-change-canid").addEventListener("click", changeCanId);
    });
})();
