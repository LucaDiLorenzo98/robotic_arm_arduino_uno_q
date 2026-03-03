// SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
// SPDX-License-Identifier: MPL-2.0

const recentDetectionsElement = document.getElementById('recentDetections');
const feedbackContentElement = document.getElementById('feedback-content');
const stateContentElement = document.getElementById('state-content');
const MAX_RECENT_SCANS = 5;
let scans = [];
const socket = io(`http://${window.location.host}`);
let errorContainer = document.getElementById('error-container');
let faceVisible = false;
let feedbackTimeout;

const PRESSURE_POLL_MS = 1500;
const STATE_POLL_MS = 800;

document.addEventListener('DOMContentLoaded', () => {
    initSocketIO();
    renderDetections();
    startPressurePolling();
    startStatePolling();
    initServoSliders();
});

function updateStateDisplay(state) {
    if (!stateContentElement) return;
    const s = state === 'detect' || state === 'grab' || state === 'release' ? state : 'detect';
    stateContentElement.textContent = s;
    stateContentElement.className = 'state-content state-' + s;
}

function startStatePolling() {
    function poll() {
        fetch('/api/state')
            .then((res) => res.json())
            .then((data) => {
                if (data && data.state != null) updateStateDisplay(data.state);
            })
            .catch(() => {});
    }
    poll();
    setInterval(poll, STATE_POLL_MS);
}

function startPressurePolling() {
    const elA0 = document.getElementById('pressure-a0');
    const elA1 = document.getElementById('pressure-a1');
    if (!elA0 || !elA1) return;

    function poll() {
        fetch('/api/pressure')
            .then((res) => res.json())
            .then((data) => {
                elA0.textContent = data.a0 != null ? String(data.a0) : '—';
                elA1.textContent = data.a1 != null ? String(data.a1) : '—';
            })
            .catch(() => {
                elA0.textContent = '—';
                elA1.textContent = '—';
            });
    }

    poll();
    setInterval(poll, PRESSURE_POLL_MS);
}

function initSocketIO() {
    socket.on('connect', () => {
        if (errorContainer) {
            errorContainer.style.display = 'none';
            errorContainer.textContent = '';
        }
    });

    socket.on('disconnect', () => {
        if (errorContainer) {
            errorContainer.textContent = 'Connection to the board lost. Please check the connection.';
            errorContainer.style.display = 'block';
        }
    });

    socket.on('state', (payload) => {
        const msg = payload && typeof payload === 'object' && payload.message !== undefined ? payload.message : payload;
        const s = msg && typeof msg === 'object' && msg.state != null ? msg.state : msg;
        if (s === 'detect' || s === 'grab' || s === 'release') updateStateDisplay(s);
    });

    socket.on('detection', (payload) => {
        const message = payload && typeof payload === 'object' && payload.message !== undefined ? payload.message : payload;
        clearTimeout(feedbackTimeout);
        pushDetection(message);
        renderDetections();

        if (!faceVisible) {
            const messages = [
                'Face detected!',
                'Hello!',
                'Face detected – servo moving.',
                'I see you!',
                'Hi there!',
            ];
            const text = messages[Math.floor(Math.random() * messages.length)];
            feedbackContentElement.innerHTML = `<p class="feedback-face">${text}</p>`;
            faceVisible = true;
        }

        feedbackTimeout = setTimeout(() => {
            feedbackContentElement.innerHTML =
                '<p class="feedback-text">Watching camera. When a face is detected, state goes to grab → release → detect.</p>';
            faceVisible = false;
        }, 3000);
    });
}

function pushDetection(detection) {
    scans.unshift(detection);
    if (scans.length > MAX_RECENT_SCANS) scans.pop();
}

const STS_POS_MIN = 0;
const STS_POS_MAX = 4095;

function initServoSliders() {
    const container = document.getElementById('servoSliders');
    if (!container) return;

    // Carica posizioni attuali e inizializza gli slider
    fetch('/api/servo_positions')
        .then((res) => res.json())
        .then((data) => {
            for (let id = 1; id <= 4; id++) {
                const key = String(id);
                const pos = data[key];
                const slider = document.getElementById('servo' + id);
                const valueEl = document.getElementById('servo' + id + '-value');
                if (slider && valueEl) {
                    if (typeof pos === 'number' && pos >= STS_POS_MIN && pos <= STS_POS_MAX) {
                        slider.value = pos;
                        valueEl.textContent = pos;
                    }
                    slider.addEventListener('input', () => onServoSliderInput(id, slider, valueEl));
                    slider.addEventListener('change', () => onServoSliderInput(id, slider, valueEl));
                }
            }
        })
        .catch(() => {
            container.querySelectorAll('.servo-value').forEach((el) => { el.textContent = '—'; });
        });
}

function onServoSliderInput(id, slider, valueEl) {
    const position = parseInt(slider.value, 10);
    valueEl.textContent = position;
    fetch(`/api/servo_move?id=${id}&position=${position}`)
        .then((res) => res.json())
        .then((data) => {
            if (data && !data.ok && data.error) {
                valueEl.title = data.error;
            } else if (valueEl.title) {
                valueEl.title = '';
            }
        })
        .catch(() => { valueEl.title = 'Errore di rete'; });
}

function renderDetections() {
    recentDetectionsElement.innerHTML = '';

    if (scans.length === 0) {
        recentDetectionsElement.innerHTML = '<li class="no-detections">No face detected yet</li>';
        return;
    }

    scans.forEach((scan) => {
        const li = document.createElement('li');
        li.className = 'scan-item';
        const conf = scan.confidence != null ? (Math.round(scan.confidence * 1000) / 10) : '—';
        const time = scan.timestamp ? new Date(scan.timestamp).toLocaleString('it-IT').replace(',', ' –') : '';
        li.innerHTML = `<span class="scan-content">${conf}% – Face</span><span class="scan-time">${time}</span>`;
        recentDetectionsElement.appendChild(li);
    });
}
