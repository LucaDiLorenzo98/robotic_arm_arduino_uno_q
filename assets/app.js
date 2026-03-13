// SPDX-FileCopyrightText: Copyright (C) ARDUINO SRL (http://www.arduino.cc)
// SPDX-License-Identifier: MPL-2.0

const recentDetectionsElement = document.getElementById('recentDetections');
const feedbackContentElement = document.getElementById('feedback-content');
const stateContentElement = document.getElementById('state-content');
const stateTimelineElement = document.getElementById('stateTimeline');
const MAX_RECENT_SCANS = 5;
let scans = [];
const socket = io(`http://${window.location.host}`);
let errorContainer = document.getElementById('error-container');
let faceVisible = false;
let feedbackTimeout;

const PRESSURE_POLL_MS = 250; // realtime-like pressure refresh
const STATE_POLL_MS = 200; // realtime-like state refresh
const SERVO_POS_POLL_MS = 150; // realtime-like servo position refresh
const MATRIX_STATUS_POLL_MS = 8000;
const SERVO_IDS = [1, 2, 3, 4];
const sliderDragState = { 1: false, 2: false, 3: false, 4: false };
let setHomeButton;
let setHomeStatus;
let setFirstSlotButton;
let setFirstSlotStatus;
let goHomeButton;
let goFirstSlotButton;
let stateCheckpointButtons = [];
let matrixIntensitySlider;
let matrixIntensityValue;
let matrixAvailabilityEl;
let matrixStatusEl;
let matrixClearButton;

document.addEventListener('DOMContentLoaded', () => {
    initSocketIO();
    renderDetections();
    initStateTimeline();
    startPressurePolling();
    startStatePolling();
    startServoPollingFallback();
    initServoSliders();
    initSetHomeButton();
    initSetFirstSlotButton();
    initGoToPoseButtons();
    initLedMatrixControls();
});

function updateStateDisplay(state) {
    if (!stateContentElement) return;
    const s = state === 'setup' || state === 'detect' || state === 'grab' || state === 'release' ? state : 'detect';
    stateContentElement.textContent = 'Current: ' + s;
    stateContentElement.className = 'state-content state-' + s;
    updateTimelineClasses(s);
}

function updateTimelineClasses(current) {
    if (!stateCheckpointButtons.length) return;
    const order = ['setup', 'detect', 'grab', 'release'];
    const currentIdx = order.indexOf(current);
    stateCheckpointButtons.forEach((btn) => {
        const s = btn.dataset.state;
        const idx = order.indexOf(s);
        btn.classList.toggle('is-current', s === current);
        btn.classList.toggle('is-completed', idx >= 0 && currentIdx >= 0 && idx < currentIdx);
    });
}

function initStateTimeline() {
    if (!stateTimelineElement) return;
    stateCheckpointButtons = Array.from(stateTimelineElement.querySelectorAll('.state-checkpoint'));
    stateCheckpointButtons.forEach((btn) => {
        btn.addEventListener('click', () => {
            const targetState = btn.dataset.state;
            if (!targetState) return;
            fetch(`/api/set_state?state=${encodeURIComponent(targetState)}`)
                .then((res) => res.json())
                .then((data) => {
                    if (data && data.ok && data.state) {
                        updateStateDisplay(data.state);
                    }
                })
                .catch(() => {});
        });
    });

    // Sync immediato al boot: evita di mostrare timeline "spenta"
    fetch('/api/state')
        .then((res) => res.json())
        .then((data) => {
            if (data && data.state) updateStateDisplay(data.state);
        })
        .catch(() => {});
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

function startServoPollingFallback() {
    function poll() {
        fetch('/api/servo_positions')
            .then((res) => res.json())
            .then((data) => applyServoPositions(data, false))
            .catch(() => {});
    }
    poll();
    setInterval(poll, SERVO_POS_POLL_MS);
}

function unwrapMessage(payload) {
    if (payload && typeof payload === 'object' && payload.message !== undefined) {
        return payload.message;
    }
    return payload;
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
        const msg = unwrapMessage(payload);
        const s = msg && typeof msg === 'object' && msg.state != null ? msg.state : msg;
        if (s === 'setup' || s === 'detect' || s === 'grab' || s === 'release') updateStateDisplay(s);
    });

    socket.on('pressure', (payload) => {
        const msg = unwrapMessage(payload);
        if (!msg || typeof msg !== 'object') return;
        const elA0 = document.getElementById('pressure-a0');
        const elA1 = document.getElementById('pressure-a1');
        if (!elA0 || !elA1) return;
        elA0.textContent = msg.a0 != null ? String(msg.a0) : '—';
        elA1.textContent = msg.a1 != null ? String(msg.a1) : '—';
    });

    socket.on('servo_positions', (payload) => {
        const msg = unwrapMessage(payload);
        if (!msg || typeof msg !== 'object') return;
        applyServoPositions(msg, true);
    });

    socket.on('home_updated', (payload) => {
        const msg = unwrapMessage(payload);
        const home = msg && typeof msg === 'object' ? msg.home : null;
        if (home && typeof home === 'object') {
            setHomeStatusText('Home salvata: ' + formatHome(home), false);
        } else {
            setHomeStatusText('Home aggiornata', false);
        }
    });

    socket.on('first_slot_updated', (payload) => {
        const msg = unwrapMessage(payload);
        const firstSlot = msg && typeof msg === 'object' ? msg.first_slot : null;
        if (firstSlot && typeof firstSlot === 'object') {
            setFirstSlotStatusText('First slot RAM: ' + formatHome(firstSlot), false);
        } else {
            setFirstSlotStatusText('First slot aggiornato', false);
        }
    });

    socket.on('detection', (payload) => {
        const message = unwrapMessage(payload);
        clearTimeout(feedbackTimeout);
        if (message && typeof message === 'object') {
            pushDetection(message);
        }
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
                '<p class="feedback-text">Watching camera. After setup, when a face is detected, state goes to grab → detect.</p>';
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
                    slider.addEventListener('pointerdown', () => { sliderDragState[id] = true; });
                    slider.addEventListener('pointerup', () => { sliderDragState[id] = false; });
                    slider.addEventListener('pointercancel', () => { sliderDragState[id] = false; });
                    slider.addEventListener('touchstart', () => { sliderDragState[id] = true; }, { passive: true });
                    slider.addEventListener('touchend', () => { sliderDragState[id] = false; }, { passive: true });
                    slider.addEventListener('mouseup', () => { sliderDragState[id] = false; });
                    slider.addEventListener('mouseleave', () => { sliderDragState[id] = false; });
                }
            }
        })
        .catch(() => {
            container.querySelectorAll('.servo-value').forEach((el) => { el.textContent = '—'; });
        });
}

function applyServoPositions(data, fromRealtime) {
    if (!data || typeof data !== 'object') return;
    SERVO_IDS.forEach((id) => {
        const slider = document.getElementById('servo' + id);
        const valueEl = document.getElementById('servo' + id + '-value');
        if (!slider || !valueEl) return;
        const raw = data[String(id)];
        if (typeof raw !== 'number' || raw < STS_POS_MIN || raw > STS_POS_MAX) return;

        // Non sovrascrive durante drag utente quando arriva telemetria realtime.
        if (fromRealtime && sliderDragState[id]) return;

        slider.value = raw;
        valueEl.textContent = raw;
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

function formatHome(home) {
    const parts = [];
    SERVO_IDS.forEach((id) => {
        const raw = home[String(id)];
        if (typeof raw === 'number') {
            parts.push(`${id}:${raw}`);
        }
    });
    return parts.length ? parts.join('  ') : 'n/d';
}

function setHomeStatusText(text, isError) {
    if (!setHomeStatus) return;
    setHomeStatus.textContent = text;
    setHomeStatus.style.color = isError ? '#f88' : '#aaa';
}

function setFirstSlotStatusText(text, isError) {
    if (!setFirstSlotStatus) return;
    setFirstSlotStatus.textContent = text;
    setFirstSlotStatus.style.color = isError ? '#f88' : '#aaa';
}

function initSetHomeButton() {
    setHomeButton = document.getElementById('setHomeButton');
    setHomeStatus = document.getElementById('setHomeStatus');
    if (!setHomeButton) return;

    setHomeButton.addEventListener('click', () => {
        setHomeButton.disabled = true;
        setHomeStatusText('Salvataggio home in corso...', false);

        fetch('/api/set_home_current')
            .then((res) => res.json())
            .then((data) => {
                if (data && data.ok) {
                    const home = data.home && typeof data.home === 'object' ? data.home : null;
                    if (home) {
                        setHomeStatusText('Home salvata: ' + formatHome(home), false);
                    } else {
                        setHomeStatusText('Home salvata', false);
                    }
                } else {
                    const err = data && data.error ? data.error : 'Errore salvataggio home';
                    setHomeStatusText(err, true);
                }
            })
            .catch(() => setHomeStatusText('Errore rete durante Set Home', true))
            .finally(() => {
                setHomeButton.disabled = false;
            });
    });
}

function initSetFirstSlotButton() {
    setFirstSlotButton = document.getElementById('setFirstSlotButton');
    setFirstSlotStatus = document.getElementById('setFirstSlotStatus');
    if (!setFirstSlotButton) return;

    setFirstSlotButton.addEventListener('click', () => {
        setFirstSlotButton.disabled = true;
        setFirstSlotStatusText('Salvataggio first slot in RAM...', false);

        fetch('/api/set_first_slot_current')
            .then((res) => res.json())
            .then((data) => {
                if (data && data.ok) {
                    const firstSlot =
                        data.first_slot && typeof data.first_slot === 'object' ? data.first_slot : null;
                    if (firstSlot) {
                        setFirstSlotStatusText('First slot RAM: ' + formatHome(firstSlot), false);
                    } else {
                        setFirstSlotStatusText('First slot salvato in RAM', false);
                    }
                } else {
                    const err = data && data.error ? data.error : 'Errore salvataggio first slot';
                    setFirstSlotStatusText(err, true);
                }
            })
            .catch(() => setFirstSlotStatusText('Errore rete durante Set First Slot', true))
            .finally(() => {
                setFirstSlotButton.disabled = false;
            });
    });
}

function initGoToPoseButtons() {
    goHomeButton = document.getElementById('goHomeButton');
    goFirstSlotButton = document.getElementById('goFirstSlotButton');

    if (goHomeButton) {
        goHomeButton.addEventListener('click', () => {
            goHomeButton.disabled = true;
            setHomeStatusText('Movimento verso Home...', false);
            fetch('/api/go_home')
                .then((res) => res.json())
                .then((data) => {
                    if (data && data.ok) {
                        setHomeStatusText('Raggiunta Home', false);
                    } else {
                        const err = data && data.error ? data.error : 'Errore Go To Home';
                        setHomeStatusText(err, true);
                    }
                })
                .catch(() => setHomeStatusText('Errore rete durante Go To Home', true))
                .finally(() => {
                    goHomeButton.disabled = false;
                });
        });
    }

    if (goFirstSlotButton) {
        goFirstSlotButton.addEventListener('click', () => {
            goFirstSlotButton.disabled = true;
            setFirstSlotStatusText('Movimento verso First Slot...', false);
            fetch('/api/go_first_slot')
                .then((res) => res.json())
                .then((data) => {
                    if (data && data.ok) {
                        setFirstSlotStatusText('Raggiunto First Slot', false);
                    } else {
                        const err = data && data.error ? data.error : 'Errore Go To First Slot';
                        setFirstSlotStatusText(err, true);
                    }
                })
                .catch(() => setFirstSlotStatusText('Errore rete durante Go To First Slot', true))
                .finally(() => {
                    goFirstSlotButton.disabled = false;
                });
        });
    }
}

function setMatrixStatusText(text, isError) {
    if (!matrixStatusEl) return;
    matrixStatusEl.textContent = text;
    matrixStatusEl.style.color = isError ? '#f88' : '#aaa';
}

function refreshLedMatrixStatus() {
    fetch('/api/led_matrix_status')
        .then((res) => res.json())
        .then((data) => {
            const available = !!(data && data.available);
            if (matrixAvailabilityEl) {
                matrixAvailabilityEl.textContent = available ? 'available' : 'not detected';
                matrixAvailabilityEl.style.color = available ? '#4ecca3' : '#ffc107';
            }
            if (matrixIntensitySlider && data && data.intensity != null) {
                matrixIntensitySlider.value = String(data.intensity);
            }
            if (matrixIntensityValue && data && data.intensity != null) {
                matrixIntensityValue.textContent = String(data.intensity);
            }
            if (matrixIntensitySlider) matrixIntensitySlider.disabled = !available;
            if (matrixClearButton) matrixClearButton.disabled = !available;
        })
        .catch(() => {
            if (matrixAvailabilityEl) {
                matrixAvailabilityEl.textContent = 'error';
                matrixAvailabilityEl.style.color = '#f88';
            }
        });
}

function initLedMatrixControls() {
    matrixIntensitySlider = document.getElementById('matrixIntensity');
    matrixIntensityValue = document.getElementById('matrix-intensity-value');
    matrixAvailabilityEl = document.getElementById('matrix-availability');
    matrixStatusEl = document.getElementById('matrix-status');
    matrixClearButton = document.getElementById('matrixClearButton');

    if (matrixIntensitySlider && matrixIntensityValue) {
        matrixIntensitySlider.addEventListener('input', () => {
            matrixIntensityValue.textContent = matrixIntensitySlider.value;
        });
        matrixIntensitySlider.addEventListener('change', () => {
            const value = parseInt(matrixIntensitySlider.value, 10);
            fetch(`/api/led_matrix_intensity?value=${value}`)
                .then((res) => res.json())
                .then((data) => {
                    if (data && data.ok) {
                        setMatrixStatusText(`Brightness: ${value}`, false);
                    } else {
                        const err = data && data.error ? data.error : 'Matrix intensity error';
                        setMatrixStatusText(err, true);
                    }
                })
                .catch(() => setMatrixStatusText('Network error on matrix intensity', true));
        });
    }

    if (matrixClearButton) {
        matrixClearButton.addEventListener('click', () => {
            fetch('/api/led_matrix_clear')
                .then((res) => res.json())
                .then((data) => {
                    if (data && data.ok) setMatrixStatusText('Matrix cleared', false);
                    else setMatrixStatusText('Matrix clear failed', true);
                })
                .catch(() => setMatrixStatusText('Network error on matrix clear', true));
        });
    }

    refreshLedMatrixStatus();
    setInterval(refreshLedMatrixStatus, MATRIX_STATUS_POLL_MS);
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
