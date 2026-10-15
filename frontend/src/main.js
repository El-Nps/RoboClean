import './components/MapCanvas.js';
import './components/Joystick.js';
import { gridBytesToImageData } from './utils/compression.js';

const RECONNECT_DELAY = 5000;
let ws = null;

const mapCanvas      = document.querySelector('map-canvas');
const status         = document.getElementById('status');

const modes          = document.getElementById('modes');
const buttonExplore  = document.getElementById('mode-explore');
const buttonClean    = document.getElementById('mode-clean');
const buttonIdle     = document.getElementById('mode-idle');

const tools          = document.getElementById('tools');
const buttonWaypoint = document.getElementById('tool-waypoint');
const buttonArea     = document.getElementById('tool-area');
const buttonPen      = document.getElementById('tool-pen');
const buttonEraser   = document.getElementById('tool-eraser');
const buttonOutline  = document.getElementById('tool-outline');
const buttonManual   = document.getElementById('tool-manual');

const buttonFocus    = document.getElementById('center-view');
const buttonSaveMap  = document.getElementById('save-map');
const buttonExit     = document.getElementById('exit');

const manualToggles  = document.getElementById('manual-toggles');
const joystick       = document.getElementById('manual-joystick');
const toggleVacuum   = document.getElementById('toggle-vacuum');
const toggleBrush    = document.getElementById('toggle-brush');

let isManualMode   = false;
let isWaypointMode = false;
let isAreaMode     = false;
let isPenMode      = false;
let isEraserMode   = false;

let areaDraftPoints = [];
let roomPolygons = [];
let splitTargetPolygonIndex = null;
let selectedRoomIndices = new Set();
let cleanRooms = [];
let isCleaningQueueRunning = false;
let activeCleanRoomNumber = null;
let activeCleanRoomIndex = null;

let vacuum = false;
let brush  = false;

let currentMapBitmap    = null;
let currentVacuumBitmap = null;
let currentKeepoutBitmap = null;
const STATS = {
    cpu:  document.getElementById('stat-cpu'),
    ram:  document.getElementById('stat-ram'),
    disk: document.getElementById('stat-dsk'),
    coverage: document.getElementById('stat-cov')
};

// HELPERS
const getTheta = ({ qw, qz, qx, qy }) => Math.atan2(
    2 * (qw * qz + qx * qy),
    1 - 2 * (qy * qy + qz * qz)
);

const updateStatus = (text) => {
    if (status) status.textContent = text;
};

const setCoverageStat = (value) => {
    if (!STATS.coverage) return;
    STATS.coverage.textContent = typeof value === 'number' ? value.toFixed(1) : '--';
};

const setRoomPolygons = (polygons) => {
    roomPolygons = Array.isArray(polygons) ? polygons.filter(poly => Array.isArray(poly) && poly.length >= 3) : [];
    mapCanvas.setRoomPolygons(roomPolygons);

    selectedRoomIndices = new Set(
        [...selectedRoomIndices].filter((index) => Number.isInteger(index) && index >= 0 && index < roomPolygons.length)
    );

    mapCanvas.setSelectedRoomIndices([...selectedRoomIndices]);
    rebuildCleanRoomsQueue();
}

const rebuildCleanRoomsQueue = () => {
    const sortedIndices = [...selectedRoomIndices].sort((a, b) => a - b);
    cleanRooms = sortedIndices
        .filter(index => roomPolygons[index] && roomPolygons[index].length >= 3)
        .map(index => ({
            index,
            label: index + 1,
            points: roomPolygons[index]
        }));
}

const distanceSquared = (a, b) => {
    const dx = a.x - b.x;
    const dy = a.y - b.y;
    return dx * dx + dy * dy;
}

const getClosestPointOnSegment = (point, a, b) => {
    const abx = b.x - a.x;
    const aby = b.y - a.y;
    const abLenSq = (abx * abx) + (aby * aby);

    if (abLenSq <= 1e-12) {
        return { point: { x: a.x, y: a.y }, t: 0.0, distSq: distanceSquared(point, a) };
    }

    const apx = point.x - a.x;
    const apy = point.y - a.y;
    const t = Math.max(0, Math.min(1, ((apx * abx) + (apy * aby)) / abLenSq));
    const closest = { x: a.x + t * abx, y: a.y + t * aby };

    return { point: closest, t, distSq: distanceSquared(point, closest) };
}

const getClosestAttachmentOnPolygon = (point, polygon) => {
    const n = polygon.length;
    let best = null;

    for (let i = 0; i < n; i++) {
        const a = polygon[i];
        const b = polygon[(i + 1) % n];
        const candidate = getClosestPointOnSegment(point, a, b);
        if (!best || candidate.distSq < best.distSq) {
            best = {
                edgeIndex: i,
                t: candidate.t,
                distSq: candidate.distSq,
                point: candidate.point
            };
        }
    }

    return best;
}

const getClosestPolygonForPoint = (point, polygons) => {
    if (!Array.isArray(polygons) || polygons.length === 0) return null;

    let best = null;

    for (let i = 0; i < polygons.length; i++) {
        const polygon = polygons[i];
        if (!Array.isArray(polygon) || polygon.length < 3) continue;

        const attachment = getClosestAttachmentOnPolygon(point, polygon);
        if (!attachment) continue;

        if (!best || attachment.distSq < best.attachment.distSq) {
            best = { index: i, attachment };
        }
    }

    return best;
}

const isPointInsidePolygon = (point, polygon) => {
    if (!Array.isArray(polygon) || polygon.length < 3) return false;

    let inside = false;
    for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
        const xi = polygon[i].x;
        const yi = polygon[i].y;
        const xj = polygon[j].x;
        const yj = polygon[j].y;

        const intersects = ((yi > point.y) !== (yj > point.y))
            && (point.x < ((xj - xi) * (point.y - yi)) / ((yj - yi) || 1e-12) + xi);

        if (intersects) inside = !inside;
    }

    return inside;
}

const getClickedRoomIndex = (point) => {
    for (let i = roomPolygons.length - 1; i >= 0; i--) {
        if (isPointInsidePolygon(point, roomPolygons[i])) {
            return i;
        }
    }
    return -1;
}

const setRoomSelected = (index, isSelected) => {
    if (!Number.isInteger(index) || index < 0 || index >= roomPolygons.length) return;

    if (isSelected) selectedRoomIndices.add(index);
    else selectedRoomIndices.delete(index);

    mapCanvas.setSelectedRoomIndices([...selectedRoomIndices]);
    rebuildCleanRoomsQueue();
}

const selectedRoomsCountText = () => `${selectedRoomIndices.size} room${selectedRoomIndices.size === 1 ? '' : 's'} selected`;

const queueLabelsText = () => {
    if (!Array.isArray(cleanRooms) || cleanRooms.length === 0) return '[]';
    return `[${cleanRooms.map(room => room.label).join(', ')}]`;
}

const cleaningQueueStatusText = () => {
    if (typeof activeCleanRoomNumber === 'number') {
        return `cleaning Room ${activeCleanRoomNumber} | queue: ${queueLabelsText()}`;
    }
    return `cleaning... | queue: ${queueLabelsText()}`;
}

const toggleRoomSelection = (index) => {
    const wasSelected = selectedRoomIndices.has(index);
    setRoomSelected(index, !wasSelected);

    if (!wasSelected) {
        updateStatus(`Room ${index + 1} selected for cleaning (${selectedRoomsCountText()})`);
    } else {
        updateStatus(`Room ${index + 1} deselected (${selectedRoomsCountText()})`);
    }
}

const pushNextRoomToCleaner = () => {
    if (!isCleaningQueueRunning) return;

    if (cleanRooms.length === 0) {
        isCleaningQueueRunning = false;
        activeCleanRoomNumber = null;
        activeCleanRoomIndex = null;
        updateStatus('all selected rooms cleaned');
        return;
    }

    const nextRoom = cleanRooms.shift();
    activeCleanRoomNumber = nextRoom.label;
    activeCleanRoomIndex = nextRoom.index;

    sendCommand('cleaning_polygon', { points: nextRoom.points });
    sendCommand('start-vacuum');

    if (buttonClean) buttonClean.checked = true;
    updateStatus(`cleaning Room ${nextRoom.label} | queue: ${queueLabelsText()}`);
    refreshToolsVisibility();
}

const startCleanQueue = () => {
    if (isCleaningQueueRunning) {
        updateStatus('clean queue already running');
        return;
    }

    rebuildCleanRoomsQueue();
    if (cleanRooms.length === 0) {
        updateStatus('no rooms selected for cleaning');
        return;
    }

    isCleaningQueueRunning = true;
    setCoverageStat(null);
    updateStatus(`starting queue: ${queueLabelsText()}`);
    pushNextRoomToCleaner();
}

const collectBoundaryPath = (polygon, startAttach, endAttach) => {
    const n = polygon.length;
    const points = [{ x: startAttach.point.x, y: startAttach.point.y }];

    if (startAttach.edgeIndex === endAttach.edgeIndex && startAttach.t <= endAttach.t) {
        points.push({ x: endAttach.point.x, y: endAttach.point.y });
        return points;
    }

    let vertexIndex = (startAttach.edgeIndex + 1) % n;
    const stopVertex = (endAttach.edgeIndex + 1) % n;

    while (vertexIndex !== stopVertex) {
        points.push({ x: polygon[vertexIndex].x, y: polygon[vertexIndex].y });
        vertexIndex = (vertexIndex + 1) % n;
    }

    points.push({ x: endAttach.point.x, y: endAttach.point.y });
    return points;
}

const splitPolygonByDrawnEdge = (polygon, startPoint, endPoint) => {
    if (!Array.isArray(polygon) || polygon.length < 3) return null;

    const startAttach = getClosestAttachmentOnPolygon(startPoint, polygon);
    const endAttach = getClosestAttachmentOnPolygon(endPoint, polygon);
    if (!startAttach || !endAttach) return null;

    const sameAttachment = distanceSquared(startAttach.point, endAttach.point) < 1e-8;
    if (sameAttachment) return null;

    const boundaryAtoB = collectBoundaryPath(polygon, startAttach, endAttach);
    const boundaryBtoA = collectBoundaryPath(polygon, endAttach, startAttach);

    const polygon1 = [
        ...boundaryAtoB,
        { x: endPoint.x, y: endPoint.y },
        { x: startPoint.x, y: startPoint.y }
    ];

    const polygon2 = [
        ...boundaryBtoA,
        { x: startPoint.x, y: startPoint.y },
        { x: endPoint.x, y: endPoint.y }
    ];

    const unique1 = new Set(polygon1.map(p => `${p.x.toFixed(4)},${p.y.toFixed(4)}`));
    const unique2 = new Set(polygon2.map(p => `${p.x.toFixed(4)},${p.y.toFixed(4)}`));
    if (unique1.size < 3 || unique2.size < 3) return null;

    return [polygon1, polygon2];
}

const refreshToolsVisibility = () => {
    if (isManualMode || isWaypointMode || isAreaMode) return;
    
    if (buttonManual && buttonIdle) {
        buttonManual.style.display = buttonIdle.checked ? '' : 'none';
    }
    if (buttonWaypoint && buttonIdle) {
        buttonWaypoint.style.display = buttonIdle.checked ? '' : 'none';
    }
    if (buttonArea && buttonIdle) {
        buttonArea.style.display = buttonIdle.checked ? '' : 'none';
    }
    if (buttonPen && buttonIdle) {
        buttonPen.style.display = buttonIdle.checked ? '' : 'none';
    }
    if (buttonEraser && buttonIdle) {
        buttonEraser.style.display = buttonIdle.checked ? '' : 'none';
    }
    if (buttonOutline && buttonIdle) {
        buttonOutline.style.display = buttonIdle.checked ? '' : 'none';
    }

    if (buttonSaveMap && buttonIdle) {
        buttonSaveMap.style.display = buttonIdle.checked ? 'flex' : 'none';
    }
};

const sendCommand = (action, payload = {}) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ action, ...payload }));
    }
};

// WEBSOCKET
function initWebSocket() {
    ws = new WebSocket(`ws://${window.location.host}/ws`);
    ws.binaryType = "arraybuffer";

    ws.onopen = () => {
        updateStatus('connected');
    };

    ws.onclose = () => {
        updateStatus('disconnected');
        setTimeout(initWebSocket, RECONNECT_DELAY);
    };

    ws.onerror = (e) => {
        updateStatus('error');
        console.error('WS error:', e);
    };

    ws.onmessage = async ({ data }) => {
        try {
            if (typeof data === "string") await handleTextFrame(JSON.parse(data));
            else if (data instanceof ArrayBuffer) await handleBinaryFrame(data);
        } catch (err) {
            console.error('Error handling message:', err);
        }
    };
}

async function handleTextFrame({ type, data }) {
    const syncModeUI = (mode) => {
        if (mode === 'explore') {
            buttonExplore.checked = true;
            updateStatus("exploring...");
        } else if (mode === 'clean') {
            buttonClean.checked = true;
            updateStatus("cleaning...");
        } else if (mode === 'idle') {
            buttonIdle.checked = true;
            updateStatus("idle");
        }
        refreshToolsVisibility();
    };


    switch (type) {
        case 'mode_update':
            syncModeUI(data);
            break;

        case 'initial_state':
            if (data.pose) {
                mapCanvas.setPose({
                    x: data.pose.x,
                    y: data.pose.y,
                    theta: getTheta(data.pose)
                });
            }
            if (data.vacuum !== undefined) {
                vacuum = data.vacuum === 1;
                if (toggleVacuum) toggleVacuum.textContent = vacuum ? 'ON' : 'OFF';
            }
            if (data.brush !== undefined) {
                brush = data.brush === 1;
                if (toggleBrush) toggleBrush.textContent = brush ? 'ON' : 'OFF';
            }

            if (data.mode === 'explore' && buttonExplore) {
                buttonExplore.checked = true;
                updateStatus("exploring...");
            } else if (data.mode === 'clean' && buttonClean) {
                buttonClean.checked = true;
                updateStatus(isCleaningQueueRunning ? cleaningQueueStatusText() : "cleaning...");
            } else if (data.mode === 'idle' && buttonIdle) {
                buttonIdle.checked = true;
                updateStatus("idle");
            }

            refreshToolsVisibility();
            syncModeUI(data.mode);
            setTimeout(() => document.body.classList.remove('loading'), 1000);
            break;

        case 'pose_update':
            mapCanvas.setPose({ x: data.x, y: data.y, theta: getTheta(data) });
            break;

        case 'path_update':
            mapCanvas.setPath(data);
            break;

        case 'status_update':
            if (data.status === "finished") {
                if (isCleaningQueueRunning && data.node === 'cleaner') {
                    const finishedRoom = activeCleanRoomNumber;
                    const finishedIndex = activeCleanRoomIndex;
                    const finishedPoints = Number.isInteger(finishedIndex) ? roomPolygons[finishedIndex] : null;
                    
                    activeCleanRoomNumber = null;
                    activeCleanRoomIndex = null;

                    if (Number.isInteger(finishedIndex)) {
                        setRoomSelected(finishedIndex, false);
                    }

                    if (typeof finishedRoom === 'number') {
                        sendCommand('coverage_polygon', { points: finishedPoints, room: finishedRoom });
                    }
                    
                    pushNextRoomToCleaner();
                    return;
                }

                if (buttonIdle) buttonIdle.checked = true;
                updateStatus("idle");
            } else if (data.status === "running") {
                if (data.node === "explorer") {
                    updateStatus("exploring...");
                } else {
                    updateStatus(isCleaningQueueRunning ? cleaningQueueStatusText() : "cleaning...");
                }
            } else if (data.status.startsWith("ERROR")) {
                if (isCleaningQueueRunning && data.node === 'cleaner') {
                    isCleaningQueueRunning = false;
                    const roomLabel = activeCleanRoomNumber;
                    activeCleanRoomNumber = null;
                    activeCleanRoomIndex = null;
                    updateStatus(
                        typeof roomLabel === 'number'
                            ? `cleaning failed in Room ${roomLabel}`
                            : 'cleaning failed'
                    );
                    refreshToolsVisibility();
                    return;
                }

                updateStatus(`error (${data.status})`);
            }
            refreshToolsVisibility();
            break;

        case 'cleaning_polygon_update':
            mapCanvas.setCleaningPolygon(data?.points || []);
            if (roomPolygons.length === 0 && Array.isArray(data?.points) && data.points.length >= 3) {
                setRoomPolygons([data.points]);
            }
            break;

        case 'system_metrics': {
            if (STATS.cpu)  STATS.cpu.textContent  = data.cpu_percent.toFixed(1);
            if (STATS.ram)  STATS.ram.textContent  = data.ram_percent.toFixed(1);
            if (STATS.disk) STATS.disk.textContent = data.disk_percent.toFixed(1);
            break;
        }
        
        case 'coverage_update':
            if (data.success) {
                setCoverageStat(data.coverage_percent);
            } else {
                setCoverageStat(null);
            }
            if (data.room !== undefined) {
                updateStatus(`Room ${data.room} cleaned (${selectedRoomsCountText()}) | queue: ${queueLabelsText()}`);
            }
            break;
            
        case 'save_map_status':
            if (buttonSaveMap) buttonSaveMap.disabled = false;
            updateStatus(data.success ? 'map saved' : (data.error || 'failed to save map'));
            refreshToolsVisibility();
            break;
            
        case 'auto_outline_status':
            if (buttonOutline) buttonOutline.disabled = false;
            if (!data.success) {
                updateStatus(data.error || 'failed to detect outline');
            } else {
                const points = data.polygon?.points || [];
                if (Array.isArray(points) && points.length >= 3) {
                    selectedRoomIndices = new Set();
                    mapCanvas.setSelectedRoomIndices([]);
                    cleanRooms = [];
                    setRoomPolygons([points]);
                }
                updateStatus('outline detected');
            }
            break;
    }
}

async function handleBinaryFrame(buffer) {
    const dataView = new DataView(buffer);
    const metaLen = dataView.getUint32(0, true);

    const metaBytes = new Uint8Array(buffer, 4, metaLen);
    const meta = JSON.parse(new TextDecoder("utf-8").decode(metaBytes));

    const compressedData = new Uint8Array(buffer, 4 + metaLen);
    const decompressedStream = new Response(compressedData)
        .body.pipeThrough(new DecompressionStream("deflate"));

    const rawData = new Uint8Array(
        await new Response(decompressedStream).arrayBuffer()
    );

    const mapType = meta.map_type; // 'map' | 'vacuum' | 'keepout'
    const imageData = gridBytesToImageData(
        rawData,
        meta.width, meta.height,
        mapType
    );

    const newBitmap = await createImageBitmap(imageData);

    if (mapType === 'vacuum') {
        if (currentVacuumBitmap) currentVacuumBitmap.close();
        currentVacuumBitmap = newBitmap;
        mapCanvas.setMap('vacuum', newBitmap, meta);
    } else if (mapType === 'keepout') {
        if (currentKeepoutBitmap) currentKeepoutBitmap.close();
        currentKeepoutBitmap = newBitmap;
        mapCanvas.setMap('keepout', newBitmap, meta);
    } else {
        if (currentMapBitmap) currentMapBitmap.close();
        currentMapBitmap = newBitmap;
        mapCanvas.setMap('map', newBitmap, meta);
    }
}

const startExplore = () => {
    sendCommand('start-explore');
    updateStatus('exploring...');
    refreshToolsVisibility();
}

const saveMap = () => {
    if (!buttonSaveMap) return;
    buttonSaveMap.disabled = true;
    updateStatus('saving map...');
    sendCommand('save-map');
}

const autoOutline = () => {
    if (!buttonOutline) return;
    buttonOutline.disabled = true;
    updateStatus('detecting outline...');
    sendCommand('auto-outline-polygon');
}

// UI TOGGLE HELPER
function setAppMode(mode) {
    isManualMode = mode === 'manual';
    isWaypointMode = mode === 'waypoint';
    isAreaMode = mode === 'area';
    isPenMode = mode === 'pen';
    isEraserMode = mode === 'eraser';
    const isActive = isManualMode || isWaypointMode || isAreaMode || isPenMode || isEraserMode;

    modes.style.display      = isActive ? 'none' : '';
    buttonExit.style.display = isActive ? 'flex' : 'none';
    
    manualToggles.style.display = isManualMode   ? 'grid' : 'none';
    joystick.style.display      = isManualMode   ? 'block' : 'none';
    mapCanvas.style.cursor      = (isWaypointMode || isAreaMode || isPenMode || isEraserMode) ? 'crosshair' : '';

    if (buttonManual)   buttonManual.style.display = isActive ? 'none' : '';
    if (buttonWaypoint) buttonWaypoint.style.display = isActive ? 'none' : '';
    if (buttonArea)     buttonArea.style.display = isActive ? 'none' : '';
    if (buttonPen)      buttonPen.style.display = isActive ? 'none' : '';
    if (buttonEraser)   buttonEraser.style.display = isActive ? 'none' : '';
    if (buttonSaveMap)  buttonSaveMap.style.display = isActive ? 'none' : '';
    if (buttonFocus)    buttonFocus.style.display = isActive || mapCanvas.mode === 'locked' ? 'none' : '';

    if (!isActive) refreshToolsVisibility();
}

// MANUAL CONTROL LOGIC
function enterManualMode() {
    sendCommand('abort');
    mapCanvas.setPath([]);
    mapCanvas.setMode('locked');
    setAppMode('manual');
}

function exitManualMode() {
    if (!isManualMode) return;

    sendCommand('manual_control', { linear: 0, angular: 0 });

    vacuum = brush = false;
    toggleVacuum.textContent = 'OFF';
    toggleBrush.textContent = 'OFF';
    sendCommand('cleaning', { vacuum: 0, brush: 0 });

    setAppMode(null);
}

// WAYPOINT LOGIC
function enterWaypointMode() {
    setAppMode('waypoint');
}

function exitWaypointMode() {
    if (!isWaypointMode) return;
    setAppMode(null);
}

function enterAreaMode() {
    areaDraftPoints = [];
    splitTargetPolygonIndex = null;
    mapCanvas.setCleaningPolygonDraft(areaDraftPoints);
    updateStatus('split mode: pick start and end point');
    setAppMode('area');
}

function exitAreaMode(clearDraft = true) {
    if (!isAreaMode) return;
    if (clearDraft) {
        areaDraftPoints = [];
        splitTargetPolygonIndex = null;
        mapCanvas.setCleaningPolygonDraft([]);
    }
    setAppMode(null);
}

function enterKeepoutMode(mode) {
    if (isManualMode) exitManualMode();
    if (isWaypointMode) exitWaypointMode();
    if (isAreaMode) exitAreaMode();
    setAppMode(mode);
    mapCanvas.interactionMode = mode;
    updateStatus('draw lines to define keepout zones');
}

function exitKeepoutMode() {
    if (!isPenMode && !isEraserMode) return;
    mapCanvas.interactionMode = 'pan';
    setAppMode(null);
}

function submitAreaPolygon() {
    if (areaDraftPoints.length < 3) {
        updateStatus('need at least 3 points for area');
        return;
    }

    sendCommand('cleaning_polygon', { points: areaDraftPoints });
    updateStatus('cleaning area saved');
    mapCanvas.setCleaningPolygonDraft([]);
    areaDraftPoints = [];
    setAppMode(null);
    refreshToolsVisibility();
}

mapCanvas.addEventListener('mapclick', (e) => {
    if (isWaypointMode) {
        sendCommand('move', { x: e.detail.x, y: e.detail.y });
        exitWaypointMode();
        return;
    }

    if (isAreaMode) {
        if (!roomPolygons.length) {
            updateStatus('no polygon available, run outline first');
            return;
        }

        if (areaDraftPoints.length === 0) {
            const target = getClosestPolygonForPoint(e.detail, roomPolygons);
            if (!target) {
                updateStatus('split failed: no valid polygon found');
                return;
            }

            splitTargetPolygonIndex = target.index;
            areaDraftPoints = [{ x: e.detail.x, y: e.detail.y }];
            mapCanvas.setCleaningPolygonDraft(areaDraftPoints);
            updateStatus('split mode: choose end point');
            return;
        }

        const startPoint = areaDraftPoints[0];
        const endPoint = { x: e.detail.x, y: e.detail.y };
        areaDraftPoints = [startPoint, endPoint];
        mapCanvas.setCleaningPolygonDraft(areaDraftPoints);

        const targetIndex = splitTargetPolygonIndex;
        const targetPolygon = Number.isInteger(targetIndex) ? roomPolygons[targetIndex] : null;
        if (!targetPolygon) {
            splitTargetPolygonIndex = null;
            areaDraftPoints = [];
            mapCanvas.setCleaningPolygonDraft([]);
            updateStatus('split failed: select start point again');
            return;
        }

        const splitResult = splitPolygonByDrawnEdge(targetPolygon, startPoint, endPoint);
        if (!splitResult) {
            updateStatus('split failed: choose a better edge');
            areaDraftPoints = [startPoint];
            mapCanvas.setCleaningPolygonDraft(areaDraftPoints);
            return;
        }

        const nextPolygons = roomPolygons.slice();
        nextPolygons.splice(targetIndex, 1, ...splitResult);

        const updatedSelection = new Set();
        for (const index of selectedRoomIndices) {
            if (index < targetIndex) {
                updatedSelection.add(index);
            } else if (index > targetIndex) {
                updatedSelection.add(index + 1);
            } else {
                updatedSelection.add(targetIndex);
                updatedSelection.add(targetIndex + 1);
            }
        }

        selectedRoomIndices = updatedSelection;
        setRoomPolygons(nextPolygons);

        mapCanvas.setCleaningPolygonDraft([]);
        areaDraftPoints = [];
        splitTargetPolygonIndex = null;
        updateStatus('polygon split, pick next start point');
        return;
    }

    if (!isManualMode && !isWaypointMode && !isAreaMode && !isPenMode && !isEraserMode) {
        if (roomPolygons.length > 0) {
            const clickedRoomIndex = getClickedRoomIndex(e.detail);
            if (clickedRoomIndex >= 0) {
                toggleRoomSelection(clickedRoomIndex);
                return;
            }
        }
    }
});

if (buttonManual) buttonManual.onclick = enterManualMode;
if (buttonWaypoint) buttonWaypoint.onclick = enterWaypointMode;
if (buttonArea) buttonArea.onclick = enterAreaMode;
if (buttonPen) buttonPen.onclick = () => enterKeepoutMode('pen');
if (buttonEraser) buttonEraser.onclick = () => enterKeepoutMode('eraser');
if (buttonOutline) buttonOutline.onclick = autoOutline;

mapCanvas.addEventListener('keepoutStroke', (e) => {
    sendCommand('keepout_stroke', e.detail);
});

if (buttonExit) {
    buttonExit.onclick = () => {
        if (isManualMode) exitManualMode();
        if (isWaypointMode) exitWaypointMode();
        if (isAreaMode) exitAreaMode();
        if (isPenMode || isEraserMode) exitKeepoutMode();
    };
}

document.addEventListener('keydown', (e) => {
    if (isAreaMode && e.key === 'Backspace') {
        e.preventDefault();
        areaDraftPoints = areaDraftPoints.slice(0, -1);
        if (areaDraftPoints.length === 0) splitTargetPolygonIndex = null;
        mapCanvas.setCleaningPolygonDraft(areaDraftPoints);
        return;
    }

    if (e.key === 'Escape') {
        if (isManualMode) exitManualMode();
        if (isWaypointMode) exitWaypointMode();
        if (isAreaMode) exitAreaMode();
        if (isPenMode || isEraserMode) exitKeepoutMode();
    }
});

let lastJoystickNav = 0;
joystick.addEventListener('move', (e) => {
    const now = Date.now();

    if (now - lastJoystickNav < 100 && (e.detail.x !== 0 || e.detail.y !== 0)) return;
    lastJoystickNav = now;

    sendCommand('manual_control', { linear: e.detail.y, angular: -e.detail.x });
});

const onToggleVacuum = () => {
    vacuum = !vacuum;
    toggleVacuum.textContent = vacuum ? 'ON' : 'OFF';
    postToggles();
};

const onToggleBrush = () => {
    brush = !brush;
    toggleBrush.textContent = brush ? 'ON' : 'OFF';
    postToggles();
};

const postToggles = () => {
    sendCommand('cleaning', {
        vacuum: vacuum ? 1 : 0,
        brush: brush   ? 1 : 0
    });
};

toggleVacuum.onclick = onToggleVacuum;
toggleBrush.onclick = onToggleBrush;

buttonExplore.onclick = () => {
    sendCommand('start-explore');
    refreshToolsVisibility();
};

 buttonClean.onclick = () => {
    if (selectedRoomIndices.size > 0) {
        startCleanQueue();
    } else {
        sendCommand('start-vacuum');
    }
    refreshToolsVisibility();
};

buttonIdle.onclick = () => {
    sendCommand('abort');
    mapCanvas.setPath([]);
    refreshToolsVisibility();
};

buttonSaveMap.onclick = saveMap;
buttonFocus.onclick = () => mapCanvas.setMode('locked');

mapCanvas.addEventListener('modechange', (e) => {
    if (e.detail === 'free' && isManualMode) {
        exitManualMode();
    }
    
    if (!isManualMode) {
        if (buttonFocus) {
            buttonFocus.style.display = e.detail === 'locked' ? 'none' : '';
        }
    }
});
mapCanvas.setMode('locked');
document.addEventListener('DOMContentLoaded', initWebSocket);
