export class MapCanvas extends HTMLElement {
    #ctx;
    #pointers = new Map();
    #raf;
    #gesture;
    #width;
    #height;
    #robotImg = new Image();

    mode = 'free'; // 'free' | 'locked'
    interactionMode = 'pan'; // 'pan' | 'pen' | 'eraser'
    robotPose = { x: 0, y: 0, theta: 0 };
    path = [];
    cleaningPolygon = [];
    roomPolygons = [];
    selectedRoomIndices = new Set();
    cleaningPolygonDraft = [];
    
    mapBitmap = null;
    vacuumBitmap = null;
    keepoutBitmap = null;
    keepoutInfo = null;
    isKeepoutDrawing = false;
    keepoutDraftPoints = [];
    mapInfo = null;
    vacuumInfo = null;

    KEEPOUT_STROKE_RADIUS = 0.075;
    KEEPOUT_MAX_DRAFT = 500;
    SINGLE_TAP_DELTA = 0.001;
    ROBOT_ICON_RADIUS_M = 0.175;

    x = 0;
    y = 0;
    rotation = 0;
    zoom = 1;

    constructor() {
        super();
        this.attachShadow({ mode: 'open' });
        this.#robotImg.src = '/assets/icons/robot.svg';
        this.#robotImg.onload = () => this.render();

        this.tabIndex = 0;
        this.role = 'application';
        this.ariaLabel = 'Interactive Map';

        this.onwheel = this.#onWheel;
        this.onpointerdown = this.#onDown;
        this.onpointermove = this.#onMove;
        this.onpointerup = this.onpointercancel = this.#onUp;
        this.onkeydown = this.#onKey;
    }

    connectedCallback() {
        if (this.#ctx) return; 

        const link = document.createElement('link');
        link.rel = 'stylesheet';
        link.href = '/assets/styles/MapCanvas.css';

        const canvas = document.createElement('canvas');
        this.#ctx = canvas.getContext('2d');
        this.shadowRoot.append(link, canvas);

        new ResizeObserver(entries => {
            const rect = entries[0].contentRect;
            this.#width = rect.width;
            this.#height = rect.height;

            // Account for high-DPI (Retina) displays
            canvas.width = this.#width * devicePixelRatio;
            canvas.height = this.#height * devicePixelRatio;

            this.render();
        }).observe(this);
    }

    setMap(type, bitmap, info) {
        if (type === 'map') {
            this.mapBitmap = bitmap;
            if (info) this.mapInfo = info;
        } else if (type === 'vacuum') {
            this.vacuumBitmap = bitmap;
            if (info) this.vacuumInfo = info;
        } else if (type === 'keepout') {
            this.keepoutBitmap = bitmap;
            if (info) this.keepoutInfo = info;
        }
        this.render();
    }

    setPose(pose) {
        this.robotPose = pose;
        this.render();
    }

    setPath(path) {
        this.path = path;
        this.render();
    }

    setCleaningPolygon(points) {
        this.cleaningPolygon = Array.isArray(points) ? points : [];
        this.render();
    }

    setRoomPolygons(polygons) {
        this.roomPolygons = Array.isArray(polygons) ? polygons : [];
        this.render();
    }

    setSelectedRoomIndices(indices) {
        this.selectedRoomIndices = new Set(Array.isArray(indices) ? indices : []);
        this.render();
    }

    setCleaningPolygonDraft(points) {
        this.cleaningPolygonDraft = Array.isArray(points) ? points : [];
        this.render();
    }

    setMode(mode) {
        this.mode = mode;
        this.dispatchEvent(new CustomEvent('modechange', { detail: mode }));
        this.render();
    }

    // HELPERS
    #clamp = z => Math.max(0.1, Math.min(50, z));

    // Converts Screen Pixels (x, y) to Map Image Pixel Coordinates
    #toMapPixel = (x, y) => {
        const dx = x - this.x, dy = y - this.y;
        const c = Math.cos(-this.rotation), s = Math.sin(-this.rotation);

        return {
            x: (dx * c - dy * s) / this.zoom,
            y: (dx * s + dy * c) / this.zoom
        };
    }

    // Converts World Coordinates (Meters) to Map Image Pixel Coordinates
    #worldToMapPixel = (wx, wy) => {
        if (!this.mapInfo) return { x: 0, y: 0 };
        const { resolution, origin, height } = this.mapInfo;

        return {
            x: (wx - origin.position.x) / resolution,
            y: height - (wy - origin.position.y) / resolution
        };
    }

    // Converts Map Image Pixel Coordinates to World Coordinates (Meters)
    #mapPixelToWorld = (px, py) => {
        if (!this.mapInfo) return { x: 0, y: 0 };
        const { resolution, origin, height } = this.mapInfo;

        return {
            x: px * resolution + origin.position.x,
            y: (height - py) * resolution + origin.position.y
        };
    }

    // INTERACTIONS
    #onDown = e => {
        this.setPointerCapture(e.pointerId);
        this.#pointers.set(e.pointerId, { 
            x: e.offsetX, 
            y: e.offsetY, 
            startX: e.offsetX, 
            startY: e.offsetY 
        });

        if (this.#pointers.size === 1 && (this.interactionMode === 'pen' || this.interactionMode === 'eraser')) {
            const mapPixel = this.#toMapPixel(e.offsetX, e.offsetY);
            const w = this.#mapPixelToWorld(mapPixel.x, mapPixel.y);
            this.isKeepoutDrawing = true;
            this.keepoutDraftPoints = [{x: w.x, y: w.y}];
            this.render();
            return;
        }

        // Initialize pinch-to-zoom/rotate gesture calculations
        if (this.#pointers.size === 2) {
            const [a, b] = this.#pointers.values();
            const dx = b.x - a.x, dy = b.y - a.y;

            // Anchor zoom to the midpoint
            this.#gesture = {
                dist: Math.hypot(dx, dy),
                angle: Math.atan2(dy, dx),
                startZ: this.zoom,
                startR: this.rotation,
                world: this.#toMapPixel((a.x + b.x) / 2, (a.y + b.y) / 2) 
            };
        }
    }

    #onMove = e => {
        const p = this.#pointers.get(e.pointerId);
        if (!p) return;

        const ox = e.offsetX, oy = e.offsetY;

        if (this.#gesture && this.#pointers.size === 2) {
            const [a, b] = this.#pointers.values();
            p.x = ox; p.y = oy;

            const dx = b.x - a.x, dy = b.y - a.y;
            const midX = (a.x + b.x) / 2, midY = (a.y + b.y) / 2;

            const zoomFactor = Math.hypot(dx, dy) / this.#gesture.dist;
            const gestureRotation = Math.atan2(dy, dx) - this.#gesture.angle;

            this.zoom = this.#clamp(this.#gesture.startZ * zoomFactor);
            this.rotation = this.#gesture.startR + gestureRotation;

            // Shift camera position to keep the midpoint anchor stationary
            const c = Math.cos(this.rotation), s = Math.sin(this.rotation);
            const wx = this.#gesture.world.x * this.zoom;
            const wy = this.#gesture.world.y * this.zoom;

            this.x = midX - (wx * c - wy * s);
            this.y = midY - (wx * s + wy * c);
            
        } else if (this.#pointers.size === 1) {
            if (this.isKeepoutDrawing && (this.interactionMode === 'pen' || this.interactionMode === 'eraser')) {
                const mapPixel = this.#toMapPixel(e.offsetX, e.offsetY);
                const w = this.#mapPixelToWorld(mapPixel.x, mapPixel.y);
                this.keepoutDraftPoints.push({x: w.x, y: w.y});
                 if (this.keepoutDraftPoints.length > this.KEEPOUT_MAX_DRAFT) {
                     this.keepoutDraftPoints.shift();
                 }
                this.render();
                p.x = ox; p.y = oy;
                return;
            }

            // Only break out of 'locked' mode if the user dragged/panned
            if (this.mode === 'locked' && (ox !== p.x || oy !== p.y)) {
                this.setMode('free');
            }

            if (this.mode === 'free') {
                this.x += ox - p.x;
                this.y += oy - p.y;
                p.x = ox; p.y = oy;
                this.render();
            }
        }
    }

    #onUp = e => {
        const p = this.#pointers.get(e.pointerId);

        if (this.isKeepoutDrawing) {
            this.isKeepoutDrawing = false;
            if (this.keepoutDraftPoints.length > 0) {
                this.dispatchEvent(new CustomEvent('keepoutStroke', {
                    detail: { points: [...this.keepoutDraftPoints], mode: this.interactionMode, radius: this.KEEPOUT_STROKE_RADIUS }
                }));
            }
            this.keepoutDraftPoints = [];
            this.render();
        }

        if (p) {
            const dx = e.offsetX - p.startX;
            const dy = e.offsetY - p.startY;
            if (this.#pointers.size === 1 && Math.hypot(dx, dy) < 5) {
                const mapPixel = this.#toMapPixel(e.offsetX, e.offsetY);
                const worldObj = this.#mapPixelToWorld(mapPixel.x, mapPixel.y);
                this.dispatchEvent(new CustomEvent('mapclick', { detail: worldObj }));
            }
        }

        this.#pointers.delete(e.pointerId);
        if (this.#pointers.size < 2) this.#gesture = null;
        this.releasePointerCapture(e.pointerId);
    }

    #onWheel = e => {
        e.preventDefault();
        const factor = Math.exp(-e.deltaY * 0.0015);
        const newZoom = this.#clamp(this.zoom * factor);

        // Calculate world coordinates at mouse cursor to zoom *into* that specific spot
        const w = this.#toMapPixel(e.offsetX, e.offsetY);
        const c = Math.cos(this.rotation), s = Math.sin(this.rotation);
        const wx = w.x * newZoom, wy = w.y * newZoom;

        if (this.mode === 'free') {
            this.x = e.offsetX - (wx * c - wy * s);
            this.y = e.offsetY - (wx * s + wy * c);
        }

        this.zoom = newZoom;
        this.render();
    }

    #onKey = e => {
        const pan = 40, rot = Math.PI / 12;
        let update = true;

        if (e.key.startsWith('Arrow')) {
            if (this.mode === 'locked') this.setMode('free');
            const dx = (e.key === 'ArrowRight') - (e.key === 'ArrowLeft');
            const dy = (e.key === 'ArrowDown') - (e.key === 'ArrowUp');
            this.x -= dx * pan; this.y -= dy * pan;
            
        } else if ('-=+'.includes(e.key)) {
            // scale zoom relative to the center of the canvas
            const f = e.key === '-' ? 1 / 1.15 : 1.15;
            const newZoom = this.#clamp(this.zoom * f);
            
            const cx = this.#width / 2, cy = this.#height / 2;
            const w = this.#toMapPixel(cx, cy);
            const c = Math.cos(this.rotation), s = Math.sin(this.rotation);
            
            if (this.mode === 'free') {
                this.x = cx - (w.x * newZoom * c - w.y * newZoom * s);
                this.y = cy - (w.x * newZoom * s + w.y * newZoom * c);
            }
            this.zoom = newZoom;
            
        } else if (e.key.toLowerCase() === 'q') {
            if (this.mode === 'locked') this.setMode('free');
            this.rotation -= rot;
        } else if (e.key.toLowerCase() === 'e') {
            if (this.mode === 'locked') this.setMode('free');
            this.rotation += rot;
        } else if (e.key === '0') {
            if (this.mode === 'locked') this.setMode('free');
            this.x = this.y = this.rotation = 0; this.zoom = 1;
        } else {
            update = false;
        }

        if (update) { e.preventDefault(); this.render(); }
    }

    // RENDERING
    render() {
        if (!this.#raf) this.#raf = requestAnimationFrame(this.#draw);
    }

    #draw = () => {
        this.#raf = null;
        const ctx = this.#ctx;
        const w = this.#width, h = this.#height;
        const dpr = devicePixelRatio;

        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.fillStyle = 'oklch(.97 0 0)';
        ctx.fillRect(0, 0, w, h);
        ctx.imageSmoothingEnabled = false;

        let tx = this.x, ty = this.y, rot = this.rotation, zoom = this.zoom;

        // If locked, override camera coordinates to center the robot and keep it pointing "up"
        if (this.mode === 'locked' && this.robotPose && this.mapInfo) {
            const { x: rx, y: ry, theta } = this.robotPose;
            rot = theta - (Math.PI / 2);

            const p = this.#worldToMapPixel(rx, ry);
            const c = Math.cos(rot), s = Math.sin(rot);
            const wx = p.x * zoom, wy = p.y * zoom;

            tx = w / 2 - (wx * c - wy * s);
            ty = h / 2 - (wx * s + wy * c);

            // Sync state so breaking the lock doesn't snap the camera back
            this.x = tx; this.y = ty; this.rotation = rot;
        }

        ctx.translate(tx, ty);
        ctx.rotate(rot);
        ctx.scale(zoom, zoom);

        const res = this.mapInfo?.resolution || 0.05;
        const originX = this.mapInfo?.origin?.position?.x || 0;
        const originY = this.mapInfo?.origin?.position?.y || 0;
        const mapH = this.mapInfo?.height || 1000;

        // Visible Grid Bounds
        const screenPixelsPerMeter = zoom / res;
        const rawLog = Math.log2(25 / screenPixelsPerMeter);
        const strideMeters = Math.pow(2, Math.max(0, Math.ceil(rawLog)));

        const cos = Math.cos(-rot), sin = Math.sin(-rot);
        let minPx = Infinity, maxPx = -Infinity, minPy = Infinity, maxPy = -Infinity;

        // Project the 4 screen corners into map coordinates to find the visible bounding box.
        // We dont want to draw grid dots outside the screen.
        for (const [spx, spy] of [[0, 0], [w, 0], [0, h], [w, h]]) {
            const dx = spx - tx, dy = spy - ty;
            const mpx = (dx * cos - dy * sin) / zoom;
            const mpy = (dx * sin + dy * cos) / zoom;

            if (mpx < minPx) minPx = mpx; if (mpx > maxPx) maxPx = mpx;
            if (mpy < minPy) minPy = mpy; if (mpy > maxPy) maxPy = mpy;
        }

        const dotR = Math.max(0.05, 1 / zoom);
        const dotD = dotR * 2;

        ctx.fillStyle = '#999';
        ctx.beginPath();

        const wX1 = originX + minPx * res, wX2 = originX + maxPx * res;
        const wY1 = originY + (mapH - maxPy) * res, wY2 = originY + (mapH - minPy) * res;

        const minWx = Math.min(wX1, wX2), maxWx = Math.max(wX1, wX2);
        const minWy = Math.min(wY1, wY2), maxWy = Math.max(wY1, wY2);

        const sx = Math.floor(minWx / strideMeters) * strideMeters;
        const sy = Math.floor(minWy / strideMeters) * strideMeters;
        const buffer = strideMeters * 2;

        for (let wx = sx - buffer; wx <= maxWx + buffer; wx += strideMeters) {
            const px = (wx - originX) / res;
            for (let wy = sy - buffer; wy <= maxWy + buffer; wy += strideMeters) {
                const py = mapH - (wy - originY) / res;
                ctx.rect(px - dotR, py - dotR, dotD, dotD);
            }
        }
        ctx.fill();

        // Draw map Layers
        if (this.mapBitmap) ctx.drawImage(this.mapBitmap, 0, 0);

        if (this.vacuumBitmap) {
            ctx.save();
            ctx.globalAlpha = 0.7;
            ctx.globalCompositeOperation = 'multiply';
            
            if (this.mapInfo && this.vacuumInfo) {
                const mapRes = this.mapInfo.resolution || 0.05;
                const vacRes = this.vacuumInfo.resolution || 0.05;
                
                const dx = (this.vacuumInfo.origin.position.x - originX) / mapRes;
                const mapTop = originY + mapH * mapRes;
                const vacTop = this.vacuumInfo.origin.position.y + this.vacuumInfo.height * vacRes;
                const dy = (mapTop - vacTop) / mapRes;
                
                const dw = this.vacuumBitmap.width * (vacRes / mapRes);
                const dh = this.vacuumBitmap.height * (vacRes / mapRes);
                
                ctx.drawImage(this.vacuumBitmap, dx, dy, dw, dh);
            } else {
                ctx.drawImage(this.vacuumBitmap, 0, 0);
            }
            
            ctx.restore();
        }

        if (this.keepoutBitmap) {
            ctx.save();
            if (this.mapInfo && this.keepoutInfo) {
                const mapRes = this.mapInfo.resolution || 0.05;
                const kRes = this.keepoutInfo.resolution || 0.05;
                const dx = (this.keepoutInfo.origin.position.x - originX) / mapRes;
                const mapTop = originY + mapH * mapRes;
                const dy = (mapTop - (this.keepoutInfo.origin.position.y + this.keepoutInfo.height * kRes)) / mapRes;
                const dw = this.keepoutBitmap.width * (kRes / mapRes);
                const dh = this.keepoutBitmap.height * (kRes / mapRes);
                ctx.drawImage(this.keepoutBitmap, dx, dy, dw, dh);
            } else {
                ctx.drawImage(this.keepoutBitmap, 0, 0);
            }
            ctx.restore();
        }

        if (this.isKeepoutDrawing && this.keepoutDraftPoints.length > 0) {
            ctx.save();
            ctx.strokeStyle = this.interactionMode === 'pen' ? 'red' : 'white';
            const radius = this.KEEPOUT_STROKE_RADIUS;
            ctx.lineWidth = (radius * 2) / res;
            ctx.lineCap = 'round';
            ctx.lineJoin = 'round';
            ctx.globalAlpha = 0.8;

            ctx.beginPath();
            let first = true;
            for (const pt of this.keepoutDraftPoints) {
                const ppx = this.#worldToMapPixel(pt.x, pt.y);
                if (first) {
                    ctx.moveTo(ppx.x, ppx.y);
                    first = false;
                } else {
                    ctx.lineTo(ppx.x, ppx.y);
                }
            }
            if (this.keepoutDraftPoints.length === 1) {
                const ppx = this.#worldToMapPixel(
                    this.keepoutDraftPoints[0].x,
                    this.keepoutDraftPoints[0].y
                );
                ctx.lineTo(ppx.x, ppx.y + this.SINGLE_TAP_DELTA);
            }
            ctx.stroke();
            ctx.restore();
        }

        // Draw robot path
        if (this.path?.length > 1 && this.mapInfo) {
            ctx.strokeStyle = '#2d81ff';
            ctx.lineWidth = 2 / zoom;
            ctx.lineJoin = 'round';
            ctx.lineCap = 'round';
            ctx.beginPath();

            const invRes = 1 / res;
            const ox = originX, oy = originY;

            for (let i = 0; i < this.path.length; i++) {
                const pt = this.path[i];
                const px = (pt.x - ox) * invRes;
                const py = mapH - (pt.y - oy) * invRes;
                if (i === 0) ctx.moveTo(px, py);
                else ctx.lineTo(px, py);
            }
            ctx.stroke();
        }

        const drawPolygon = (points, closePath, strokeStyle, fillStyle, drawPoints = true) => {
            if (!points || points.length < 2 || !this.mapInfo) return;

            ctx.save();
            ctx.strokeStyle = strokeStyle;
            ctx.lineWidth = 2 / zoom;
            ctx.lineJoin = 'round';
            ctx.lineCap = 'round';

            if (fillStyle && points.length >= 3) {
                ctx.fillStyle = fillStyle;
            }

            ctx.beginPath();
            for (let i = 0; i < points.length; i++) {
                const p = points[i];
                const mp = this.#worldToMapPixel(p.x, p.y);
                if (i === 0) ctx.moveTo(mp.x, mp.y);
                else ctx.lineTo(mp.x, mp.y);
            }

            if (closePath && points.length >= 3) {
                ctx.closePath();
                if (fillStyle) ctx.fill();
            }

            ctx.stroke();

            if (drawPoints) {
                for (const p of points) {
                    const mp = this.#worldToMapPixel(p.x, p.y);
                    ctx.beginPath();
                    ctx.arc(mp.x, mp.y, 3 / zoom, 0, Math.PI * 2);
                    ctx.fillStyle = strokeStyle;
                    ctx.fill();
                }
            }

            ctx.restore();
        };

        if (Array.isArray(this.roomPolygons) && this.roomPolygons.length > 0) {
            const palette = [
                ['rgba(59, 130, 246, 0.98)', 'rgba(59, 130, 246, 0.30)', 'rgba(59, 130, 246, 0.10)'],
                ['rgba(249, 115, 22, 0.98)', 'rgba(249, 115, 22, 0.30)', 'rgba(249, 115, 22, 0.10)'],
                ['rgba(34, 197, 94, 0.98)', 'rgba(34, 197, 94, 0.30)', 'rgba(34, 197, 94, 0.10)'],
                ['rgba(168, 85, 247, 0.98)', 'rgba(168, 85, 247, 0.30)', 'rgba(168, 85, 247, 0.10)']
            ];

            for (let i = 0; i < this.roomPolygons.length; i++) {
                const [stroke, fillSelected, fillDeselected] = palette[i % palette.length];
                const fill = this.selectedRoomIndices.has(i) ? fillSelected : fillDeselected;
                drawPolygon(this.roomPolygons[i], true, stroke, fill, false);
            }
        } else {
            drawPolygon(this.cleaningPolygon, true, 'rgba(255, 128, 0, 0.95)', 'rgba(255, 128, 0, 0.16)');
        }

        drawPolygon(this.cleaningPolygonDraft, false, 'rgba(255, 64, 64, 0.95)', null);

        // Draw Robot & Stats
        if (this.robotPose) {
            const { x, y, theta } = this.robotPose;
            const p = this.#worldToMapPixel(x, y);

            ctx.save();
            ctx.translate(p.x, p.y);
            ctx.rotate(-theta);

            const r = 0.175 / res;

            if (this.#robotImg.complete) {
                ctx.save();
                ctx.rotate(-Math.PI / 2); // Correct SVG orientation
                ctx.drawImage(this.#robotImg, -r, -r, r * 2, r * 2);
                ctx.restore();
            }

            // Orientation indicators
            const overlayRadius = Math.max(50 / zoom, r * 1.5);
            ctx.strokeStyle = 'rgba(128, 128, 128, 0.8)';
            ctx.lineWidth = 2 / zoom;
            
            ctx.beginPath();
            ctx.arc(0, 0, overlayRadius, 0, Math.PI * 2);
            ctx.stroke();

            const lineLen = Math.max(20 / zoom, overlayRadius * 0.4);
            ctx.beginPath();
            ctx.moveTo(overlayRadius - lineLen / 2, 0);
            ctx.lineTo(overlayRadius + lineLen / 2, 0);
            ctx.stroke();

            ctx.rotate(theta - rot);
            ctx.scale(1 / zoom, 1 / zoom);

            ctx.fillStyle = 'rgba(128, 128, 128, 0.8)';
            ctx.font = '14px monospace';
            ctx.textAlign = 'left';
            ctx.textBaseline = 'middle';

            const formatNum = num => num.toFixed(2).padStart(7, ' ');
            const textX = (overlayRadius * zoom) + 10;
            const textY = -20;

            ctx.fillText(`X${formatNum(x)}`, textX, textY);
            ctx.fillText(`Y${formatNum(y)}`, textX, textY + 20);
            ctx.fillText(`θ${formatNum(theta)}`, textX, textY + 40);

            ctx.restore();
        }
    }
}

customElements.define('map-canvas', MapCanvas);
