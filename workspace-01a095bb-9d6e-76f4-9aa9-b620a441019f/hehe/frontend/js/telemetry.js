/**
 * Frontend telemetry WebSocket client (Phase 3).
 * Connects to /ws/telemetry, parses message envelopes, emits events.
 *
 * Importers/callers:
 * - index.html: imports TelemetryClient, uses for real-time position/stats display
 *
 * Affected API:
 * - Connects to GET /ws/telemetry WebSocket endpoint
 *
 * Envelope Schemas:
 * - {"channel": "position", "t": timestamp, "data": {...}}
 * - {"channel": "stats",    "t": timestamp, "data": {...}}
 * - {"channel": "event",    "t": timestamp, "data": {"type": "...", "value": "..."}}
 */
export class TelemetryClient {
    constructor(backendUrl) {
        this._backendUrl = backendUrl;
        this._ws = null;
        this._reconnectTimer = null;
        this._listeners = new Map();
        this._latestSnapshot = null;
    }

    connect() {
        if (this._ws && this._ws.readyState === WebSocket.OPEN) {
            return;
        }

        const wsUrl = this._backendUrl.replace(/^http/, 'ws') + '/ws/telemetry';
        this._ws = new WebSocket(wsUrl);

        this._ws.onopen = () => {
            console.log('[telemetry] connected');
            this._emit('connected', null);
        };

        this._ws.onmessage = (event) => {
            try {
                const msg = JSON.parse(event.data);
                if (msg.channel) {
                    this._emit('envelope', msg);
                    this._emit(msg.channel, msg.data);
                    if (msg.channel === 'event') {
                        this._emit('drone_event', msg.data);
                    } else {
                        this._latestSnapshot = msg.data;
                        this._emit('snapshot', msg.data);
                        this._emitSpecific(msg.data);
                    }
                } else {
                    // Fallback for legacy flat snapshot format
                    this._latestSnapshot = msg;
                    this._emit('snapshot', msg);
                    this._emitSpecific(msg);
                }
            } catch (e) {
                console.warn('[telemetry] parse error:', e);
            }
        };

        this._ws.onclose = () => {
            console.log('[telemetry] disconnected, reconnecting in 2s...');
            this._emit('disconnected', null);
            this._scheduleReconnect();
        };

        this._ws.onerror = (err) => {
            console.error('[telemetry] error:', err);
        };
    }

    _scheduleReconnect() {
        if (this._reconnectTimer) return;
        this._reconnectTimer = setTimeout(() => {
            this._reconnectTimer = null;
            this.connect();
        }, 2000);
    }

    disconnect() {
        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }
        if (this._ws) {
            this._ws.close();
            this._ws = null;
        }
    }

    on(event, callback) {
        if (!this._listeners.has(event)) {
            this._listeners.set(event, new Set());
        }
        this._listeners.get(event).add(callback);
    }

    off(event, callback) {
        if (this._listeners.has(event)) {
            this._listeners.get(event).delete(callback);
        }
    }

    _emit(event, data) {
        if (this._listeners.has(event)) {
            this._listeners.get(event).forEach(cb => cb(data));
        }
    }

    _emitSpecific(snapshot) {
        if (snapshot.position) {
            this._emit('position', snapshot.position);
        }
        if (snapshot.attitude) {
            this._emit('attitude', snapshot.attitude);
        }
        if (snapshot.velocity) {
            this._emit('velocity', snapshot.velocity);
        }
        if (snapshot.gps) {
            this._emit('gps', snapshot.gps);
        }
        if (snapshot.battery) {
            this._emit('battery', snapshot.battery);
        }
        if (snapshot.mode !== undefined) {
            this._emit('mode', snapshot.mode);
        }
        if (snapshot.armed !== undefined) {
            this._emit('armed', snapshot.armed);
        }
    }

    getLatest() {
        return this._latestSnapshot;
    }

    isConnected() {
        return this._ws && this._ws.readyState === WebSocket.OPEN;
    }
}

if (typeof window !== 'undefined') {
    window.TelemetryClient = TelemetryClient;
}