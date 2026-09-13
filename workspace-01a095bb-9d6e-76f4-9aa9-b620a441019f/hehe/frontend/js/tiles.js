/**
 * Map Tile & ENU Meter Grid Controller (Phase 4).
 *
 * Features:
 * 1. Manual Online (OSM) / Offline (local /tiles/{z}/{x}/{y}.png) source toggle.
 * 2. Meter grid overlay based on local ENU (East-North-Up) math.
 *    ENU formulas are direct ports of backend/geo_utils.py.
 */

export const EARTH_RADIUS_M = 6371000.0;

/**
 * Convert lat/lon to local meters relative to origin (port of backend/geo_utils.py latlon_to_enu).
 */
export function latlonToEnu(lat, lon, originLat, originLon) {
    const dlatRad = (lat - originLat) * Math.PI / 180.0;
    const dlonRad = (lon - originLon) * Math.PI / 180.0;
    const originLatRad = originLat * Math.PI / 180.0;

    const north = dlatRad * EARTH_RADIUS_M;
    const east = dlonRad * EARTH_RADIUS_M * Math.cos(originLatRad);
    return { x: east, y: north };
}

/**
 * Convert local ENU meters to lat/lon (port of backend/geo_utils.py enu_to_latlon).
 */
export function enuToLatlon(x, y, originLat, originLon) {
    const originLatRad = originLat * Math.PI / 180.0;
    const dlat = (y / EARTH_RADIUS_M) * 180.0 / Math.PI;
    const dlon = (x / (EARTH_RADIUS_M * Math.cos(originLatRad))) * 180.0 / Math.PI;
    return { lat: originLat + dlat, lon: originLon + dlon };
}

export class TileManager {
    constructor(map, getBackendUrl) {
        this._map = map;
        this._getBackendUrl = getBackendUrl;
        this._isOffline = false;
        this._tileLayer = null;
        this._gridGroup = L.layerGroup().addTo(map);
        this._gridEnabled = true;
        this._gridSpacingMeters = 50;
        this._gridRangeMeters = 500;
        this._origin = null; // { lat, lon, isFallback }

        this.setOnlineMode();
    }

    setOnlineMode() {
        this._isOffline = false;
        this._updateTileLayer();
    }

    setOfflineMode() {
        this._isOffline = true;
        this._updateTileLayer();
    }

    toggleMode() {
        this._isOffline = !this._isOffline;
        this._updateTileLayer();
        return this._isOffline;
    }

    isOffline() {
        return this._isOffline;
    }

    _updateTileLayer() {
        if (this._tileLayer) {
            this._map.removeLayer(this._tileLayer);
        }

        if (this._isOffline) {
            const backend = (this._getBackendUrl() || '').replace(/\/$/, '');
            const tileUrl = backend + '/tiles/{z}/{x}/{y}.png';
            this._tileLayer = L.tileLayer(tileUrl, {
                maxZoom: 20,
                minZoom: 15,
                attribution: 'Offline MBTiles Cache'
            }).addTo(this._map);
        } else {
            this._tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                maxZoom: 20,
                attribution: '&copy; OpenStreetMap contributors'
            }).addTo(this._map);
        }
    }

    setGridOrigin(lat, lon, isFallback = false) {
        this._origin = { lat, lon, isFallback };
        this.renderGrid();
    }

    setGridEnabled(enabled) {
        this._gridEnabled = enabled;
        if (!enabled) {
            this._gridGroup.clearLayers();
        } else {
            this.renderGrid();
        }
    }

    renderGrid() {
        this._gridGroup.clearLayers();
        if (!this._gridEnabled) return;

        let originLat, originLon, isFallback;
        if (this._origin && this._origin.lat != null && this._origin.lon != null) {
            originLat = this._origin.lat;
            originLon = this._origin.lon;
            isFallback = this._origin.isFallback;
        } else {
            const center = this._map.getCenter();
            originLat = center.lat;
            originLon = center.lng;
            isFallback = true;
        }

        const step = this._gridSpacingMeters;
        const range = this._gridRangeMeters;

        // Draw North-South grid lines (constant X / East)
        for (let x = -range; x <= range; x += step) {
            const p1 = enuToLatlon(x, -range, originLat, originLon);
            const p2 = enuToLatlon(x, range, originLat, originLon);
            const isCenter = x === 0;

            const line = L.polyline([[p1.lat, p1.lon], [p2.lat, p2.lon]], {
                color: isCenter ? '#ffb020' : '#52c7e0',
                weight: isCenter ? 1.5 : 0.8,
                opacity: isCenter ? 0.7 : 0.35,
                dashArray: isCenter ? null : '4, 4'
            });
            this._gridGroup.addLayer(line);
        }

        // Draw East-West grid lines (constant Y / North)
        for (let y = -range; y <= range; y += step) {
            const p1 = enuToLatlon(-range, y, originLat, originLon);
            const p2 = enuToLatlon(range, y, originLat, originLon);
            const isCenter = y === 0;

            const line = L.polyline([[p1.lat, p1.lon], [p2.lat, p2.lon]], {
                color: isCenter ? '#ffb020' : '#52c7e0',
                weight: isCenter ? 1.5 : 0.8,
                opacity: isCenter ? 0.7 : 0.35,
                dashArray: isCenter ? null : '4, 4'
            });
            this._gridGroup.addLayer(line);
        }

        // Add origin marker & text label
        const originMarker = L.circleMarker([originLat, originLon], {
            radius: 6,
            color: isFallback ? '#ff5c5c' : '#ffb020',
            fillColor: isFallback ? '#ff5c5c' : '#ffb020',
            fillOpacity: 0.9
        });
        const labelText = isFallback ? 'Grid Origin (Map Center Fallback)' : 'EKF Geofence Origin (0,0)';
        originMarker.bindTooltip(labelText, { permanent: false, direction: 'top' });
        this._gridGroup.addLayer(originMarker);
    }
}

if (typeof window !== 'undefined') {
    window.TileManager = TileManager;
}
