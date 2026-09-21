#!/usr/bin/env python3
"""
La Raspberry ejecuta el vuelo y el barrido en yaw. La estación terrena:
- lee RSSI raw y RSSI filtrado con EMA (alpha=0.1),
- descarta lecturas RSSI fuera del rango válido [-100, -20] dBm,
- grafica RSSI vs tiempo,
- construye un patrón polar en vivo,
- estima AoA al FINAL del barrido con número configurable de máximos candidatos y vecinos por lado,
- guarda CSV de alta resolución sincronizando RSSI filtrado con yaw, altura, posición y lat/lon.

Al finalizar el barrido, esta fase calcula una sola vez el AoA robusto, RSSI del máximo elegido y ancho de haz, y envía el AoA de forma confiable a la Raspberry.
La Raspberry mantiene posición/altura en OFFBOARD, orienta el dron al AoA absoluto respecto al norte y después aterriza.

"""

import csv
import datetime as _dt
import json
import math
import os
import queue
import random
import threading
import time
import tkinter as tk
import zlib
from collections import deque, defaultdict
from tkinter import ttk, messagebox
from tkinter.scrolledtext import ScrolledText

import serial

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.lines import Line2D


# =========================
# Puertos por defecto
# =========================

SIK_PORT = "COM5"          # Telemetría SiK hacia Raspberry
SIK_BAUD = 57600

RSSI_RADIO_PORT = "COM8"   # MRAPC/ZigBee para consultar RSSI
RSSI_RADIO_BAUD = 57600


# =========================
# Parámetros de enlace SiK
# =========================

HEARTBEAT_PERIOD_S = 0.50
STATUS_LOG_PERIOD_S = 5.0
LINK_TIMEOUT_S = 5.0
# Si la última telemetría PX4/Raspberry es vieja, las muestras RSSI no se agregan
# al patrón para evitar asociarlas a un yaw/ángulo ya obsoleto durante pérdidas SiK.
MAX_STATUS_AGE_FOR_PATTERN_S = 0.75

DEFAULT_ACK_TIMEOUT_S = 0.5
DEFAULT_MAX_RETRIES = 30
LAND_ACK_TIMEOUT_S = 0.3
LAND_MAX_RETRIES = 50
AOA_ACK_TIMEOUT_S = 0.4
AOA_MAX_RETRIES = 30
AOA_AUTO_SEND_DELAY_MS = 700


# =========================
# Parámetros RSSI MRAPC
# =========================

# MACs de PCs (0xFxxx)
PC_MAC = 0xF021
DEST_MAC = 0xF022

# Radios ZigBee (0x30xx)
LOCAL_RADIO_MAC = 0x3021
REMOTE_RADIO_MAC = 0x3022

RSSI_IM = 0x73
REGISTER_EVERY_S = 2.0
RSSI_QUERY_EVERY_S = 0.10
RSSI_ALPHA = 0.1

# Gráficas
RSSI_PLOT_WINDOW_S = 30.0
RSSI_PLOT_MAX_POINTS = int(RSSI_PLOT_WINDOW_S / RSSI_QUERY_EVERY_S) + 50
BASE_DBM_MIN = -100.0
BASE_DBM_MAX = -20.0
VALID_RSSI_MIN_DBM = -100.0
VALID_RSSI_MAX_DBM = -20.0
CONTINUOUS_POLAR_BIN_DEG = 5.0

# AoA robusto (valores por defecto y límites de la GUI)
DEFAULT_ROBUST_PEAK_CANDIDATES = 3
DEFAULT_ROBUST_PEAK_NEIGHBORS = 1
MIN_ROBUST_PEAK_CANDIDATES = 1
MAX_ROBUST_PEAK_CANDIDATES = 5
MIN_ROBUST_PEAK_NEIGHBORS = 1
MAX_ROBUST_PEAK_NEIGHBORS = 3


# Orden visual de máximos candidatos en el patrón polar y en los CSV.
CANDIDATE_COLORS = ["red", "green", "orange", "hotpink", "gold"]


# =========================
# Protocolo SiK compacto
# =========================

STATE_NAMES = {
    "I": "ESPERA",
    "TK": "DESPEGANDO",
    "HT": "ALTURA_ALCANZADA",
    "OF": "ENTRANDO_OFFBOARD",
    "N0": "ORIENTANDO_NORTE",
    "BC": "BARRIDO_CONTINUO",
    "BD": "BARRIDO_DISCRETO",
    "QA": "ESPERANDO_AOA",
    "YA": "ORIENTANDO_AOA",
    "W": "ESPERA",
    "LND": "ATERRIZANDO",
    "LL": "ENLACE_PERDIDO",
    "ER": "ERROR",
}

CMD_NAMES = {
    "N": "ninguno",
    "S": "comenzar_barrido",
    "SC": "barrido_completo",
    "SE": "error_barrido",
    "Y": "orientar_aoa",
    "YC": "aoa_orientado",
    "L": "aterrizar",
    "LC": "aterrizaje_completo",
    "R": "reinicio",
    "RC": "reinicio_completo",
    "M": "armado_2s",
}

MODE_NAMES = {
    "U": "UNKNOWN",
    "P": "POSITION",
    "O": "OFFBOARD",
    "S": "STABILIZED",
    "A": "ALTITUDE",
    "M": "MANUAL",
    "H": "HOLD",
    "R": "RETURN/RTL",
    "L": "LAND",
    "T": "TAKEOFF",
    "N": "MISSION",
    "C": "ACRO",
}

LANDED_NAMES = {
    "U": "UNKNOWN",
    "G": "EN TIERRA",
    "A": "EN AIRE",
    "T": "DESPEGANDO",
    "L": "ATERRIZANDO",
}

ACTIVE_STATES = {"TK", "HT", "OF", "N0", "BC", "BD", "QA", "YA", "W", "LND"}


def decode(mapping: dict, code, default="desconocido"):
    return mapping.get(code, default)


def wrap180(deg: float) -> float:
    return ((float(deg) + 180.0) % 360.0) - 180.0


def wrap360(deg: float) -> float:
    return float(deg) % 360.0


def safe_float(v):
    try:
        if v is None or v == "":
            return None
        x = float(v)
        if math.isnan(x) or math.isinf(x):
            return None
        return x
    except Exception:
        return None


def bin_angle(angle_deg: float, step_deg: float) -> float:
    if step_deg <= 0:
        step_deg = 1.0
    b = round(wrap360(angle_deg) / step_deg) * step_deg
    b = b % 360.0
    if abs(b - 360.0) < 1e-9:
        b = 0.0
    return round(b, 3)



def make_frame(msg: dict) -> bytes:
    payload = json.dumps(msg, separators=(",", ":"), ensure_ascii=False)
    crc = zlib.crc32(payload.encode("utf-8")) & 0xFFFFFFFF
    frame = f"${payload}*{crc:08X}\n"
    return frame.encode("utf-8")


def parse_frame(line: str):
    line = line.strip()
    if not line:
        return None, "vacio"
    start = line.find("$")
    if start < 0:
        return None, "sin_inicio_de_trama"
    if start > 0:
        line = line[start:]
    if not line.startswith("$"):
        return None, "sin_inicio_de_trama"
    body = line[1:]
    if "*" not in body:
        return None, "sin_crc"
    payload, crc_txt = body.rsplit("*", 1)
    if len(crc_txt) != 8:
        return None, "crc_longitud_invalida"
    try:
        crc_rx = int(crc_txt, 16)
    except ValueError:
        return None, "crc_no_hexadecimal"
    crc_calc = zlib.crc32(payload.encode("utf-8")) & 0xFFFFFFFF
    if crc_rx != crc_calc:
        return None, "crc_no_coincide"
    try:
        msg = json.loads(payload)
    except json.JSONDecodeError:
        return None, "json_invalido"
    return msg, None


# =========================
# Utilidades MRAPC / RSSI
# =========================


def crc16_ibm(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def u16_to_bytes(x: int) -> bytes:
    return bytes([(x >> 8) & 0xFF, x & 0xFF])


def build_frame_mrapc(do, dd, I, N=0x00, R=0x00, M=b"") -> bytes:
    payload_wo_crc = bytes([
        (do >> 8) & 0xFF, do & 0xFF,
        (dd >> 8) & 0xFF, dd & 0xFF,
        I, N, R,
    ]) + M
    L = 1 + len(payload_wo_crc) + 2
    internal_wo_crc = bytes([L]) + payload_wo_crc
    c = crc16_ibm(internal_wo_crc, init=0xFFFF)
    crc_bytes = bytes([c & 0xFF, (c >> 8) & 0xFF])
    return bytes([0xFE]) + internal_wo_crc + crc_bytes + bytes([0xEF])


def build_rssi_request_frame(pc_mac: int, mrapc_mac: int, remote_radio_mac: int, seqn: int) -> bytes:
    internal_wo_crc = bytes([
        0x07, seqn & 0xFF, RSSI_IM,
        (remote_radio_mac >> 8) & 0xFF, remote_radio_mac & 0xFF,
    ])
    c = crc16_ibm(internal_wo_crc, init=0xFFFF)
    crc_bytes = bytes([c & 0xFF, (c >> 8) & 0xFF])
    inner_pkt = bytes([0xFE]) + internal_wo_crc + crc_bytes + bytes([0xEF])
    M = bytes([
        (PC_MAC >> 8) & 0xFF, PC_MAC & 0xFF,
        (mrapc_mac >> 8) & 0xFF, mrapc_mac & 0xFF,
    ]) + inner_pkt
    return build_frame_mrapc(do=PC_MAC, dd=mrapc_mac, I=0x04, N=(seqn & 0xFF), R=0x00, M=M)


def parse_frames_from_buffer(buf: bytes):
    frames = []
    while True:
        start = buf.find(b"\xFE")
        if start < 0:
            return frames, b""
        buf = buf[start:]
        if len(buf) < 3:
            return frames, buf
        L = buf[1]
        total_len = 1 + L + 1
        if len(buf) < total_len:
            return frames, buf
        frame = buf[:total_len]
        buf = buf[total_len:]
        if frame[-1] != 0xEF:
            buf = frame[1:] + buf
            continue
        frames.append(frame)


def decode_frame_mrapc(frame: bytes):
    L = frame[1]
    internal = frame[1:1 + L]
    payload = internal[1:]
    do = (payload[0] << 8) | payload[1]
    dd = (payload[2] << 8) | payload[3]
    I = payload[4]
    N = payload[5]
    R = payload[6]
    M = payload[7:-2]
    c_recv = (payload[-1] << 8) | payload[-2]
    c_calc = crc16_ibm(internal[:-2], init=0xFFFF)
    return c_calc == c_recv, do, dd, I, N, R, M


def try_parse_data_packet(m: bytes):
    if len(m) < 2 + 2 + 1 + 1 + 2 + 1:
        return None
    df = (m[0] << 8) | m[1]
    ds = (m[2] << 8) | m[3]
    it = m[4]
    if it != 0xFE:
        return None
    Lm = m[5]
    total_len = 2 + 2 + 1 + Lm + 1
    if len(m) < total_len:
        return None
    if m[total_len - 1] != 0xEF:
        return None
    internal = m[5:5 + Lm]
    body = internal[1:]
    if len(body) < 1 + 1 + 2:
        return None
    nm = body[0]
    im = body[1]
    mm = body[2:-2]
    c_recv = (body[-1] << 8) | body[-2]
    c_calc = crc16_ibm(internal[:-2], init=0xFFFF)
    if c_calc != c_recv:
        return None
    return df, ds, nm, im, mm


class ReliableRSSINode:
    """Nodo PC/MRAPC para descubrir radio local y consultar RSSI del radio remoto."""

    def __init__(self, port, baud, gui_queue: queue.Queue):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.rx = b""
        self.gui_q = gui_queue
        self.mrapc_mac = None
        self.ready = False
        self.running = True
        self.tx_lock = threading.Lock()
        self.rssi_raw = None
        self.rssi_dbm = None
        self.rssi_filt = None
        self._seqn = 0
        self.invalid_frames = 0

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass

    def _write(self, data: bytes):
        with self.tx_lock:
            self.ser.write(data)
            self.ser.flush()

    def send_register(self):
        if self.mrapc_mac is None:
            return
        reg = build_frame_mrapc(do=PC_MAC, dd=self.mrapc_mac, I=0x02, M=bytes([0x02]))
        self._write(reg)

    def send_ping_response(self):
        if self.mrapc_mac is None:
            return
        resp = build_frame_mrapc(do=PC_MAC, dd=self.mrapc_mac, I=0xFF, M=bytes([0x02]))
        self._write(resp)

    def request_rssi(self):
        if self.mrapc_mac is None or not self.ready:
            return
        self._seqn = (self._seqn + 1) & 0xFF
        fr = build_rssi_request_frame(PC_MAC, self.mrapc_mac, REMOTE_RADIO_MAC, self._seqn)
        self._write(fr)

    def _update_rssi_from_mm(self, mm: bytes):
        # En los radios TTR el campo RSSI se transmite como RSSI + 128.
        if len(mm) < 5:
            return
        raw = int(mm[4])
        dbm = raw - 128
        self.rssi_raw = raw
        self.rssi_dbm = dbm
        if self.rssi_filt is None:
            self.rssi_filt = float(dbm)
        else:
            self.rssi_filt = (1.0 - RSSI_ALPHA) * self.rssi_filt + RSSI_ALPHA * float(dbm)
        self.gui_q.put(("rssi", raw, dbm, float(self.rssi_filt)))

    def loop(self):
        last_reg = 0.0
        while self.running:
            try:
                self.rx += self.ser.read(4096)
                frames, self.rx = parse_frames_from_buffer(self.rx)
                for f in frames:
                    crc_ok, do, dd, I, N, R, M = decode_frame_mrapc(f)
                    if not crc_ok:
                        self.invalid_frames += 1
                        continue
                    if I == 0x03 and len(M) >= 3:
                        self.mrapc_mac = (M[1] << 8) | M[2]
                        self.gui_q.put(("mrapc", self.mrapc_mac))
                    elif I == 0xFE:
                        if not self.ready:
                            self.ready = True
                            self.gui_q.put(("radio_ready", True))
                    elif I == 0x01:
                        self.send_ping_response()
                    elif I == 0x04:
                        parsed = try_parse_data_packet(M)
                        if parsed is None:
                            continue
                        df, ds, nm, im, mm = parsed
                        if im == RSSI_IM:
                            self._update_rssi_from_mm(mm)
                    elif I == RSSI_IM:
                        self._update_rssi_from_mm(M)

                now = time.time()
                if (not self.ready) and self.mrapc_mac is not None and (now - last_reg > REGISTER_EVERY_S):
                    self.send_register()
                    last_reg = now
                time.sleep(0.005)
            except Exception as e:
                self.gui_q.put(("rssi_error", str(e)))
                time.sleep(0.2)


# =========================
# AoA y ancho de haz
# =========================


def ranked_peak_candidates_from_results(
    results: dict[float, float],
    expected_angles: list[float],
    peak_candidates: int = DEFAULT_ROBUST_PEAK_CANDIDATES,
    neighbors_per_side: int = DEFAULT_ROBUST_PEAK_NEIGHBORS,
):
    """
    Devuelve una lista ordenada de máximos candidatos.

    El orden final no depende solo del valor puntual del pico, sino de la media
    vecinal utilizada para el criterio robusto. Así, AoA 1 siempre es el candidato
    finalmente elegido, mientras que AoA 2, AoA 3, etc. son los otros máximos
    locales relevantes que quedaron detrás en importancia.
    """
    if not results or not expected_angles:
        return []
    angles = [float(a) for a in expected_angles if a in results]
    if not angles:
        return []
    vals = [float(results[a]) for a in angles]
    n = len(angles)
    if n == 1:
        return [{
            'rank': 1, 'angle': angles[0] % 360.0, 'peak_dbm': vals[0],
            'robust_mean_dbm': vals[0], 'index': 0, 'color': CANDIDATE_COLORS[0],
        }]

    local_max_idx = []
    for i in range(n):
        left = vals[(i - 1) % n]
        center = vals[i]
        right = vals[(i + 1) % n]
        if center >= left and center >= right:
            local_max_idx.append(i)
    if not local_max_idx:
        local_max_idx = list(range(n))

    local_max_idx = sorted(local_max_idx, key=lambda i: vals[i], reverse=True)
    n_candidates = max(MIN_ROBUST_PEAK_CANDIDATES, min(MAX_ROBUST_PEAK_CANDIDATES, int(peak_candidates)))
    local_max_idx = local_max_idx[:min(n_candidates, len(local_max_idx))]

    nb = max(MIN_ROBUST_PEAK_NEIGHBORS, min(MAX_ROBUST_PEAK_NEIGHBORS, int(neighbors_per_side)))
    candidates = []
    for i in local_max_idx:
        neigh = [vals[(i + k) % n] for k in range(-nb, nb + 1)]
        avg = sum(neigh) / len(neigh)
        candidates.append({
            'angle': angles[i] % 360.0,
            'peak_dbm': vals[i],
            'robust_mean_dbm': avg,
            'index': i,
        })

    candidates.sort(key=lambda c: (c['robust_mean_dbm'], c['peak_dbm']), reverse=True)
    for rank, cand in enumerate(candidates, start=1):
        cand['rank'] = rank
        cand['color'] = CANDIDATE_COLORS[(rank - 1) % len(CANDIDATE_COLORS)]
    return candidates


def robust_peak_from_results(
    results: dict[float, float],
    expected_angles: list[float],
    peak_candidates: int = DEFAULT_ROBUST_PEAK_CANDIDATES,
    neighbors_per_side: int = DEFAULT_ROBUST_PEAK_NEIGHBORS,
):
    candidates = ranked_peak_candidates_from_results(
        results, expected_angles,
        peak_candidates=peak_candidates,
        neighbors_per_side=neighbors_per_side,
    )
    if not candidates:
        return None
    best = candidates[0]
    return best['angle'], best['peak_dbm'], best['robust_mean_dbm']


def interp_crossing_angle(a1: float, y1: float, a2: float, y2: float, thr: float) -> float:
    da = wrap180(a2 - a1)
    if abs(y2 - y1) < 1e-12:
        return a1 % 360.0
    t = (thr - y1) / (y2 - y1)
    t = max(0.0, min(1.0, t))
    return (a1 + t * da) % 360.0


def compute_beamwidth_info(
    results: dict[float, float],
    expected_angles: list[float],
    peak_candidates: int = DEFAULT_ROBUST_PEAK_CANDIDATES,
    neighbors_per_side: int = DEFAULT_ROBUST_PEAK_NEIGHBORS,
):
    candidates = ranked_peak_candidates_from_results(
        results, expected_angles,
        peak_candidates=peak_candidates,
        neighbors_per_side=neighbors_per_side,
    )
    if not candidates:
        return None
    best = candidates[0]
    peak_angle = best['angle']
    peak_dbm = best['peak_dbm']
    thr = float(peak_dbm) - 3.0
    angles = [float(a) for a in expected_angles if a in results]
    if len(angles) < 3:
        return None
    vals = [float(results[a]) for a in angles]
    n = len(angles)
    try:
        peak_idx = angles.index(float(peak_angle))
    except ValueError:
        return None

    left_cross = None
    for step in range(1, n + 1):
        j = (peak_idx - step) % n
        prev = (j + 1) % n
        y_near = vals[prev]
        y_far = vals[j]
        if y_near >= thr and y_far < thr:
            left_cross = interp_crossing_angle(angles[prev], y_near, angles[j], y_far, thr)
            break

    right_cross = None
    for step in range(1, n + 1):
        j = (peak_idx + step) % n
        prev = (j - 1) % n
        y_near = vals[prev]
        y_far = vals[j]
        if y_near >= thr and y_far < thr:
            right_cross = interp_crossing_angle(angles[prev], y_near, angles[j], y_far, thr)
            break

    if left_cross is None or right_cross is None:
        return None

    beamwidth = (right_cross - left_cross) % 360.0
    if beamwidth > 180.0:
        beamwidth = 360.0 - beamwidth

    return {
        'peak_angle': float(peak_angle) % 360.0,
        'peak_dbm': float(peak_dbm),
        'robust_mean_dbm': float(best['robust_mean_dbm']),
        'half_power_dbm': thr,
        'left_angle': float(left_cross) % 360.0,
        'right_angle': float(right_cross) % 360.0,
        'beamwidth_deg': float(beamwidth),
        'candidates': candidates,
    }


def format_candidates_multiline(beam: dict | None) -> str:
    if beam is None:
        return 'AoA: ---'
    lines = []
    for cand in beam.get('candidates', []):
        rank = cand.get('rank', '?')
        ang = cand.get('angle')
        peak = cand.get('peak_dbm')
        if rank == 1:
            bw = beam.get('beamwidth_deg')
            lines.append(f"AoA 1: {ang:.1f}° | RSSI {peak:.1f} dBm | BW {bw:.1f}°")
        else:
            lines.append(f"AoA {rank}: {ang:.1f}° | RSSI {peak:.1f} dBm")
    return ' | '.join(lines) if lines else 'AoA: ---'


# =========================
# Ventanas de gráfica
# =========================


class RssiPlotWindow:
    def __init__(self, parent: tk.Tk, t_buf, raw_buf, filt_buf):
        self.parent = parent
        self.t_buf = t_buf
        self.raw_buf = raw_buf
        self.filt_buf = filt_buf
        self.win = tk.Toplevel(parent)
        self.win.title("RSSI vs Tiempo")
        self.fig = Figure(figsize=(8.5, 4.5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("RSSI (dBm) vs Tiempo")
        self.ax.set_xlabel("Tiempo (s)")
        self.ax.set_ylabel("RSSI (dBm)")
        self.ax.grid(True)
        (self.line_raw,) = self.ax.plot([], [], color="blue", label="RSSI raw (dBm)")
        (self.line_filt,) = self.ax.plot([], [], color="red", label="RSSI filtrado (dBm)")
        self.ax.legend(loc="upper left")
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.win)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._running = True
        self.win.protocol("WM_DELETE_WINDOW", self.on_close)
        self.refresh()

    def on_close(self):
        self._running = False
        try:
            self.win.destroy()
        except Exception:
            pass

    def clear_plot(self):
        """Limpia visualmente la gráfica cuando se reinicia la serie RSSI."""
        try:
            self.line_raw.set_data([], [])
            self.line_filt.set_data([], [])
            self.ax.set_xlim(-0.05, 0.05)
            self.ax.set_ylim(BASE_DBM_MIN, BASE_DBM_MAX)
            self.canvas.draw_idle()
        except Exception:
            pass

    def refresh(self):
        if not self._running:
            return
        t = list(self.t_buf)
        y1 = list(self.raw_buf)
        y2 = list(self.filt_buf)

        # Protección contra cambios de configuración mientras la gráfica está abierta.
        # Si se reinicia el filtro RSSI, las colas pueden vaciarse entre dos
        # refrescos. Usamos la longitud común para evitar que matplotlib reciba series
        # de distinto tamaño.
        n = min(len(t), len(y1), len(y2))
        if n >= 2:
            t = t[-n:]
            y1 = y1[-n:]
            y2 = y2[-n:]
            self.line_raw.set_data(t, y1)
            self.line_filt.set_data(t, y2)
            tmax = t[-1]
            tmin = max(0.0, tmax - RSSI_PLOT_WINDOW_S)
            if tmax <= tmin:
                tmin = max(0.0, tmax - 1.0)
            self.ax.set_xlim(tmin, tmax)
            vals = [v for v in y1 + y2 if v is not None]
            ymin, ymax = BASE_DBM_MIN, BASE_DBM_MAX
            if vals:
                ymin = min(BASE_DBM_MIN, math.floor(min(vals) / 5.0) * 5.0)
                ymax = max(BASE_DBM_MAX, math.ceil(max(vals) / 5.0) * 5.0)
                if ymax - ymin < 10:
                    ymax = ymin + 10
            self.ax.set_ylim(ymin, ymax)
        else:
            self.line_raw.set_data([], [])
            self.line_filt.set_data([], [])
            self.ax.set_xlim(-0.05, 0.05)
            self.ax.set_ylim(BASE_DBM_MIN, BASE_DBM_MAX)
        self.canvas.draw_idle()
        self.win.after(100, self.refresh)


class PatternPlotWindow:
    def __init__(self, parent: tk.Tk, get_pattern_callable, get_expected_callable, get_beam_callable):
        self.parent = parent
        self.get_pattern = get_pattern_callable
        self.get_expected = get_expected_callable
        self.get_beam = get_beam_callable
        self.win = tk.Toplevel(parent)
        self.win.title("Patrón de radiación (polar)")
        self.fig = Figure(figsize=(6.8, 6.2), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="polar")
        self.ax.set_title("Patrón de radiación (RSSI filtrado en dBm)", va="bottom")
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self._configure_polar_axis(BASE_DBM_MIN, BASE_DBM_MAX)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.win)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        self._running = True
        self.win.protocol("WM_DELETE_WINDOW", self.on_close)
        self.refresh()

    def _configure_polar_axis(self, dbm_min, dbm_max):
        span = dbm_max - dbm_min
        if span <= 0:
            span = 1.0
        self.ax.set_rlim(0, span)
        step_db = 10
        start = int(math.floor(dbm_min / step_db) * step_db)
        stop = int(math.ceil(dbm_max / step_db) * step_db)
        ticks_dbm = list(range(start, stop + 1, step_db))
        ticks_r = [dbm - dbm_min for dbm in ticks_dbm]
        self.ax.set_yticks(ticks_r)
        self.ax.set_yticklabels([f"{dbm:d}" for dbm in ticks_dbm])
        self.ax.set_rlabel_position(22.5)

    def on_close(self):
        self._running = False
        try:
            self.win.destroy()
        except Exception:
            pass

    def save_png(self, filepath: str):
        self.fig.savefig(filepath, dpi=200)

    def refresh(self):
        if not self._running:
            return
        results = self.get_pattern()
        expected = self.get_expected()
        beam = self.get_beam()
        self.ax.clear()
        self.ax.set_title("Patrón de radiación (RSSI filtrado en dBm)", va="bottom")
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)

        vals_all = list(results.values()) if results else []
        dbm_min, dbm_max = BASE_DBM_MIN, BASE_DBM_MAX
        if vals_all:
            dbm_min = min(BASE_DBM_MIN, math.floor(min(vals_all) / 5.0) * 5.0)
            dbm_max = max(BASE_DBM_MAX, math.ceil(max(vals_all) / 5.0) * 5.0)
            if dbm_max - dbm_min < 10:
                dbm_max = dbm_min + 10
        self._configure_polar_axis(dbm_min, dbm_max)

        if results:
            angles = expected if expected else sorted(results.keys())
            angles = [a for a in angles if a in results]
            if angles:
                angles_closed = angles + [angles[0] + 360.0]
                dbm_vals = [results[a] for a in angles] + [results[angles[0]]]
                theta = [math.radians(a % 360.0) for a in angles_closed]
                r = [max(dbm_min, min(dbm_max, float(v))) - dbm_min for v in dbm_vals]
                self.ax.plot(theta, r, marker="o", linewidth=1.5, color="blue")

        if beam is not None:
            candidates = beam.get('candidates', []) or []
            for cand in candidates:
                ang = float(cand['angle']) % 360.0
                peak_dbm = float(cand['peak_dbm'])
                th = math.radians(ang)
                rr = max(dbm_min, min(dbm_max, peak_dbm)) - dbm_min
                self.ax.plot([th], [rr], marker='o', markersize=9, linestyle='None', color=cand.get('color', 'red'))

            peak_angle = beam['peak_angle']
            peak_dbm = beam['peak_dbm']
            th_best = math.radians(peak_angle)
            r_best = max(dbm_min, min(dbm_max, peak_dbm)) - dbm_min
            thr = beam['half_power_dbm']
            r_thr = max(dbm_min, min(dbm_max, thr)) - dbm_min
            left_angle = beam['left_angle']
            right_angle = beam['right_angle']
            self.ax.plot([math.radians(left_angle), math.radians(left_angle)], [0.0, r_thr], linewidth=1.6, color='black')
            self.ax.plot([math.radians(right_angle), math.radians(right_angle)], [0.0, r_thr], linewidth=1.6, color='black')

            ang = peak_angle % 360.0
            xytext = (-135, 10) if 90.0 < ang < 270.0 else (10, 10)
            ha = 'right' if 90.0 < ang < 270.0 else 'left'
            lines = []
            for cand in candidates:
                rank = cand['rank']
                prefix = f"AoA {rank}: {cand['angle']:.0f}° | {cand['peak_dbm']:.1f} dBm"
                if rank == 1:
                    prefix += f" | BW {beam['beamwidth_deg']:.1f}°"
                lines.append(prefix)
            text_box = '\n'.join(lines) if lines else f"AoA 1: {ang:.0f}° | {peak_dbm:.1f} dBm | BW {beam['beamwidth_deg']:.1f}°"
            self.ax.annotate(
                text_box,
                xy=(th_best, r_best), xytext=xytext, textcoords='offset points',
                fontsize=10, ha=ha, color='red',
                bbox=dict(boxstyle='round,pad=0.25', fc='white', ec='red', alpha=0.8),
            )

            # Leyenda visual de máximos candidatos. Solo aparece al finalizar el barrido,
            # cuando existe un análisis AoA congelado. El color rojo corresponde al AoA
            # elegido; los demás colores identifican candidatos evaluados que no fueron
            # seleccionados como dirección final.
            legend_handles = []
            for cand in candidates:
                legend_handles.append(Line2D(
                    [0], [0], marker='o', linestyle='None',
                    markerfacecolor=cand.get('color', 'red'),
                    markeredgecolor=cand.get('color', 'red'),
                    markersize=8,
                    label=f"AoA {cand['rank']}: {cand['angle']:.0f}°",
                ))
            if legend_handles:
                self.ax.legend(
                    handles=legend_handles,
                    loc='upper right',
                    bbox_to_anchor=(1.18, 1.12),
                    title='Candidatos',
                    fontsize=8,
                    title_fontsize=8,
                    framealpha=0.85,
                )

        self.canvas.draw_idle()
        self.win.after(200, self.refresh)


# =========================
# GUI principal
# =========================

# =========================


class GroundStationGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Fase 4.9 - Estación terrena PX4 + RSSI + AoA sin compensación")
        self.root.geometry("1280x940")

        self.sik_ser = None
        self.rssi_node = None
        self.running = False
        self.seq = 0
        self.cmd_id = 0
        self.session_id = self.new_session_id()

        self.rx_queue = queue.Queue()
        self.rssi_queue = queue.Queue()
        self.rx_buffer = bytearray()
        self.tx_lock = threading.Lock()

        self.last_drone_msg_time = None
        self.link_ok = False
        self.rx_invalid_count = 0
        self.last_status_log_time = 0.0
        self.last_status_tuple = None
        self.pending_cmd = None
        self.last_ignored_session = None
        self.routine_active = False

        # Última telemetría PX4 recibida desde Raspberry.
        self.last_status_msg = {}
        self.current_mission_cfg = {}

        # RSSI y logs.
        # latest_rssi_dbm conserva el RSSI válido recibido del MRAPC.
        # Esta versión no aplica compensación: el RSSI válido entra directamente al filtro EMA y al patrón.
        self.latest_rssi_raw = None
        self.latest_rssi_dbm = None
        self.latest_rssi_filt = None
        self.latest_rssi_time = None
        self.rssi_filter_state = None
        self.mission_start_wall_time = None
        self.logging_enabled = False
        self.log_records = []
        self.pattern_samples = defaultdict(list)
        self.pattern_lock = threading.Lock()
        self.pattern_stale_drop_count = 0
        self.final_beam = None
        self.mission_start_lat = None
        self.mission_start_lon = None
        # Coordenadas del barrido real (se capturan al entrar a BC/BD, no al pulsar el botón).
        self.sweep_start_lat = None
        self.sweep_start_lon = None
        self.sweep_geo_samples = []
        # Control de envío automático del AoA al terminar el barrido.
        self.auto_aoa_sent = False
        self.auto_aoa_scheduled = False
        self.aoa_sent_deg = None

        self.t0 = time.time()
        self.t_buf = deque(maxlen=RSSI_PLOT_MAX_POINTS)
        self.raw_buf = deque(maxlen=RSSI_PLOT_MAX_POINTS)
        self.filt_buf = deque(maxlen=RSSI_PLOT_MAX_POINTS)
        self.rssi_plot_win = None
        self.pattern_plot_win = None

        self.discrete_mode = tk.BooleanVar(value=False)
        self.mode_text_var = tk.StringVar(value="Modo: continuo")

        self.build_ui()
        self.update_mode_widgets(force_default=True)
        self.update_time_estimate()

    @staticmethod
    def new_session_id() -> int:
        return random.randint(1, 65535)

    def build_ui(self):
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="COM SiK:").grid(row=0, column=0, sticky="w", padx=5, pady=5)
        self.sik_port_var = tk.StringVar(value=SIK_PORT)
        ttk.Entry(top, textvariable=self.sik_port_var, width=12).grid(row=0, column=1, padx=5, pady=5)

        ttk.Label(top, text="Baudios SiK:").grid(row=0, column=2, sticky="w", padx=5, pady=5)
        self.sik_baud_var = tk.StringVar(value=str(SIK_BAUD))
        ttk.Entry(top, textvariable=self.sik_baud_var, width=12).grid(row=0, column=3, padx=5, pady=5)

        ttk.Label(top, text="COM RSSI/MRAPC:").grid(row=0, column=4, sticky="w", padx=5, pady=5)
        self.rssi_port_var = tk.StringVar(value=RSSI_RADIO_PORT)
        ttk.Entry(top, textvariable=self.rssi_port_var, width=12).grid(row=0, column=5, padx=5, pady=5)

        ttk.Label(top, text="Baudios RSSI:").grid(row=0, column=6, sticky="w", padx=5, pady=5)
        self.rssi_baud_var = tk.StringVar(value=str(RSSI_RADIO_BAUD))
        ttk.Entry(top, textvariable=self.rssi_baud_var, width=12).grid(row=0, column=7, padx=5, pady=5)

        self.btn_connect = ttk.Button(top, text="Conectar", command=self.connect_all)
        self.btn_connect.grid(row=0, column=8, padx=10, pady=5)
        self.btn_disconnect = ttk.Button(top, text="Desconectar", command=self.disconnect_all, state="disabled")
        self.btn_disconnect.grid(row=0, column=9, padx=5, pady=5)

        params = ttk.LabelFrame(self.root, text="Parámetros de barrido", padding=10)
        params.pack(fill="x", padx=10, pady=5)
        self.params_frame = params

        self.mode_check = ttk.Checkbutton(params, textvariable=self.mode_text_var, variable=self.discrete_mode, command=self.toggle_sweep_mode)
        self.mode_check.grid(row=0, column=0, columnspan=2, sticky="w", padx=5, pady=5)

        self.height_var = tk.StringVar(value="8")
        self.yaw_rate_var = tk.StringVar(value="10.0")
        self.rotations_var = tk.IntVar(value=1)
        self.step_var = tk.StringVar(value="15")
        self.hold_var = tk.StringVar(value="2")
        self.peak_candidates_var = tk.IntVar(value=DEFAULT_ROBUST_PEAK_CANDIDATES)
        self.peak_neighbors_var = tk.IntVar(value=DEFAULT_ROBUST_PEAK_NEIGHBORS)

        ttk.Label(params, text="Altura [m]:").grid(row=1, column=0, sticky="w", padx=5, pady=5)
        self.height_entry = ttk.Entry(params, textvariable=self.height_var, width=12)
        self.height_entry.grid(row=1, column=1, padx=5, pady=5)

        ttk.Label(params, text="Velocidad yaw [deg/s]:").grid(row=1, column=2, sticky="w", padx=5, pady=5)
        self.yaw_rate_entry = ttk.Entry(params, textvariable=self.yaw_rate_var, width=12)
        self.yaw_rate_entry.grid(row=1, column=3, padx=5, pady=5)

        ttk.Label(params, text="Vueltas completas:").grid(row=1, column=4, sticky="w", padx=5, pady=5)
        self.rotations_spin = ttk.Spinbox(params, from_=1, to=3, increment=1, width=8, textvariable=self.rotations_var, state="readonly")
        self.rotations_spin.grid(row=1, column=5, padx=5, pady=5)

        self.step_label = ttk.Label(params, text="Paso angular [deg]:")
        self.step_entry = ttk.Entry(params, textvariable=self.step_var, width=12)
        self.hold_label = ttk.Label(params, text="Tiempo por ángulo [s]:")
        self.hold_entry = ttk.Entry(params, textvariable=self.hold_var, width=12)

        # Criterio robusto configurable. Estos parámetros solo afectan el cálculo FINAL
        # del AoA, RSSI máximo seleccionado y ancho de haz; no modifican el vuelo.
        ttk.Label(params, text="Máximos a evaluar:").grid(row=3, column=0, sticky="w", padx=5, pady=5)
        self.peak_candidates_spin = ttk.Spinbox(
            params, from_=MIN_ROBUST_PEAK_CANDIDATES, to=MAX_ROBUST_PEAK_CANDIDATES,
            increment=1, width=8, textvariable=self.peak_candidates_var, state="readonly"
        )
        self.peak_candidates_spin.grid(row=3, column=1, padx=5, pady=5)

        ttk.Label(params, text="Vecinos por lado:").grid(row=3, column=2, sticky="w", padx=5, pady=5)
        self.peak_neighbors_spin = ttk.Spinbox(
            params, from_=MIN_ROBUST_PEAK_NEIGHBORS, to=MAX_ROBUST_PEAK_NEIGHBORS,
            increment=1, width=8, textvariable=self.peak_neighbors_var, state="readonly"
        )
        self.peak_neighbors_spin.grid(row=3, column=3, padx=5, pady=5)
        self.mode_help_var = tk.StringVar(value="Continuo: usa yaw real para agrupar RSSI por ángulo. Velocidad por defecto: 10 deg/s.")
        ttk.Label(params, textvariable=self.mode_help_var).grid(row=4, column=0, columnspan=8, sticky="w", padx=5, pady=5)

        self.time_estimate_var = tk.StringVar(value="Tiempo estimado de barrido: ---")
        ttk.Label(params, textvariable=self.time_estimate_var).grid(row=5, column=0, columnspan=8, sticky="w", padx=5, pady=5)

        for var in (self.height_var, self.yaw_rate_var, self.step_var, self.hold_var):
            var.trace_add("write", lambda *_: self.update_time_estimate())
        self.rotations_var.trace_add("write", lambda *_: self.update_time_estimate())

        buttons = ttk.Frame(self.root, padding=10)
        buttons.pack(fill="x")
        self.btn_start = ttk.Button(buttons, text="Comenzar barrido", command=self.send_start_sweep, state="disabled")
        self.btn_start.pack(side="left", padx=5)
        self.btn_land = ttk.Button(buttons, text="Aterrizar", command=self.send_land, state="disabled")
        self.btn_land.pack(side="left", padx=5)
        self.btn_plot_rssi = ttk.Button(buttons, text="Graficar RSSI", command=self.on_plot_rssi)
        self.btn_plot_rssi.pack(side="left", padx=15)
        self.btn_plot_pattern = ttk.Button(buttons, text="Graficar patrón", command=self.on_plot_pattern)
        self.btn_plot_pattern.pack(side="left", padx=5)
        self.btn_save_csv = ttk.Button(buttons, text="Guardar logs CSV", command=self.save_logs_csv)
        self.btn_save_csv.pack(side="left", padx=15)

        progress_frame = ttk.LabelFrame(self.root, text="Avance del barrido", padding=10)
        progress_frame.pack(fill="x", padx=10, pady=5)
        self.progress_var = tk.DoubleVar(value=0.0)
        self.progress_bar = ttk.Progressbar(progress_frame, maximum=100.0, variable=self.progress_var)
        self.progress_bar.pack(side="left", fill="x", expand=True, padx=10, pady=5)
        self.progress_label_var = tk.StringVar(value="Progreso: 0.0 %")
        ttk.Label(progress_frame, textvariable=self.progress_label_var, width=18).pack(side="left", padx=10)

        link_frame = ttk.LabelFrame(self.root, text="Estado del enlace y del nodo", padding=10)
        link_frame.pack(fill="x", padx=10, pady=5)
        self.link_var = tk.StringVar(value="Enlace SiK: desconectado")
        self.rssi_link_var = tk.StringVar(value="MRAPC/RSSI: desconectado")
        self.session_var = tk.StringVar(value=f"Sesión: {self.session_id}")
        self.state_var = tk.StringVar(value="Estado: ---")
        self.ready_var = tk.StringVar(value="Ready PX4: ---")
        self.last_cmd_var = tk.StringVar(value="Último cmd: ---")
        self.exec_cmd_var = tk.StringVar(value="Exec cmd_id: 0")
        self.rx_invalid_var = tk.StringVar(value="RX inválidos: 0")
        self.pending_var = tk.StringVar(value="Pendiente: ninguno")
        ttk.Label(link_frame, textvariable=self.link_var).grid(row=0, column=0, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.rssi_link_var).grid(row=0, column=1, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.session_var).grid(row=0, column=2, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.state_var).grid(row=0, column=3, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.ready_var).grid(row=0, column=4, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.last_cmd_var).grid(row=0, column=5, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.exec_cmd_var).grid(row=0, column=6, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.rx_invalid_var).grid(row=1, column=0, sticky="w", padx=10, pady=3)
        ttk.Label(link_frame, textvariable=self.pending_var).grid(row=1, column=1, columnspan=6, sticky="w", padx=10, pady=3)

        telem_frame = ttk.LabelFrame(self.root, text="Telemetría real PX4 y RSSI", padding=10)
        telem_frame.pack(fill="x", padx=10, pady=5)
        self.px4_var = tk.StringVar(value="PX4: ---")
        self.mode_var = tk.StringVar(value="Modo de vuelo: ---")
        self.armed_var = tk.StringVar(value="Armado: ---")
        self.yaw_var = tk.StringVar(value="Azimut/Yaw: --- deg")
        self.yaw_ref_var = tk.StringVar(value="Yaw ref: --- deg")
        self.alt_var = tk.StringVar(value="Altura relativa: --- m")
        self.local_pos_var = tk.StringVar(value="Posición local NED: ---")
        self.global_pos_var = tk.StringVar(value="Lat/Lon: ---")
        self.landed_var = tk.StringVar(value="Estado tierra/aire: ---")
        self.health_var = tk.StringVar(value="Health: local --- | global --- | home ---")
        self.error_var = tk.StringVar(value="Error: ninguno")
        self.sweep_ready_var = tk.StringVar(value="Listo para barrido: ---")
        self.rssi_var = tk.StringVar(value="RSSI: ---")
        self.aoa_var = tk.StringVar(value="AoA: ---")
        self.samples_var = tk.StringVar(value="Muestras patrón: 0")
        ttk.Label(telem_frame, textvariable=self.px4_var).grid(row=0, column=0, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.mode_var).grid(row=0, column=1, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.armed_var).grid(row=0, column=2, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.yaw_var).grid(row=0, column=3, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.yaw_ref_var).grid(row=0, column=4, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.alt_var).grid(row=1, column=0, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.landed_var).grid(row=1, column=1, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.sweep_ready_var).grid(row=1, column=2, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.rssi_var).grid(row=1, column=3, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.aoa_var).grid(row=1, column=4, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.local_pos_var).grid(row=2, column=0, columnspan=3, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.global_pos_var).grid(row=2, column=3, columnspan=2, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.health_var).grid(row=3, column=0, columnspan=3, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.error_var).grid(row=3, column=3, sticky="w", padx=10, pady=3)
        ttk.Label(telem_frame, textvariable=self.samples_var).grid(row=3, column=4, sticky="w", padx=10, pady=3)

        log_frame = ttk.LabelFrame(self.root, text="Terminal de mensajes", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)
        self.log_text = ScrolledText(log_frame, wrap="word", height=18, state="disabled")
        self.log_text.pack(fill="both", expand=True)

        bottom = ttk.Frame(self.root, padding=10)
        bottom.pack(fill="x")
        ttk.Button(bottom, text="Limpiar mensajes", command=self.clear_log).pack(side="left")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def toggle_sweep_mode(self):
        self.update_mode_widgets(force_default=True)
        self.update_time_estimate()

    def update_mode_widgets(self, force_default=False):
        if self.discrete_mode.get():
            self.mode_text_var.set("Modo: discreto")
            self.mode_help_var.set("Discreto: promedia RSSI filtrado durante el tiempo detenido en cada ángulo.")
            if force_default:
                self.yaw_rate_var.set("45.0")
            self.step_label.grid(row=2, column=0, sticky="w", padx=5, pady=5)
            self.step_entry.grid(row=2, column=1, padx=5, pady=5)
            self.hold_label.grid(row=2, column=2, sticky="w", padx=5, pady=5)
            self.hold_entry.grid(row=2, column=3, padx=5, pady=5)
        else:
            self.mode_text_var.set("Modo: continuo")
            self.mode_help_var.set("Continuo: usa yaw real para agrupar RSSI por ángulo. Velocidad por defecto: 10 deg/s.")
            if force_default:
                self.yaw_rate_var.set("10.0")
            self.step_label.grid_remove()
            self.step_entry.grid_remove()
            self.hold_label.grid_remove()
            self.hold_entry.grid_remove()


    def reset_rssi_series(self, reason: str = "reinicio"):
        """
        Reinicia el filtro EMA y las colas de la gráfica RSSI.

        Se usa al comenzar un nuevo barrido o cuando se reinicia manualmente
        la serie RSSI para que la ventana de gráfica arranque limpia.
        """
        self.rssi_filter_state = None
        self.latest_rssi_filt = None
        # Reinicia también el origen de tiempo de la gráfica RSSI.
        # Así, al presionar Comenzar barrido, el eje temporal vuelve a 0 s.
        self.t0 = time.time()
        try:
            self.t_buf.clear()
            self.raw_buf.clear()
            self.filt_buf.clear()
        except Exception:
            pass
        if self.rssi_plot_win is not None and getattr(self.rssi_plot_win, "_running", False):
            try:
                self.rssi_plot_win.clear_plot()
            except Exception:
                pass
        if hasattr(self, "log_text"):
            self.log(f"[INFO] Se reinició la gráfica y el filtro RSSI por {reason}.")

    def reset_rssi_filter(self):
        """Compatibilidad con versiones previas: reinicia filtro y buffers RSSI."""
        self.reset_rssi_series(reason="reinicio del filtro RSSI")

    def update_time_estimate(self):
        try:
            yaw_rate = float(self.yaw_rate_var.get().strip())
            rotations = int(self.rotations_var.get())
            if yaw_rate <= 0:
                raise ValueError
            total_angle = 360.0 * rotations
            if self.discrete_mode.get():
                step = float(self.step_var.get().strip())
                hold = float(self.hold_var.get().strip())
                if step <= 0:
                    raise ValueError
                targets = [0.0]
                a = step
                while a < total_angle:
                    targets.append(a)
                    a += step
                targets.append(total_angle)
                sweep_s = total_angle / yaw_rate + len(targets) * hold
                detail = f"{len(targets)} puntos, incluye cierre a 360°"
            else:
                sweep_s = total_angle / yaw_rate
                detail = "giro continuo"
            self.time_estimate_var.set(
                f"Tiempo estimado de barrido: {sweep_s/60.0:.2f} min ({sweep_s:.1f} s, {detail}; sin contar despegue/aterrizaje)"
            )
        except Exception:
            self.time_estimate_var.set("Tiempo estimado de barrido: ---")

    def log(self, text):
        self.log_text.configure(state="normal")
        self.log_text.insert("end", text + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    def set_param_entries_state(self, state: str):
        widgets = (
            self.height_entry, self.yaw_rate_entry, self.step_entry, self.hold_entry,
            self.rotations_spin, self.peak_candidates_spin, self.peak_neighbors_spin,
            self.mode_check,
        )
        for widget in widgets:
            widget.config(state=state)
        if state == "normal":
            self.rotations_spin.config(state="readonly")
            self.peak_candidates_spin.config(state="readonly")
            self.peak_neighbors_spin.config(state="readonly")
        self.update_mode_widgets(force_default=False)

    def set_routine_active(self, active: bool):
        self.routine_active = active
        if not self.running:
            self.btn_start.config(state="disabled")
            self.btn_land.config(state="disabled")
            self.set_param_entries_state("normal")
            return
        self.btn_land.config(state="normal")
        if active:
            self.btn_start.config(state="disabled")
            self.set_param_entries_state("disabled")
        else:
            self.btn_start.config(state="normal")
            self.set_param_entries_state("normal")

    def set_progress(self, value):
        val = max(0.0, min(100.0, float(value or 0.0)))
        self.progress_var.set(val)
        self.progress_label_var.set(f"Progreso: {val:.1f} %")

    def next_seq(self):
        self.seq += 1
        return self.seq

    def next_cmd_id(self):
        self.cmd_id += 1
        return self.cmd_id

    def connect_all(self):
        if self.running:
            return
        try:
            sik_baud = int(self.sik_baud_var.get().strip())
            rssi_baud = int(self.rssi_baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Error", "Baudios inválidos")
            return
        sik_port = self.sik_port_var.get().strip()
        rssi_port = self.rssi_port_var.get().strip()

        self.session_id = self.new_session_id()
        self.session_var.set(f"Sesión: {self.session_id}")
        self.seq = 0
        self.cmd_id = 0
        self.pending_cmd = None
        self.last_ignored_session = None
        self.routine_active = False
        self.log_records = []
        self.pattern_samples.clear()
        self.pattern_stale_drop_count = 0
        self.final_beam = None
        self.logging_enabled = False
        self.rssi_filter_state = None
        self.latest_rssi_filt = None
        self.mission_start_wall_time = None
        self.mission_start_lat = None
        self.mission_start_lon = None
        self.sweep_start_lat = None
        self.sweep_start_lon = None
        self.sweep_geo_samples = []
        self.auto_aoa_sent = False
        self.auto_aoa_scheduled = False
        self.aoa_sent_deg = None
        self.set_progress(0.0)
        self.aoa_var.set("AoA: ---")

        try:
            self.sik_ser = serial.Serial(sik_port, sik_baud, timeout=0.05)
            self.sik_ser.reset_input_buffer()
            self.sik_ser.reset_output_buffer()
            time.sleep(1.0)
        except Exception as e:
            messagebox.showerror("Error SiK", f"No se pudo abrir {sik_port}:\n{e}")
            return

        try:
            self.rssi_node = ReliableRSSINode(rssi_port, rssi_baud, self.rssi_queue)
        except Exception as e:
            try:
                self.sik_ser.close()
            except Exception:
                pass
            self.sik_ser = None
            messagebox.showerror("Error RSSI/MRAPC", f"No se pudo abrir {rssi_port}:\n{e}")
            return

        self.running = True
        self.link_ok = False
        self.last_drone_msg_time = None
        self.rx_buffer = bytearray()
        self.rx_invalid_count = 0
        self.last_status_tuple = None
        self.rx_invalid_var.set("RX inválidos: 0")
        self.btn_connect.config(state="disabled")
        self.btn_disconnect.config(state="normal")
        self.btn_start.config(state="normal")
        self.btn_land.config(state="normal")
        self.set_param_entries_state("normal")

        self.log(f"[INFO] SiK abierto en {sik_port} a {sik_baud} baudios")
        self.log(f"[INFO] RSSI/MRAPC abierto en {rssi_port} a {rssi_baud} baudios")
        self.log("[INFO] Fase 4.9: PX4 + RSSI real sin compensación + patrón polar en vivo + AoA final.")
        self.log("[INFO] Logger interno: cada muestra RSSI se guarda con la última telemetría PX4 disponible.")
        self.log(f"[INFO] Sesión u={self.session_id} | RSSI cada {RSSI_QUERY_EVERY_S:.2f} s | EMA alpha={RSSI_ALPHA}")

        threading.Thread(target=self.receive_loop, daemon=True).start()
        threading.Thread(target=self.rssi_node.loop, daemon=True).start()
        self.root.after(100, self.gui_loop)
        self.root.after(50, self.rssi_gui_loop)
        self.root.after(int(HEARTBEAT_PERIOD_S * 1000), self.heartbeat_gui_loop)
        self.root.after(int(RSSI_QUERY_EVERY_S * 1000), self.rssi_query_tick)

    def disconnect_all(self):
        self.running = False
        time.sleep(0.2)
        try:
            if self.sik_ser is not None:
                self.sik_ser.close()
        except Exception:
            pass
        try:
            if self.rssi_node is not None:
                self.rssi_node.close()
        except Exception:
            pass
        self.sik_ser = None
        self.rssi_node = None
        self.btn_connect.config(state="normal")
        self.btn_disconnect.config(state="disabled")
        self.btn_start.config(state="disabled")
        self.btn_land.config(state="disabled")
        self.set_param_entries_state("normal")
        self.link_var.set("Enlace SiK: desconectado")
        self.rssi_link_var.set("MRAPC/RSSI: desconectado")
        self.pending_var.set("Pendiente: ninguno")
        self.log("[INFO] Desconectado")

    def write_frame(self, msg, verbose=True, prefix="TX"):
        if not self.running or self.sik_ser is None:
            self.log("[ERROR] No hay conexión serial SiK")
            return False
        frame = make_frame(msg)
        try:
            with self.tx_lock:
                self.sik_ser.write(frame)
                self.sik_ser.flush()
        except Exception as e:
            self.log(f"[ERROR TX] {e}")
            return False
        if verbose:
            self.log(f"[{prefix}] {msg}")
        return True

    def send_message_unreliable(self, msg, verbose=False):
        msg = dict(msg)
        msg["u"] = self.session_id
        msg["q"] = self.next_seq()
        self.write_frame(msg, verbose=verbose)

    def get_reliable_params(self, msg_type: str):
        if msg_type == "L":
            return LAND_MAX_RETRIES, LAND_ACK_TIMEOUT_S
        if msg_type == "Y":
            return AOA_MAX_RETRIES, AOA_ACK_TIMEOUT_S
        return DEFAULT_MAX_RETRIES, DEFAULT_ACK_TIMEOUT_S

    def send_reliable_command(self, msg) -> bool:
        msg = dict(msg)
        msg_type = msg.get("t", "?")
        if self.pending_cmd is not None:
            pending_type = self.pending_cmd.get("cmd_type", "?")
            if msg_type == "L" and pending_type != "L":
                self.log(f"[PRIORIDAD] Cancelando pendiente {pending_type} para enviar aterrizar")
                self.pending_cmd = None
                self.update_pending_label()
            else:
                messagebox.showwarning("Comando pendiente", "Ya hay un comando esperando ACK o STATUS de confirmación.")
                return False
        max_retries, ack_timeout_s = self.get_reliable_params(msg_type)
        msg["u"] = self.session_id
        msg["q"] = self.next_seq()
        msg["c"] = self.next_cmd_id()
        ok = self.write_frame(msg, verbose=True, prefix=f"TX intento 1/{max_retries}")
        if not ok:
            return False
        self.pending_cmd = {
            "session_id": self.session_id,
            "cmd_id": msg["c"],
            "msg": msg,
            "attempts": 1,
            "last_send": time.monotonic(),
            "cmd_type": msg_type,
            "max_retries": max_retries,
            "ack_timeout_s": ack_timeout_s,
        }
        self.update_pending_label()
        return True

    def update_pending_label(self):
        if self.pending_cmd is None:
            self.pending_var.set("Pendiente: ninguno")
        else:
            self.pending_var.set(
                f"Pendiente: {self.pending_cmd['cmd_type']} u={self.pending_cmd['session_id']} "
                f"cmd_id={self.pending_cmd['cmd_id']} intento={self.pending_cmd['attempts']}/{self.pending_cmd['max_retries']}"
            )

    def check_pending_command(self):
        if self.pending_cmd is None:
            return
        now = time.monotonic()
        if now - self.pending_cmd["last_send"] < self.pending_cmd["ack_timeout_s"]:
            return
        if self.pending_cmd["attempts"] >= self.pending_cmd["max_retries"]:
            self.log(
                f"[ACK TIMEOUT] No llegó confirmación para {self.pending_cmd['cmd_type']} "
                f"u={self.pending_cmd['session_id']} cmd_id={self.pending_cmd['cmd_id']}"
            )
            timed_out_type = self.pending_cmd.get("cmd_type")
            self.pending_cmd = None
            self.update_pending_label()
            if timed_out_type == "S":
                self.set_routine_active(False)
            elif timed_out_type == "Y":
                self.auto_aoa_sent = False
                self.log("[SAFE] No se confirmó el AoA. Se solicita aterrizaje prioritario.")
                self.send_land()
            return
        self.pending_cmd["attempts"] += 1
        self.pending_cmd["last_send"] = now
        attempt = self.pending_cmd["attempts"]
        msg = self.pending_cmd["msg"]
        self.write_frame(msg, verbose=True, prefix=f"RETX intento {attempt}/{self.pending_cmd['max_retries']}")
        self.update_pending_label()

    def heartbeat_gui_loop(self):
        if not self.running:
            return
        self.send_message_unreliable({"t": "H"}, verbose=False)
        self.root.after(int(HEARTBEAT_PERIOD_S * 1000), self.heartbeat_gui_loop)

    def rssi_query_tick(self):
        if self.running and self.rssi_node is not None:
            try:
                self.rssi_node.request_rssi()
            except Exception as e:
                self.log(f"[RSSI ERROR] {e}")
        if self.running:
            self.root.after(int(RSSI_QUERY_EVERY_S * 1000), self.rssi_query_tick)

    def current_config_snapshot(self) -> dict:
        return {
            "modo_barrido_gui": "discreto" if self.discrete_mode.get() else "continuo",
            "altura_cfg_m": self.height_var.get().strip(),
            "velocidad_yaw_cfg_deg_s": self.yaw_rate_var.get().strip(),
            "vueltas_cfg": self.rotations_var.get(),
            "paso_cfg_deg": self.step_var.get().strip() if self.discrete_mode.get() else "",
            "tiempo_por_angulo_cfg_s": self.hold_var.get().strip() if self.discrete_mode.get() else "",
            "maximos_candidatos_cfg": self.peak_candidates_var.get(),
            "vecinos_por_lado_cfg": self.peak_neighbors_var.get(),
            "continuous_bin_deg": CONTINUOUS_POLAR_BIN_DEG,
            "rssi_valid_min_dbm": VALID_RSSI_MIN_DBM,
            "rssi_valid_max_dbm": VALID_RSSI_MAX_DBM,
        }

    def validate_common_params(self):
        try:
            height_m = float(self.height_var.get().strip())
            yaw_rate = float(self.yaw_rate_var.get().strip())
            rotations = int(self.rotations_var.get())
        except ValueError:
            messagebox.showerror("Error", "Altura, velocidad angular o número de vueltas inválidos")
            return None
        if not (1.0 <= height_m <= 15.0):
            messagebox.showerror("Error", "La altura debe estar entre 1 y 15 m")
            return None
        if not (0.05 <= yaw_rate <= 45.0):
            messagebox.showerror("Error", "La velocidad yaw debe estar entre 0.05 y 45 deg/s")
            return None
        if rotations < 1 or rotations > 3:
            messagebox.showerror("Error", "Las vueltas deben estar entre 1 y 3")
            return None
        return height_m, yaw_rate, rotations

    def send_start_sweep(self):
        params = self.validate_common_params()
        if params is None:
            return
        height_m, yaw_rate, rotations = params
        try:
            peak_candidates = int(self.peak_candidates_var.get())
            peak_neighbors = int(self.peak_neighbors_var.get())
        except (TypeError, ValueError, tk.TclError):
            messagebox.showerror("Error", "Número de máximos o vecinos inválido")
            return
        if not (MIN_ROBUST_PEAK_CANDIDATES <= peak_candidates <= MAX_ROBUST_PEAK_CANDIDATES):
            messagebox.showerror("Error", "Los máximos a evaluar deben estar entre 1 y 5")
            return
        if not (MIN_ROBUST_PEAK_NEIGHBORS <= peak_neighbors <= MAX_ROBUST_PEAK_NEIGHBORS):
            messagebox.showerror("Error", "Los vecinos por lado deben estar entre 1 y 3")
            return

        mode = "D" if self.discrete_mode.get() else "C"
        msg = {"t": "S", "b": mode, "h": height_m, "w": yaw_rate, "n": rotations}
        if mode == "D":
            try:
                step_deg = float(self.step_var.get().strip())
                hold_s = float(self.hold_var.get().strip())
            except ValueError:
                messagebox.showerror("Error", "Paso angular o tiempo por ángulo inválidos")
                return
            if not (1.0 <= step_deg <= 180.0):
                messagebox.showerror("Error", "El paso angular debe estar entre 1 y 180 deg")
                return
            if not (0.1 <= hold_s <= 120.0):
                messagebox.showerror("Error", "El tiempo por ángulo debe estar entre 0.1 y 120 s")
                return
            msg["p"] = step_deg
            msg["d"] = hold_s
        else:
            step_deg = CONTINUOUS_POLAR_BIN_DEG
            hold_s = ""

        self.current_mission_cfg = {
            "mode": mode,
            "height_m": height_m,
            "yaw_rate_deg_s": yaw_rate,
            "rotations": rotations,
            "step_deg": float(self.step_var.get().strip()) if mode == "D" else CONTINUOUS_POLAR_BIN_DEG,
            "hold_s": float(self.hold_var.get().strip()) if mode == "D" else 0.0,
            "peak_candidates": peak_candidates,
            "peak_neighbors": peak_neighbors,
            "rssi_valid_min_dbm": VALID_RSSI_MIN_DBM,
            "rssi_valid_max_dbm": VALID_RSSI_MAX_DBM,
        }
        self.log_records = []
        self.pattern_samples.clear()
        self.pattern_stale_drop_count = 0
        self.final_beam = None
        self.mission_start_wall_time = time.time()
        self.logging_enabled = True
        self.reset_rssi_series(reason="inicio de misión")
        # Posición al lanzar la misión. El inicio real del barrido se captura después, al entrar a BC/BD.
        self.mission_start_lat = safe_float(self.last_status_msg.get("la"))
        self.mission_start_lon = safe_float(self.last_status_msg.get("lo"))
        self.sweep_start_lat = None
        self.sweep_start_lon = None
        self.sweep_geo_samples = []
        self.auto_aoa_sent = False
        self.auto_aoa_scheduled = False
        self.aoa_sent_deg = None
        self.aoa_var.set("AoA: pendiente hasta finalizar barrido")
        self.log(
            f"[AOA CFG] Máximos candidatos={peak_candidates} | vecinos por lado={peak_neighbors}. "
            "AoA/RSSI máximo/BW se calcularán solo al final."
        )
        self.log("[RSSI] Sin compensación: el RSSI válido entra directo al filtro EMA.")
        self.samples_var.set("Muestras patrón: 0")
        self.set_progress(0.0)
        sent = self.send_reliable_command(msg)
        if sent:
            self.set_routine_active(True)
        else:
            self.logging_enabled = False
            self.current_mission_cfg = {}
            self.aoa_var.set("AoA: ---")
            self.log("[ERROR] No se pudo transmitir el comando de inicio de barrido.")

    def send_land(self):
        self.send_reliable_command({"t": "L"})

    def receive_loop(self):
        while self.running:
            try:
                chunk = self.sik_ser.read(self.sik_ser.in_waiting or 1)
                if not chunk:
                    continue
                self.rx_buffer.extend(chunk)
                if len(self.rx_buffer) > 8192:
                    last_start = self.rx_buffer.rfind(b"$")
                    self.rx_buffer = self.rx_buffer[last_start:] if last_start >= 0 else bytearray()
                while b"\n" in self.rx_buffer:
                    raw_line, _, rest = self.rx_buffer.partition(b"\n")
                    self.rx_buffer = bytearray(rest)
                    line = raw_line.decode("utf-8", errors="ignore").strip()
                    if not line:
                        continue
                    msg, err = parse_frame(line)
                    if err is not None:
                        self.rx_queue.put({"t": "invalid_frame", "e": err, "raw": line})
                    else:
                        self.rx_queue.put(msg)
            except Exception as e:
                self.rx_queue.put({"t": "local_error", "e": str(e)})
                self.running = False
                break

    def gui_loop(self):
        if not self.running:
            return
        now = time.monotonic()
        try:
            while True:
                msg = self.rx_queue.get_nowait()
                if msg.get("t") not in ["invalid_frame", "local_error"]:
                    self.last_drone_msg_time = now
                    if not self.link_ok:
                        self.link_ok = True
                        self.link_var.set("Enlace SiK: OK")
                        self.log("[LINK] Comunicación con Raspberry establecida")
                self.handle_message(msg)
        except queue.Empty:
            pass
        if self.last_drone_msg_time is not None and (now - self.last_drone_msg_time > LINK_TIMEOUT_S):
            if self.link_ok:
                self.link_ok = False
                self.link_var.set("Enlace SiK: PERDIDO")
                self.log("[LINK] Comunicación con Raspberry perdida")
        self.check_pending_command()
        self.root.after(100, self.gui_loop)

    def rssi_gui_loop(self):
        if not self.running:
            return
        try:
            while True:
                ev = self.rssi_queue.get_nowait()
                et = ev[0]
                if et == "mrapc":
                    self.rssi_link_var.set(f"MRAPC/RSSI: detectado 0x{ev[1]:04X}")
                    self.log(f"[RSSI] Detectado MRAPC=0x{ev[1]:04X}")
                elif et == "radio_ready":
                    self.rssi_link_var.set("MRAPC/RSSI: listo")
                    self.log("[RSSI] Radio MRAPC listo para consultar RSSI")
                elif et == "rssi":
                    raw, dbm, filt = ev[1], ev[2], ev[3]
                    self.handle_rssi_sample(raw, dbm, filt)
                elif et == "rssi_error":
                    self.log(f"[RSSI ERROR] {ev[1]}")
        except queue.Empty:
            pass
        self.root.after(50, self.rssi_gui_loop)

    def handle_rssi_sample(self, raw, dbm, _filt_from_node=None):
        # El radio entrega RSSI como raw byte; dbm = raw - 128.
        # Esta versión no aplica compensación. Además descarta lecturas fuera del
        # rango físico esperado para evitar deformar la gráfica y contaminar el EMA.
        dbm_value = float(dbm)
        if dbm_value < VALID_RSSI_MIN_DBM or dbm_value > VALID_RSSI_MAX_DBM:
            self.latest_rssi_raw = raw
            self.latest_rssi_dbm = dbm_value
            self.latest_rssi_time = time.time()
            self.rssi_var.set(
                f"RSSI: {dbm_value:.0f} dBm (raw {raw}) | muestra descartada fuera de rango"
            )
            if self.logging_enabled:
                self.log(
                    f"[RSSI WARN] Muestra descartada fuera de rango: "
                    f"{dbm_value:.1f} dBm, rango válido [{VALID_RSSI_MIN_DBM:.0f}, {VALID_RSSI_MAX_DBM:.0f}] dBm"
                )
            return

        if self.rssi_filter_state is None:
            self.rssi_filter_state = float(dbm_value)
        else:
            self.rssi_filter_state = (1.0 - RSSI_ALPHA) * self.rssi_filter_state + RSSI_ALPHA * float(dbm_value)
        filt = float(self.rssi_filter_state)

        self.latest_rssi_raw = raw
        self.latest_rssi_dbm = dbm_value
        self.latest_rssi_filt = filt
        self.latest_rssi_time = time.time()

        self.rssi_var.set(f"RSSI: {dbm_value:.0f} dBm (raw {raw}) | filtrado {filt:.1f} dBm")

        t = time.time() - self.t0
        # La línea azul representa el RSSI válido recibido del MRAPC.
        self.t_buf.append(t)
        self.raw_buf.append(float(dbm_value))
        self.filt_buf.append(float(filt))

        if self.logging_enabled:
            record = self.make_log_record(raw, dbm_value, filt)
            self.log_records.append(record)
            self.add_pattern_sample_from_record(record)

    def make_log_record(self, raw, dbm_value, filt):
        msg = dict(self.last_status_msg or {})
        cfg = self.current_mission_cfg or {}
        now = time.time()
        mission_elapsed = ""
        if self.mission_start_wall_time is not None:
            mission_elapsed = round(now - self.mission_start_wall_time, 3)
        yaw_real = safe_float(msg.get("y"))
        yaw_ref = safe_float(msg.get("yr"))
        yaw_err = ""
        if yaw_real is not None and yaw_ref is not None:
            yaw_err = round(wrap180(yaw_real - yaw_ref), 3)
        status_age_s = ""
        if self.last_drone_msg_time is not None:
            status_age_s = round(time.monotonic() - self.last_drone_msg_time, 3)
        row = {
            "timestamp_local": _dt.datetime.now().isoformat(timespec="milliseconds"),
            "mission_elapsed_s": mission_elapsed,
            "telemetry_age_s": status_age_s,
            "session_id": msg.get("u", self.session_id),
            "estado_codigo": msg.get("s", ""),
            "estado": decode(STATE_NAMES, msg.get("s", ""), msg.get("s", "")),
            "progreso_pct": msg.get("pr", ""),
            "px4_conectado": msg.get("px", ""),
            "armado": msg.get("ar", ""),
            "modo_vuelo_codigo": msg.get("m", ""),
            "modo_vuelo": decode(MODE_NAMES, msg.get("m", ""), msg.get("m", "")),
            "yaw_real_deg": msg.get("y", ""),
            "yaw_ref_deg": msg.get("yr", ""),
            "yaw_error_deg": yaw_err,
            "angulo_bin_reportado_deg": msg.get("ba", ""),
            "measurement_active": msg.get("ma", ""),
            "sweep_mode_active": msg.get("bm", ""),
            "altura_relativa_m": msg.get("z", ""),
            "local_norte_m": msg.get("pn", ""),
            "local_este_m": msg.get("pe", ""),
            "local_down_m": msg.get("pd", ""),
            "lat_deg": msg.get("la", ""),
            "lon_deg": msg.get("lo", ""),
            "lat_inicio_mision_deg": self.mission_start_lat if self.mission_start_lat is not None else "",
            "lon_inicio_mision_deg": self.mission_start_lon if self.mission_start_lon is not None else "",
            "lat_inicio_barrido_deg": self.sweep_start_lat if self.sweep_start_lat is not None else "",
            "lon_inicio_barrido_deg": self.sweep_start_lon if self.sweep_start_lon is not None else "",
            "aoa_enviado_deg": self.aoa_sent_deg if self.aoa_sent_deg is not None else "",
            "health_local": msg.get("hl", ""),
            "health_global": msg.get("hg", ""),
            "health_home": msg.get("hh", ""),
            "landed_state_codigo": msg.get("ls", ""),
            "landed_state": decode(LANDED_NAMES, msg.get("ls", ""), msg.get("ls", "")),
            "ultimo_cmd_codigo": msg.get("lc", ""),
            "ultimo_cmd": decode(CMD_NAMES, msg.get("lc", ""), msg.get("lc", "")),
            "exec_cmd_id": msg.get("ec", ""),
            "error_codigo": msg.get("er", ""),
            "rssi_raw": raw,
            "rssi_dbm": round(float(dbm_value), 3),
            "rssi_filtrado_dbm": round(float(filt), 3),
            "pattern_angle_bin_deg": "",
            "pattern_valid_sample": 0,
            "pattern_skip_reason": "",
            "mission_mode_cfg": cfg.get("mode", ""),
            "height_cfg_m": cfg.get("height_m", ""),
            "yaw_rate_cfg_deg_s": cfg.get("yaw_rate_deg_s", ""),
            "rotations_cfg": cfg.get("rotations", ""),
            "step_cfg_deg": cfg.get("step_deg", ""),
            "hold_cfg_s": cfg.get("hold_s", ""),
            "peak_candidates_cfg": cfg.get("peak_candidates", ""),
            "peak_neighbors_per_side_cfg": cfg.get("peak_neighbors", ""),
            "rssi_valid_min_dbm_cfg": cfg.get("rssi_valid_min_dbm", VALID_RSSI_MIN_DBM),
            "rssi_valid_max_dbm_cfg": cfg.get("rssi_valid_max_dbm", VALID_RSSI_MAX_DBM),
        }
        return row

    def add_pattern_sample_from_record(self, row):
        # Solo se permite agregar al patrón cuando la telemetría SiK que contiene
        # yaw/ba/ma es reciente. Si el enlace se cae, el RSSI MRAPC puede seguir
        # llegando, pero NO debe asociarse al último ángulo viejo.
        if int(row.get("measurement_active") or 0) != 1:
            row["pattern_skip_reason"] = "ma0"
            return
        age = safe_float(row.get("telemetry_age_s"))
        if age is None or age > MAX_STATUS_AGE_FOR_PATTERN_S or not self.link_ok:
            row["pattern_skip_reason"] = "telemetry_stale"
            self.pattern_stale_drop_count += 1
            if self.pattern_stale_drop_count <= 3 or self.pattern_stale_drop_count % 25 == 0:
                self.log(
                    f"[PATTERN] Muestra RSSI descartada: telemetría/yaw obsoletos "
                    f"(age={age}, link_ok={self.link_ok})."
                )
            return
        # Protección adicional: no se agregan muestras al patrón si PX4 no está
        # reportando OFFBOARD. Esto evita llenar bins con datos tomados en HOLD
        # durante transiciones o pérdidas momentáneas de modo.
        if str(row.get("modo_vuelo_codigo") or "") != "O":
            row["pattern_skip_reason"] = "not_offboard"
            return
        filt = safe_float(row.get("rssi_filtrado_dbm"))
        if filt is None:
            row["pattern_skip_reason"] = "no_rssi"
            return
        mode = row.get("sweep_mode_active") or self.current_mission_cfg.get("mode", "")
        if mode == "C":
            yaw_real = safe_float(row.get("yaw_real_deg"))
            if yaw_real is None:
                row["pattern_skip_reason"] = "no_yaw"
                return
            angle = bin_angle(yaw_real, CONTINUOUS_POLAR_BIN_DEG)
        elif mode == "D":
            a = safe_float(row.get("angulo_bin_reportado_deg"))
            if a is None:
                a = safe_float(row.get("yaw_ref_deg"))
            if a is None:
                row["pattern_skip_reason"] = "no_angle"
                return
            step = float(self.current_mission_cfg.get("step_deg") or self.step_var.get() or 30.0)
            angle = bin_angle(a, step)
        else:
            row["pattern_skip_reason"] = "no_mode"
            return
        row["pattern_angle_bin_deg"] = angle
        row["pattern_valid_sample"] = 1
        row["pattern_skip_reason"] = ""
        with self.pattern_lock:
            self.pattern_samples[angle].append(filt)
            count = sum(len(v) for v in self.pattern_samples.values())
        self.samples_var.set(f"Muestras patrón: {count}")

    def get_pattern_results(self):
        with self.pattern_lock:
            return {k: sum(v) / len(v) for k, v in self.pattern_samples.items() if v}

    def get_expected_angles(self):
        results = self.get_pattern_results()
        if not results:
            return []
        mode = self.current_mission_cfg.get("mode", "C")
        step = CONTINUOUS_POLAR_BIN_DEG if mode == "C" else float(self.current_mission_cfg.get("step_deg") or 30.0)
        expected = []
        a = 0.0
        while a < 360.0 - 1e-9:
            b = round(a, 3)
            if b in results:
                expected.append(b)
            a += step
        # Si por redondeos hay ángulos no previstos, los incluimos ordenados.
        for k in sorted(results.keys()):
            if k not in expected:
                expected.append(k)
        return expected

    def compute_beam_from_final_pattern(self):
        """Calcula AoA, RSSI del máximo elegido y BW solo cuando el barrido ya terminó."""
        results = self.get_pattern_results()
        expected = self.get_expected_angles()
        if len(results) < 3:
            return None
        peak_candidates = int(self.current_mission_cfg.get(
            "peak_candidates", DEFAULT_ROBUST_PEAK_CANDIDATES
        ))
        peak_neighbors = int(self.current_mission_cfg.get(
            "peak_neighbors", DEFAULT_ROBUST_PEAK_NEIGHBORS
        ))
        return compute_beamwidth_info(
            results, expected,
            peak_candidates=peak_candidates,
            neighbors_per_side=peak_neighbors,
        )

    def get_final_beam(self):
        """La gráfica polar consulta solo el análisis final ya congelado."""
        return self.final_beam

    def capture_sweep_geography_from_status(self, msg):
        """Captura y acumula coordenadas globales durante el barrido activo."""
        state = msg.get("s", "")
        if state not in ["BC", "BD"]:
            return
        lat = safe_float(msg.get("la"))
        lon = safe_float(msg.get("lo"))
        if lat is None or lon is None:
            return
        if self.sweep_start_lat is None:
            self.sweep_start_lat = lat
            self.sweep_start_lon = lon
            self.log(f"[GPS] Inicio real del barrido: lat={lat:.7f}, lon={lon:.7f}")
        self.sweep_geo_samples.append((lat, lon))

    def get_sweep_geo_summary(self):
        vals = [(a, b) for a, b in self.sweep_geo_samples if a is not None and b is not None]
        if not vals:
            return None, None
        return (sum(v[0] for v in vals) / len(vals), sum(v[1] for v in vals) / len(vals))

    def schedule_auto_send_aoa(self):
        if self.auto_aoa_sent or self.auto_aoa_scheduled:
            return
        self.auto_aoa_scheduled = True
        self.log("[AOA] Barrido finalizado. Cerrando muestras RSSI antes del cálculo único de AoA/RSSI máximo/BW...")
        self.root.after(AOA_AUTO_SEND_DELAY_MS, self.auto_send_aoa_if_ready)

    def auto_send_aoa_if_ready(self):
        self.auto_aoa_scheduled = False
        if self.auto_aoa_sent or not self.running:
            return
        # Si todavía existe otro comando pendiente, esperamos sin perder la oportunidad de enviar el AoA.
        if self.pending_cmd is not None:
            self.auto_aoa_scheduled = True
            self.root.after(300, self.auto_send_aoa_if_ready)
            return
        beam = self.compute_beam_from_final_pattern()
        if beam is None:
            self.log("[AOA ERROR] No fue posible calcular un AoA robusto. Se solicita aterrizaje prioritario.")
            self.send_land()
            return
        aoa = wrap360(float(beam["peak_angle"]))
        self.final_beam = beam
        self.log(
            f"[AOA FINAL] Criterio usado: máximos={self.current_mission_cfg.get('peak_candidates')} | "
            f"vecinos/lado={self.current_mission_cfg.get('peak_neighbors')}"
        )
        self.aoa_sent_deg = round(aoa, 3)
        self.aoa_var.set(format_candidates_multiline(beam) + " | enviando")
        cand_summary = '; '.join([f"AoA {c['rank']}={c['angle']:.1f}°/{c['peak_dbm']:.1f} dBm" for c in beam.get('candidates', [])])
        self.log(
            f"[AOA] Enviando AoA estimado al dron: {aoa:.3f} deg "
            f"(RSSI={beam['peak_dbm']:.2f} dBm, media vecinal={beam['robust_mean_dbm']:.2f} dBm) | {cand_summary}"
        )
        sent = self.send_reliable_command({"t": "Y", "g": round(aoa, 3)})
        if sent:
            self.auto_aoa_sent = True
        else:
            self.auto_aoa_scheduled = True
            self.root.after(300, self.auto_send_aoa_if_ready)

    def handle_message(self, msg):
        msg_type = msg.get("t", "?")
        if msg_type == "H":
            return
        if msg_type == "T":
            self.handle_status(msg)
        elif msg_type == "A":
            self.handle_ack(msg)
        elif msg_type == "invalid_frame":
            self.rx_invalid_count += 1
            self.rx_invalid_var.set(f"RX inválidos: {self.rx_invalid_count}")
            if self.rx_invalid_count <= 5 or self.rx_invalid_count % 10 == 0:
                self.log(f"[RX INVALIDO #{self.rx_invalid_count}] {msg.get('e')}")
        elif msg_type == "local_error":
            self.log(f"[ERROR LOCAL] {msg.get('e')}")
        else:
            self.log(f"[RX] {msg}")

    def handle_ack(self, msg):
        msg_session = msg.get("u")
        if msg_session != self.session_id:
            self.log_ignored_session(msg_session)
            return
        ack_cmd_id = msg.get("c")
        ack_type = msg.get("a")
        accepted = bool(msg.get("ok", 0))
        reason = msg.get("e", "")
        cname = decode(CMD_NAMES, ack_type, ack_type)
        self.log(f"[ACK] u={msg_session} cmd={cname}({ack_type}) cmd_id={ack_cmd_id} accepted={accepted} reason={reason}")
        if self.pending_cmd is not None and msg_session == self.pending_cmd["session_id"] and ack_cmd_id == self.pending_cmd["cmd_id"]:
            pending_type = self.pending_cmd.get("cmd_type")
            self.pending_cmd = None
            self.update_pending_label()
            if pending_type == "S" and not accepted:
                self.set_routine_active(False)
                self.logging_enabled = False
            elif pending_type == "Y":
                if accepted:
                    self.log("[AOA] Comando de orientación confirmado por ACK.")
                    if self.final_beam is not None:
                        self.aoa_var.set(format_candidates_multiline(self.final_beam) + " | confirmado")
                else:
                    self.auto_aoa_sent = False
                    self.log(f"[SAFE] Raspberry rechazó AoA ({reason}). Se solicita aterrizaje prioritario.")
                    self.send_land()

    def log_ignored_session(self, msg_session):
        if msg_session != self.last_ignored_session:
            self.last_ignored_session = msg_session
            self.log(f"[INFO] Ignorando mensaje de sesión anterior/distinta u={msg_session}")

    def handle_status(self, msg):
        msg_session = msg.get("u", 0)
        if msg_session not in [self.session_id, 0, None]:
            self.log_ignored_session(msg_session)
            return
        self.last_status_msg = dict(msg)
        self.capture_sweep_geography_from_status(msg)

        state = msg.get("s", "?")
        ready = bool(msg.get("r", 0))
        link_ok_rpi = bool(msg.get("lk", 0))
        last_cmd = msg.get("lc", "?")
        exec_cmd_id = msg.get("ec", 0)
        rx_invalid_remote = msg.get("x", 0)
        progress = msg.get("pr", 0.0)
        px4 = bool(msg.get("px", 0))
        armed = bool(msg.get("ar", 0))
        mode_code = msg.get("m", "U")
        yaw = msg.get("y", None)
        yaw_ref = msg.get("yr", None)
        alt = msg.get("z", None)
        pn = msg.get("pn", None)
        pe = msg.get("pe", None)
        pd = msg.get("pd", None)
        lat = msg.get("la", None)
        lon = msg.get("lo", None)
        hl = bool(msg.get("hl", 0))
        hg = bool(msg.get("hg", 0))
        hh = bool(msg.get("hh", 0))
        landed = msg.get("ls", "U")
        err = msg.get("er", "")

        self.state_var.set(f"Estado: {decode(STATE_NAMES, state)} ({state})")
        self.ready_var.set(f"Ready PX4: {ready}")
        self.last_cmd_var.set(f"Último cmd: {decode(CMD_NAMES, last_cmd, last_cmd)} ({last_cmd})")
        self.exec_cmd_var.set(f"Exec cmd_id: {exec_cmd_id}")
        self.rx_invalid_var.set(f"RX inválidos: {self.rx_invalid_count} | RPi: {rx_invalid_remote}")
        self.link_var.set(f"Enlace SiK: {'OK' if link_ok_rpi else 'PERDIDO'}")
        self.set_progress(progress)
        self.px4_var.set(f"PX4: {'conectado' if px4 else 'sin conexión'}")
        self.mode_var.set(f"Modo de vuelo: {decode(MODE_NAMES, mode_code)} ({mode_code})")
        self.armed_var.set(f"Armado: {'SÍ' if armed else 'NO'}")
        self.yaw_var.set("Azimut/Yaw: --- deg" if yaw is None else f"Azimut/Yaw: {float(yaw):.1f} deg")
        self.yaw_ref_var.set("Yaw ref: --- deg" if yaw_ref is None else f"Yaw ref: {float(yaw_ref):.1f} deg")
        self.alt_var.set("Altura relativa: --- m" if alt is None else f"Altura relativa: {float(alt):.1f} m")
        if pn is None or pe is None or pd is None:
            self.local_pos_var.set("Posición local NED: ---")
        else:
            self.local_pos_var.set(f"Posición local NED: N={float(pn):.2f} m | E={float(pe):.2f} m | D={float(pd):.2f} m")
        if lat is None or lon is None:
            self.global_pos_var.set("Lat/Lon: ---")
        else:
            self.global_pos_var.set(f"Lat/Lon: {float(lat):.7f}, {float(lon):.7f}")
        self.landed_var.set(f"Estado tierra/aire: {decode(LANDED_NAMES, landed)} ({landed})")
        self.health_var.set(f"Health: local {'OK' if hl else 'NO'} | global {'OK' if hg else 'NO'} | home {'OK' if hh else 'NO'}")
        self.error_var.set(f"Error: {err if err else 'ninguno'}")
        sweep_ready = px4 and (not armed) and mode_code == "P" and hl and hg and hh and landed == "G"
        self.sweep_ready_var.set(f"Listo para barrido: {'SÍ' if sweep_ready else 'NO'}")

        # Fase 4.2: cuando la Raspberry termina el barrido queda en OFFBOARD esperando el AoA.
        # La estación calcula el patrón final y envía automáticamente el ángulo absoluto.
        if state == "QA":
            self.schedule_auto_send_aoa()

        if state in ACTIVE_STATES:
            if not self.routine_active:
                self.set_routine_active(True)
        elif state == "I" and last_cmd in ["SC", "LC", "SE", "N"]:
            if self.routine_active and self.pending_cmd is None:
                self.set_routine_active(False)
            if last_cmd in ["SC", "SE", "LC"]:
                self.logging_enabled = False
                # No se recalcula el AoA aquí. El resultado final se congela al terminar el barrido.

        if self.pending_cmd is not None and msg_session == self.pending_cmd["session_id"] and exec_cmd_id == self.pending_cmd["cmd_id"]:
            pending_type = self.pending_cmd["cmd_type"]
            self.log(f"[STATUS CONFIRM] u={msg_session} {pending_type} cmd_id={exec_cmd_id} confirmado por status ec")
            self.pending_cmd = None
            self.update_pending_label()
            if pending_type == "Y" and self.final_beam is not None:
                self.log("[AOA] Comando de orientación confirmado por STATUS ec.")
                self.aoa_var.set(format_candidates_multiline(self.final_beam) + " | confirmado")

        status_tuple = (msg_session, state, ready, link_ok_rpi, last_cmd, exec_cmd_id, rx_invalid_remote, progress, px4, armed, mode_code, yaw, yaw_ref, alt, pn, pe, pd, lat, lon, hl, hg, hh, landed, err, msg.get("ma", 0), msg.get("ba", None))
        now = time.monotonic()
        if status_tuple != self.last_status_tuple or (now - self.last_status_log_time) > STATUS_LOG_PERIOD_S:
            self.log(
                f"[STATUS] u={msg_session} s={state}({decode(STATE_NAMES, state)}) r={int(ready)} lk={int(link_ok_rpi)} "
                f"lc={last_cmd}({decode(CMD_NAMES, last_cmd, last_cmd)}) ec={exec_cmd_id} x={rx_invalid_remote} pr={progress} | "
                f"px={int(px4)} ar={int(armed)} m={mode_code}({decode(MODE_NAMES, mode_code)}) "
                f"yaw={yaw} yr={yaw_ref} ba={msg.get('ba', None)} ma={msg.get('ma', 0)} alt={alt} "
                f"lat={lat} lon={lon} health={int(hl)}/{int(hg)}/{int(hh)} ls={landed}"
            )
            self.last_status_tuple = status_tuple
            self.last_status_log_time = now

    def on_plot_rssi(self):
        if self.rssi_plot_win is None or not getattr(self.rssi_plot_win, "_running", False):
            self.rssi_plot_win = RssiPlotWindow(self.root, self.t_buf, self.raw_buf, self.filt_buf)

    def on_plot_pattern(self):
        if self.pattern_plot_win is None or not getattr(self.pattern_plot_win, "_running", False):
            self.pattern_plot_win = PatternPlotWindow(self.root, self.get_pattern_results, self.get_expected_angles, self.get_final_beam)

    def save_logs_csv(self):
        if not self.log_records:
            messagebox.showinfo("Guardar logs CSV", "Todavía no hay registros RSSI/telemetría para guardar.")
            return
        timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        base = os.getcwd()
        log_path = os.path.join(base, f"fase4_8_log_u{self.session_id}_{timestamp}.csv")
        pattern_path = os.path.join(base, f"fase4_8_patron_u{self.session_id}_{timestamp}.csv")
        png_path = os.path.join(base, f"fase4_8_patron_u{self.session_id}_{timestamp}.png")
        try:
            # Completa metadatos conocidos al final de la misión también en filas antiguas.
            beam = self.final_beam
            candidates = beam.get('candidates', []) if beam is not None else []
            for row in self.log_records:
                if self.sweep_start_lat is not None:
                    row["lat_inicio_barrido_deg"] = self.sweep_start_lat
                    row["lon_inicio_barrido_deg"] = self.sweep_start_lon
                if self.aoa_sent_deg is not None:
                    row["aoa_enviado_deg"] = self.aoa_sent_deg
                for idx in range(1, MAX_ROBUST_PEAK_CANDIDATES + 1):
                    row[f"aoa_{idx}_deg"] = ""
                    row[f"aoa_{idx}_rssi_dbm"] = ""
                    row[f"aoa_{idx}_media_vecinal_dbm"] = ""
                for cand in candidates:
                    idx = int(cand['rank'])
                    row[f"aoa_{idx}_deg"] = round(cand['angle'], 3)
                    row[f"aoa_{idx}_rssi_dbm"] = round(cand['peak_dbm'], 3)
                    row[f"aoa_{idx}_media_vecinal_dbm"] = round(cand['robust_mean_dbm'], 3)

            fieldnames = list(self.log_records[0].keys())
            with open(log_path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.log_records)

            expected = self.get_expected_angles()
            beam = self.final_beam
            # Coordenadas medias por bin angular, calculadas solo con muestras válidas del patrón.
            geo_by_angle = defaultdict(list)
            for row in self.log_records:
                if int(row.get("pattern_valid_sample") or 0) != 1:
                    continue
                ang = safe_float(row.get("pattern_angle_bin_deg"))
                lat = safe_float(row.get("lat_deg"))
                lon = safe_float(row.get("lon_deg"))
                if ang is not None and lat is not None and lon is not None:
                    geo_by_angle[ang].append((lat, lon))

            sweep_mean_lat, sweep_mean_lon = self.get_sweep_geo_summary()
            with open(pattern_path, "w", newline="", encoding="utf-8-sig") as f:
                fields = [
                    "angulo_deg", "rssi_filtrado_prom_dbm", "num_muestras",
                    "lat_prom_deg", "lon_prom_deg"
                ]
                writer = csv.DictWriter(f, fieldnames=fields)
                writer.writeheader()
                with self.pattern_lock:
                    for ang in expected:
                        vals = self.pattern_samples.get(ang, [])
                        if vals:
                            geo = geo_by_angle.get(ang, [])
                            lat_prom = sum(v[0] for v in geo) / len(geo) if geo else ""
                            lon_prom = sum(v[1] for v in geo) / len(geo) if geo else ""
                            writer.writerow({
                                "angulo_deg": ang,
                                "rssi_filtrado_prom_dbm": round(sum(vals) / len(vals), 3),
                                "num_muestras": len(vals),
                                "lat_prom_deg": round(lat_prom, 7) if geo else "",
                                "lon_prom_deg": round(lon_prom, 7) if geo else "",
                            })
                    f.write("\n")
                    f.write(f"lat_inicio_mision_deg,{self.mission_start_lat if self.mission_start_lat is not None else ''}\n")
                    f.write(f"lon_inicio_mision_deg,{self.mission_start_lon if self.mission_start_lon is not None else ''}\n")
                    f.write(f"lat_inicio_barrido_deg,{self.sweep_start_lat if self.sweep_start_lat is not None else ''}\n")
                    f.write(f"lon_inicio_barrido_deg,{self.sweep_start_lon if self.sweep_start_lon is not None else ''}\n")
                    f.write(f"lat_media_barrido_deg,{round(sweep_mean_lat, 7) if sweep_mean_lat is not None else ''}\n")
                    f.write(f"lon_media_barrido_deg,{round(sweep_mean_lon, 7) if sweep_mean_lon is not None else ''}\n")
                    f.write(f"maximos_candidatos_cfg,{self.current_mission_cfg.get('peak_candidates', '')}\n")
                    f.write(f"vecinos_por_lado_cfg,{self.current_mission_cfg.get('peak_neighbors', '')}\n")
                    if beam is not None:
                        for cand in beam.get('candidates', []):
                            idx = int(cand['rank'])
                            f.write(f"AoA_{idx}_deg,{cand['angle']:.3f}\n")
                            f.write(f"AoA_{idx}_rssi_dbm,{cand['peak_dbm']:.3f}\n")
                            f.write(f"AoA_{idx}_media_vecinal_dbm,{cand['robust_mean_dbm']:.3f}\n")
                        f.write(f"AoA_deg,{beam['peak_angle']:.3f}\n")
                        f.write(f"AoA_enviado_deg,{self.aoa_sent_deg if self.aoa_sent_deg is not None else ''}\n")
                        f.write(f"AoA_rssi_dbm,{beam['peak_dbm']:.3f}\n")
                        f.write(f"AoA_media_vecinal_dbm,{beam['robust_mean_dbm']:.3f}\n")
                        f.write(f"beamwidth_deg,{beam['beamwidth_deg']:.3f}\n")
                        f.write(f"left_3dB_deg,{beam['left_angle']:.3f}\n")
                        f.write(f"right_3dB_deg,{beam['right_angle']:.3f}\n")
            if self.pattern_plot_win is not None and getattr(self.pattern_plot_win, "_running", False):
                self.pattern_plot_win.save_png(png_path)
        except Exception as e:
            messagebox.showerror("Guardar logs CSV", f"No se pudieron guardar los archivos:\n{e}")
            return
        self.log(f"[INFO] CSV principal guardado: {log_path}")
        self.log(f"[INFO] CSV patrón guardado: {pattern_path}")
        if os.path.exists(png_path):
            self.log(f"[INFO] PNG patrón guardado: {png_path}")
        messagebox.showinfo("Guardar logs CSV", f"Archivos guardados en:\n{log_path}\n{pattern_path}")

    def on_close(self):
        self.disconnect_all()
        self.root.destroy()


def main():
    root = tk.Tk()
    GroundStationGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
