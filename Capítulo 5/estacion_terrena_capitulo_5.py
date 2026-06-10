import os
import time
import math
import threading
import tkinter as tk
from tkinter import ttk
from queue import Queue, Empty
from collections import deque
from datetime import datetime

import serial

# Matplotlib
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ================== CONFIG (WINDOWS) ==================
PORT = "COM8"
BAUD = 57600

# MACs de PCs (0xFxxx)
PC_MAC   = 0xF021          # esta PC (Windows)
DEST_MAC = 0xF022          # Raspberry (antena)

# Radios (0x30xx)
LOCAL_RADIO_MAC  = 0x3021  # radio conectado a Windows (informativo)
REMOTE_RADIO_MAC = 0x3022  # radio en Raspberry (para consulta RSSI)

# Protocolo de aplicación
CHAT_IM = 0x20
ACK_IM  = 0x21

RSSI_IM = 0x73     # consulta RSSI
TELE_IM = 0x30     # telemetría "az,flag" desde Raspberry cada 100 ms

# Retransmisiones chat
RETRY_COUNT = 5
ACK_TIMEOUT_S = 0.35

# Periodicidades
REGISTER_EVERY_S = 2.0
RSSI_QUERY_EVERY_S = 0.10  # 100 ms

# Filtro RSSI (EMA)
RSSI_ALPHA = 0.1

# Gráfica RSSI
PLOT_WINDOW_S = 30.0
PLOT_MAX_POINTS = int(PLOT_WINDOW_S / RSSI_QUERY_EVERY_S) + 10

# Barrido
SWEEP_STEP_DEG = 5
SWEEP_ANGLES = list(range(0, 360, SWEEP_STEP_DEG))  # 0..350
SWEEP_SETTLE_EXTRA_S = 2.0
SWEEP_SAMPLES_PER_ANGLE = 30
SWEEP_SAMPLE_DT_S = 0.10  # 100 ms entre muestras de RSSI filtrado
SWEEP_WAIT_STABLE_TIMEOUT_S = 5.0
SWEEP_TARGET_TOL_DEG = 2.0  # tolerancia para aceptar que llegó al ángulo (por azimuth reportado)

# Reintentos por ángulo (para "despabilar" la estación móvil si se atora)
SWEEP_MAX_RESENDS_PER_ANGLE = 50

# Si la telemetría no llega en este tiempo, NO damos por válida la condición de estabilidad
TELE_STALE_S = 1.0

# Patrón polar (dBm negativos). Graficamos "magnitud desplazada":
DBM_MIN = -100.0
DBM_MAX = -40.0

# Máximo robusto (mitigar picos)
ROBUST_PEAK_CANDIDATES = 3      # número de máximos locales candidatos a comparar
ROBUST_PEAK_NEIGHBORS = 3       # vecinos a izquierda y derecha para promediar (ventana = 2*N+1)
# =======================================================


def robust_peak_from_results(results: dict[int, float], expected_angles: list[int]):
    if not results or not expected_angles:
        return None

    angles = [a for a in expected_angles if a in results]
    if not angles:
        return None

    vals = [float(results[a]) for a in angles]
    n = len(angles)
    if n == 1:
        return angles[0], vals[0], vals[0]

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
    local_max_idx = local_max_idx[:min(ROBUST_PEAK_CANDIDATES, len(local_max_idx))]

    best = None
    nb = max(0, int(ROBUST_PEAK_NEIGHBORS))
    for i in local_max_idx:
        neigh = [vals[(i + k) % n] for k in range(-nb, nb + 1)]
        avg = sum(neigh) / len(neigh)
        candidate = (angles[i], vals[i], avg)
        if best is None:
            best = candidate
        else:
            if (candidate[2] > best[2]) or (candidate[2] == best[2] and candidate[1] > best[1]):
                best = candidate

    return best


def wrap180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def interp_crossing_angle(a1: float, y1: float, a2: float, y2: float, thr: float) -> float:
    da = wrap180(a2 - a1)
    if abs(y2 - y1) < 1e-12:
        return a1 % 360.0
    t = (thr - y1) / (y2 - y1)
    t = max(0.0, min(1.0, t))
    return (a1 + t * da) % 360.0


def compute_beamwidth_info(results: dict[int, float], expected_angles: list[int]):
    """
    Devuelve un dict con:
      peak_angle, peak_dbm, half_power_dbm,
      left_angle, right_angle, beamwidth_deg
    o None si no se puede calcular.
    """
    robust = robust_peak_from_results(results, expected_angles)
    if robust is None:
        return None

    peak_angle, peak_dbm, _ = robust
    thr = float(peak_dbm) - 3.0

    angles = [a for a in expected_angles if a in results]
    if len(angles) < 3:
        return None

    vals = [float(results[a]) for a in angles]
    n = len(angles)
    try:
        peak_idx = angles.index(peak_angle)
    except ValueError:
        return None

    # Buscar cruce por izquierda saliendo del lóbulo principal
    left_cross = None
    i = peak_idx
    for step in range(1, n + 1):
        j = (peak_idx - step) % n
        prev = (j + 1) % n  # más cercano al pico
        y_near = vals[prev]
        y_far = vals[j]
        if y_near >= thr and y_far < thr:
            left_cross = interp_crossing_angle(angles[prev], y_near, angles[j], y_far, thr)
            break

    # Buscar cruce por derecha saliendo del lóbulo principal
    right_cross = None
    for step in range(1, n + 1):
        j = (peak_idx + step) % n
        prev = (j - 1) % n  # más cercano al pico
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
        "peak_angle": float(peak_angle) % 360.0,
        "peak_dbm": float(peak_dbm),
        "half_power_dbm": thr,
        "left_angle": float(left_cross) % 360.0,
        "right_angle": float(right_cross) % 360.0,
        "beamwidth_deg": float(beamwidth),
    }


# ---------------- CRC / Frame helpers ----------------
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


def bytes_to_u16(b: bytes) -> int:
    return (b[0] << 8) | b[1]


def build_frame(do, dd, I, N=0x00, R=0x00, M=b"") -> bytes:
    payload_wo_crc = bytes([
        (do >> 8) & 0xFF, do & 0xFF,
        (dd >> 8) & 0xFF, dd & 0xFF,
        I, N, R
    ]) + M

    L = 1 + len(payload_wo_crc) + 2
    internal_wo_crc = bytes([L]) + payload_wo_crc

    c = crc16_ibm(internal_wo_crc, init=0xFFFF)
    crc_bytes = bytes([c & 0xFF, (c >> 8) & 0xFF])

    return bytes([0xFE]) + internal_wo_crc + crc_bytes + bytes([0xEF])


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


def decode_frame(frame: bytes):
    L = frame[1]
    internal = frame[1:1 + L]
    payload = internal[1:]

    do = (payload[0] << 8) | payload[1]
    dd = (payload[2] << 8) | payload[3]
    I  = payload[4]
    N  = payload[5]
    R  = payload[6]
    M  = payload[7:-2]
    C_recv = (payload[-1] << 8) | payload[-2]

    C_calc = crc16_ibm(internal[:-2], init=0xFFFF)
    crc_ok = (C_calc == C_recv)

    return crc_ok, do, dd, I, N, R, M


def build_data_packet(df_u16, ds_u16, im, mm: bytes, nm=0x00) -> bytes:
    df = u16_to_bytes(df_u16)
    ds = u16_to_bytes(ds_u16)
    it = bytes([0xFE])

    body_wo_crc = bytes([nm, im]) + mm

    Lm = 1 + len(body_wo_crc) + 2
    internal_wo_crc = bytes([Lm]) + body_wo_crc

    c = crc16_ibm(internal_wo_crc, init=0xFFFF)
    crc_bytes = bytes([c & 0xFF, (c >> 8) & 0xFF])

    return df + ds + it + internal_wo_crc + crc_bytes + bytes([0xEF])


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


def build_rssi_request_frame(pc_mac: int, mrapc_mac: int, remote_radio_mac: int, seqn: int) -> bytes:
    internal_wo_crc = bytes([0x07, seqn & 0xFF, RSSI_IM,
                             (remote_radio_mac >> 8) & 0xFF, remote_radio_mac & 0xFF])
    c = crc16_ibm(internal_wo_crc, init=0xFFFF)
    crc_bytes = bytes([c & 0xFF, (c >> 8) & 0xFF])
    inner_pkt = bytes([0xFE]) + internal_wo_crc + crc_bytes + bytes([0xEF])

    M = bytes([(pc_mac >> 8) & 0xFF, pc_mac & 0xFF,
               (mrapc_mac >> 8) & 0xFF, mrapc_mac & 0xFF]) + inner_pkt

    return build_frame(do=pc_mac, dd=mrapc_mac, I=0x04, N=(seqn & 0xFF), R=0x00, M=M)


# ---------------- Node (radio) ----------------
class ReliableChatNode:
    def __init__(self, port, baud, gui_queue: Queue):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.rx = b""
        self.gui_q = gui_queue

        self.mrapc_mac = None
        self.ready = False

        self.running = True
        self.tx_lock = threading.Lock()

        self.msg_id = 1
        self.ack_lock = threading.Lock()
        self.acks_received = set()

        self.rssi_raw = None
        self.rssi_dbm = None
        self.rssi_filt = None

        self.azimuth = None
        self.moving_flag = 0

        self._seqn = 0

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass

    def _write(self, data: bytes):
        with self.tx_lock:
            self.ser.write(data)

    def send_register(self):
        if self.mrapc_mac is None:
            return
        reg = build_frame(do=PC_MAC, dd=self.mrapc_mac, I=0x02, M=bytes([0x02]))
        self._write(reg)

    def send_ping_response(self):
        if self.mrapc_mac is None:
            return
        resp = build_frame(do=PC_MAC, dd=self.mrapc_mac, I=0xFF, M=bytes([0x02]))
        self._write(resp)

    def _send_data_im(self, im: int, mm: bytes):
        if self.mrapc_mac is None:
            return
        pkt = build_data_packet(df_u16=PC_MAC, ds_u16=DEST_MAC, im=im, mm=mm, nm=0x00)
        fr = build_frame(do=PC_MAC, dd=self.mrapc_mac, I=0x04, M=pkt)
        self._write(fr)

    def send_ack(self, msg_id: int):
        mm = u16_to_bytes(msg_id)
        self._send_data_im(ACK_IM, mm)

    def send_chat_reliable(self, text: str) -> bool:
        if (self.mrapc_mac is None) or (not self.ready):
            return False

        text_bytes = text.encode("utf-8", errors="replace")
        mid = self.msg_id & 0xFFFF
        self.msg_id = (self.msg_id + 1) & 0xFFFF
        if self.msg_id == 0:
            self.msg_id = 1

        mm = u16_to_bytes(mid) + text_bytes

        for _ in range(RETRY_COUNT):
            self._send_data_im(CHAT_IM, mm)

            t0 = time.time()
            while time.time() - t0 < ACK_TIMEOUT_S:
                with self.ack_lock:
                    if mid in self.acks_received:
                        return True
                time.sleep(0.01)

        return False

    def request_rssi(self):
        if self.mrapc_mac is None:
            return
        self._seqn = (self._seqn + 1) & 0xFF
        fr = build_rssi_request_frame(PC_MAC, self.mrapc_mac, REMOTE_RADIO_MAC, self._seqn)
        self._write(fr)

    def _update_rssi_from_mm(self, mm: bytes):
        if len(mm) < 5:
            return
        raw = mm[4]
        dbm = int(raw) - 128

        self.rssi_raw = raw
        self.rssi_dbm = dbm

        if self.rssi_filt is None:
            self.rssi_filt = float(dbm)
        else:
            self.rssi_filt = (1.0 - RSSI_ALPHA) * self.rssi_filt + RSSI_ALPHA * float(dbm)

        self.gui_q.put(("rssi", self.rssi_raw, self.rssi_dbm, self.rssi_filt))

    def _update_telemetry_from_mm(self, mm: bytes):
        try:
            s = mm.decode("ascii", errors="replace").strip()
            parts = s.split(",")
            if len(parts) != 2:
                return
            az_s, fl_s = parts[0].strip(), parts[1].strip()

            if az_s.lower() == "nan":
                az = None
            else:
                az = float(az_s)

            fl = 1 if int(float(fl_s)) != 0 else 0

            self.azimuth = az
            self.moving_flag = fl
            self.gui_q.put(("tele", self.azimuth, self.moving_flag))
        except Exception:
            return

    def loop(self):
        last_reg = 0.0

        while self.running:
            self.rx += self.ser.read(4096)
            frames, self.rx = parse_frames_from_buffer(self.rx)

            for f in frames:
                crc_ok, do, dd, I, N, R, M = decode_frame(f)
                if not crc_ok:
                    continue

                if I == 0x03 and len(M) >= 3:
                    self.mrapc_mac = (M[1] << 8) | M[2]
                    self.gui_q.put(("mrapc", self.mrapc_mac))

                if I == 0xFE:
                    if not self.ready:
                        self.ready = True
                        self.gui_q.put(("ready", True))

                if I == 0x01:
                    self.send_ping_response()

                if I == 0x04:
                    parsed = try_parse_data_packet(M)
                    if parsed is None:
                        continue
                    df, ds, nm, im, mm = parsed

                    if im == ACK_IM and len(mm) >= 2:
                        mid = bytes_to_u16(mm[:2])
                        with self.ack_lock:
                            self.acks_received.add(mid)
                        continue

                    if im == CHAT_IM and len(mm) >= 2:
                        mid = bytes_to_u16(mm[:2])
                        self.send_ack(mid)
                        continue

                    if im == RSSI_IM:
                        self._update_rssi_from_mm(mm)
                        continue

                    if im == TELE_IM:
                        self._update_telemetry_from_mm(mm)
                        continue

            now = time.time()
            if (self.mrapc_mac is not None) and ((now - last_reg) > REGISTER_EVERY_S):
                self.send_register()
                last_reg = now


# ---------------- Plot windows ----------------
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

        (self.line_raw,) = self.ax.plot([], [], label="RSSI (dBm)")
        (self.line_filt,) = self.ax.plot([], [], color="red", label="RSSI filtrado (dBm)")

        self.leg = self.ax.legend(loc="upper left")
        try:
            self.leg.set_draggable(True)
        except Exception:
            pass

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

    def refresh(self):
        if not self._running:
            return

        t = list(self.t_buf)
        y1 = list(self.raw_buf)
        y2 = list(self.filt_buf)

        if len(t) >= 2:
            self.line_raw.set_data(t, y1)
            self.line_filt.set_data(t, y2)

            tmax = t[-1]
            tmin = max(0.0, tmax - PLOT_WINDOW_S)
            self.ax.set_xlim(tmin, tmax)
            self.ax.set_ylim(DBM_MIN, DBM_MAX)

        self.canvas.draw_idle()
        self.win.after(100, self.refresh)


class PatternPlotWindow:
    def __init__(self, parent: tk.Tk, get_results_callable):
        self.parent = parent
        self.get_results = get_results_callable

        self.win = tk.Toplevel(parent)
        self.win.title("Patrón de radiación (polar)")

        self.fig = Figure(figsize=(6.5, 6.0), dpi=100)
        self.ax = self.fig.add_subplot(111, projection="polar")
        self.ax.set_title("Patrón de radiación (RSSI en dBm)", va="bottom")

        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)
        self._configure_polar_axis()

        (self.line,) = self.ax.plot([], [], marker="o", linewidth=1.5)
        (self.max_pt,) = self.ax.plot([], [], marker="o", markersize=9, color="red", linestyle="None")
        self.max_annot = self.ax.annotate(
            "",
            xy=(0.0, 0.0),
            xytext=(10, 10),
            textcoords="offset points",
            color="red",
            fontsize=10,
            bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="red", alpha=0.7),
        )
        self.max_pt.set_visible(False)
        self.max_annot.set_visible(False)

        (self.ray_left,) = self.ax.plot([], [], color="black", linewidth=1.6)
        (self.ray_right,) = self.ax.plot([], [], color="black", linewidth=1.6)
        self.ray_left.set_visible(False)
        self.ray_right.set_visible(False)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.win)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        self._running = True
        self.win.protocol("WM_DELETE_WINDOW", self.on_close)

        self.refresh()

    def _configure_polar_axis(self):
        span = DBM_MAX - DBM_MIN
        if span <= 0:
            span = 1.0
        self.ax.set_rlim(0, span)

        step_db = 10
        start = int(math.floor(DBM_MIN / step_db) * step_db)
        stop = int(math.ceil(DBM_MAX / step_db) * step_db)
        ticks_dbm = list(range(start, stop + 1, step_db))
        ticks_r = [dbm - DBM_MIN for dbm in ticks_dbm]
        self.ax.set_yticks(ticks_r)
        self.ax.set_yticklabels([f"{dbm:d}" for dbm in ticks_dbm])
        self.ax.set_rlabel_position(22.5)

    def on_close(self):
        self._running = False
        try:
            self.win.destroy()
        except Exception:
            pass

    def clear(self):
        self.line.set_data([], [])
        self.max_pt.set_data([], [])
        self.max_pt.set_visible(False)
        self.max_annot.set_visible(False)
        self.max_annot.set_text("")
        self.ray_left.set_data([], [])
        self.ray_right.set_data([], [])
        self.ray_left.set_visible(False)
        self.ray_right.set_visible(False)
        self.canvas.draw_idle()

    def save_png(self, filepath: str):
        self.fig.savefig(filepath, dpi=200)

    def refresh(self):
        if not self._running:
            return

        results = self.get_results()
        if results:
            angles = sorted(results.keys())
            angles_closed = angles + [angles[0] + 360]
            dbm_vals = [results[a] for a in angles] + [results[angles[0]]]
            theta = [math.radians(a % 360) for a in angles_closed]

            r = []
            for v in dbm_vals:
                v2 = float(v)
                if v2 < DBM_MIN:
                    v2 = DBM_MIN
                if v2 > DBM_MAX:
                    v2 = DBM_MAX
                r.append(v2 - DBM_MIN)

            self.line.set_data(theta, r)
            self._configure_polar_axis()

            expected_n = len(SWEEP_ANGLES)
            if expected_n and len(results) >= expected_n:
                beam = compute_beamwidth_info(results, SWEEP_ANGLES)
                if beam is not None:
                    peak_angle = beam["peak_angle"]
                    peak_dbm = beam["peak_dbm"]
                    th_best = math.radians(peak_angle)
                    r_best = max(DBM_MIN, min(DBM_MAX, peak_dbm)) - DBM_MIN

                    self.max_pt.set_data([th_best], [r_best])
                    self.max_pt.set_visible(True)

                    left_angle = beam["left_angle"]
                    right_angle = beam["right_angle"]
                    thr = beam["half_power_dbm"]
                    bw = beam["beamwidth_deg"]
                    r_thr = max(DBM_MIN, min(DBM_MAX, thr)) - DBM_MIN

                    self.ray_left.set_data([math.radians(left_angle), math.radians(left_angle)], [0.0, r_thr])
                    self.ray_right.set_data([math.radians(right_angle), math.radians(right_angle)], [0.0, r_thr])
                    self.ray_left.set_visible(True)
                    self.ray_right.set_visible(True)

                    ang = peak_angle % 360.0
                    if 90.0 < ang < 270.0:
                        xytext = (-90, 10)
                        ha = "right"
                    else:
                        xytext = (10, 10)
                        ha = "left"

                    self.max_annot.xy = (th_best, r_best)
                    self.max_annot.set_text(f"({int(round(ang))}, {peak_dbm:.1f}, {bw:.1f})")
                    self.max_annot.set_ha(ha)
                    self.max_annot.set_position(xytext)
                    self.max_annot.set_visible(True)
                else:
                    self.max_pt.set_visible(False)
                    self.max_annot.set_visible(False)
                    self.ray_left.set_visible(False)
                    self.ray_right.set_visible(False)
            else:
                self.max_pt.set_visible(False)
                self.max_annot.set_visible(False)
                self.ray_left.set_visible(False)
                self.ray_right.set_visible(False)

        self.canvas.draw_idle()
        self.win.after(200, self.refresh)


class RadioChatApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Estación Terrena (Barrido + RSSI + Telemetría)")

        self.q = Queue()
        self.node = ReliableChatNode(PORT, BAUD, self.q)

        self.latest_rssi_filt = None
        self.latest_azimuth = None
        self.latest_moving = 0
        self.last_tele_time = None

        self.sweep_stop = threading.Event()
        self.sweep_running = False
        self.sweep_results = {}
        self.sweep_lock = threading.Lock()

        top = tk.Frame(root)
        top.pack(fill="x", padx=8, pady=6)

        self.entry = tk.Entry(top)
        self.entry.pack(side="left", fill="x", expand=True)

        self.btn_send = tk.Button(top, text="Enviar", command=self.on_send, state="disabled")
        self.btn_send.pack(side="left", padx=(8, 0))

        self.btn_home = tk.Button(top, text="Home", command=self.on_home, state="disabled")
        self.btn_home.pack(side="left", padx=(8, 0))

        row_controls = tk.Frame(root)
        row_controls.pack(fill="x", padx=8, pady=(0, 6))

        self.btn_start_sweep = tk.Button(row_controls, text="Comenzar barrido", command=self.on_start_sweep, state="disabled")
        self.btn_start_sweep.pack(side="left")

        self.btn_stop_sweep = tk.Button(row_controls, text="Detener barrido", command=self.on_stop_sweep, state="disabled")
        self.btn_stop_sweep.pack(side="left", padx=(8, 0))

        self.btn_plot_rssi = tk.Button(row_controls, text="Graficar RSSI", command=self.on_plot_rssi)
        self.btn_plot_rssi.pack(side="left", padx=(8, 0))

        self.btn_plot_pattern = tk.Button(row_controls, text="Graficar patrón", command=self.on_plot_pattern)
        self.btn_plot_pattern.pack(side="left", padx=(8, 0))

        self.btn_save_logs = tk.Button(row_controls, text="Guardar logs", command=self.on_save_logs, state="disabled")
        self.btn_save_logs.pack(side="right")

        row_prog = tk.Frame(root)
        row_prog.pack(fill="x", padx=8, pady=(0, 6))

        tk.Label(row_prog, text="Progreso barrido:").pack(side="left")
        self.prog = ttk.Progressbar(row_prog, orient="horizontal", length=300, mode="determinate", maximum=100.0)
        self.prog.pack(side="left", padx=(8, 0), fill="x", expand=True)
        self.lbl_prog = tk.Label(row_prog, text="0%")
        self.lbl_prog.pack(side="left", padx=(8, 0))

        row1 = tk.Frame(root)
        row1.pack(fill="x", padx=8, pady=(0, 4))

        self.lbl_status = tk.Label(row1, text="Iniciando enlace...")
        self.lbl_status.pack(side="left")

        self.lbl_rssi = tk.Label(row1, text="RSSI: (sin dato)")
        self.lbl_rssi.pack(side="right")

        row2 = tk.Frame(root)
        row2.pack(fill="x", padx=8, pady=(0, 8))

        self.lbl_az = tk.Label(row2, text="Azimuth: (sin dato)")
        self.lbl_az.pack(side="left")

        self.lbl_mov = tk.Label(row2, text="Antena: detenida")
        self.lbl_mov.pack(side="right")

        self.entry.bind("<Return>", lambda _e: self.on_send())

        self.worker = threading.Thread(target=self.node.loop, daemon=True)
        self.worker.start()

        self.t0 = time.time()
        self.t_buf = deque(maxlen=PLOT_MAX_POINTS)
        self.raw_buf = deque(maxlen=PLOT_MAX_POINTS)
        self.filt_buf = deque(maxlen=PLOT_MAX_POINTS)
        self.rssi_plot_win = None
        self.pattern_plot_win = None

        self.root.after(50, self.ui_poll)
        self.root.after(int(RSSI_QUERY_EVERY_S * 1000), self.rssi_tick)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def on_send(self):
        text = self.entry.get().strip()
        if not text:
            return
        self.entry.delete(0, "end")
        self._send_command_async(text)

    def on_home(self):
        self._send_command_async("0")

    def _send_command_async(self, text: str):
        self.btn_send.config(state="disabled")
        self.btn_home.config(state="disabled")
        self.lbl_status.config(text="Enviando...")
        threading.Thread(target=self._send_worker, args=(text,), daemon=True).start()

    def _send_worker(self, text: str):
        ok = self.node.send_chat_reliable(text)
        self.root.after(0, self._send_done, ok)

    def _send_done(self, ok: bool):
        if self.node.ready and (not self.sweep_running):
            self.btn_send.config(state="normal")
            self.btn_home.config(state="normal")

        if ok:
            self.lbl_status.config(text="(enlace listo) (enviado OK)")
        else:
            self.lbl_status.config(text="(enlace listo) (sin ACK, se envió pero no confirmó)")

    def rssi_tick(self):
        if self.node.mrapc_mac is not None:
            self.node.request_rssi()
        self.root.after(int(RSSI_QUERY_EVERY_S * 1000), self.rssi_tick)

    def ui_poll(self):
        try:
            while True:
                ev = self.q.get_nowait()
                et = ev[0]

                if et == "mrapc":
                    m = ev[1]
                    self.lbl_status.config(text=f"Detectado MRAPC=0x{m:04X}")

                elif et == "ready":
                    self.lbl_status.config(text="(enlace listo)")
                    if not self.sweep_running:
                        self.btn_send.config(state="normal")
                        self.btn_home.config(state="normal")
                    self.btn_start_sweep.config(state="normal")

                elif et == "rssi":
                    raw, dbm, filt = ev[1], ev[2], ev[3]
                    self.latest_rssi_filt = float(filt)
                    self.lbl_rssi.config(text=f"RSSI: {dbm} dBm (raw {raw}) (filt {filt:.1f})")
                    t = time.time() - self.t0
                    self.t_buf.append(t)
                    self.raw_buf.append(dbm)
                    self.filt_buf.append(filt)

                elif et == "tele":
                    az, mv = ev[1], ev[2]
                    self.latest_azimuth = az
                    self.latest_moving = mv
                    self.last_tele_time = time.time()

                    if az is None:
                        self.lbl_az.config(text="Azimuth: (sin dato)")
                    else:
                        self.lbl_az.config(text=f"Azimuth: {az:.2f}°")

                    self.lbl_mov.config(text=("Antena: en movimiento" if mv else "Antena: detenida"))

        except Empty:
            pass

        self.root.after(50, self.ui_poll)

    def on_plot_rssi(self):
        if self.rssi_plot_win is None or not getattr(self.rssi_plot_win, "_running", False):
            self.rssi_plot_win = RssiPlotWindow(self.root, self.t_buf, self.raw_buf, self.filt_buf)

    def _get_sweep_results_copy(self):
        with self.sweep_lock:
            return dict(self.sweep_results)

    def on_plot_pattern(self):
        if self.pattern_plot_win is None or not getattr(self.pattern_plot_win, "_running", False):
            self.pattern_plot_win = PatternPlotWindow(self.root, self._get_sweep_results_copy)

    def on_start_sweep(self):
        if self.sweep_running:
            return

        with self.sweep_lock:
            self.sweep_results = {}

        self._set_progress(0.0)
        self.btn_save_logs.config(state="disabled")

        if self.pattern_plot_win is not None and getattr(self.pattern_plot_win, "_running", False):
            self.pattern_plot_win.clear()

        self.sweep_stop.clear()
        self.sweep_running = True

        self.btn_start_sweep.config(state="disabled")
        self.btn_stop_sweep.config(state="normal")
        self.btn_send.config(state="disabled")
        self.btn_home.config(state="disabled")

        self.lbl_status.config(text="Barrido iniciado...")
        threading.Thread(target=self._sweep_worker, daemon=True).start()

    def on_stop_sweep(self):
        self.sweep_stop.set()
        self.lbl_status.config(text="Deteniendo barrido...")

        with self.sweep_lock:
            self.sweep_results = {}
        self._set_progress(0.0)

        if self.pattern_plot_win is not None and getattr(self.pattern_plot_win, "_running", False):
            self.pattern_plot_win.clear()

        self.btn_save_logs.config(state="disabled")

    def _set_progress(self, pct: float):
        pct = max(0.0, min(100.0, pct))
        self.prog["value"] = pct
        self.lbl_prog.config(text=f"{pct:.0f}%")

    def _sweep_worker(self):
        total = len(SWEEP_ANGLES)

        for idx, ang in enumerate(SWEEP_ANGLES):
            if self.sweep_stop.is_set():
                break

            stable = False
            for attempt in range(1, SWEEP_MAX_RESENDS_PER_ANGLE + 1):
                if self.sweep_stop.is_set():
                    break

                ok = self.node.send_chat_reliable(str(ang))
                self.root.after(
                    0,
                    self.lbl_status.config,
                    {"text": f"Barrido: {ang}° (intento {attempt}/{SWEEP_MAX_RESENDS_PER_ANGLE}) (ACK {'OK' if ok else 'no'})"}
                )

                if self._wait_until_stable(ang):
                    stable = True
                    break

                if self.sweep_stop.is_set():
                    break

                self.root.after(
                    0,
                    self.lbl_status.config,
                    {"text": f"No estable en {ang}° (reintentando {attempt}/{SWEEP_MAX_RESENDS_PER_ANGLE})"}
                )
                time.sleep(0.25)

            if self.sweep_stop.is_set():
                break

            if not stable:
                self.root.after(0, self.lbl_status.config, {"text": f"Error: no se estabilizó en {ang}° tras reintentos"})
                break

            t_end = time.time() + SWEEP_SETTLE_EXTRA_S
            while time.time() < t_end:
                if self.sweep_stop.is_set():
                    break
                time.sleep(0.05)

            if self.sweep_stop.is_set():
                break

            samples = []
            for _ in range(SWEEP_SAMPLES_PER_ANGLE):
                if self.sweep_stop.is_set():
                    break
                v = self.latest_rssi_filt
                if v is not None:
                    samples.append(float(v))
                time.sleep(SWEEP_SAMPLE_DT_S)

            if self.sweep_stop.is_set():
                break

            if not samples:
                self.root.after(0, self.lbl_status.config, {"text": f"Sin muestras RSSI en {ang}°"})
                break

            avg = sum(samples) / len(samples)
            with self.sweep_lock:
                self.sweep_results[int(ang)] = float(avg)

            pct = (idx + 1) * 100.0 / total
            self.root.after(0, self._set_progress, pct)
            self.root.after(0, self.lbl_status.config, {"text": f"Barrido: {ang}° OK (prom {avg:.1f} dBm)"})

        finished = (not self.sweep_stop.is_set()) and (len(self._get_sweep_results_copy()) == len(SWEEP_ANGLES))

        def _finish_ui():
            self.sweep_running = False
            self.btn_stop_sweep.config(state="disabled")

            if self.node.ready:
                self.btn_start_sweep.config(state="normal")
                self.btn_send.config(state="normal")
                self.btn_home.config(state="normal")
            else:
                self.btn_start_sweep.config(state="disabled")

            if finished:
                self._set_progress(100.0)
                self.lbl_status.config(text="Barrido terminado (100%)")
                self.btn_save_logs.config(state="normal")
            else:
                self.lbl_status.config(text="Barrido detenido" if self.sweep_stop.is_set() else "Barrido terminado con error")

        self.root.after(0, _finish_ui)

    def _telemetry_is_fresh(self) -> bool:
        if self.last_tele_time is None:
            return False
        return (time.time() - self.last_tele_time) <= TELE_STALE_S

    def _wait_until_stable(self, target_deg: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < SWEEP_WAIT_STABLE_TIMEOUT_S:
            if self.sweep_stop.is_set():
                return False
            if not self._telemetry_is_fresh():
                time.sleep(0.05)
                continue

            az = self.latest_azimuth
            mv = self.latest_moving
            if (az is not None) and (mv == 0):
                err = abs(wrap180(float(az) - float(target_deg)))
                if err <= SWEEP_TARGET_TOL_DEG:
                    return True
            time.sleep(0.05)
        return False

    def on_save_logs(self):
        results = self._get_sweep_results_copy()
        if len(results) != len(SWEEP_ANGLES):
            self.lbl_status.config(text="No se puede guardar (barrido incompleto)")
            return

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_dir = os.path.abspath(os.path.dirname(__file__))
        csv_path = os.path.join(base_dir, f"logs_barrido_{ts}.csv")
        png_path = os.path.join(base_dir, f"patron_{ts}.png")

        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("angulo_deg,rssi_prom_dbm\n")
            for ang in sorted(results.keys()):
                f.write(f"{ang},{results[ang]:.3f}\n")

        if self.pattern_plot_win is not None and getattr(self.pattern_plot_win, "_running", False):
            self.pattern_plot_win.save_png(png_path)
        else:
            fig = Figure(figsize=(6.5, 6.0), dpi=200)
            ax = fig.add_subplot(111, projection="polar")
            ax.set_theta_zero_location("N")
            ax.set_theta_direction(-1)
            ax.set_title("Patrón de radiación (RSSI en dBm)", va="bottom")

            span = DBM_MAX - DBM_MIN
            if span <= 0:
                span = 1.0
            ax.set_rlim(0, span)
            step_db = 10
            start = int(math.floor(DBM_MIN / step_db) * step_db)
            stop = int(math.ceil(DBM_MAX / step_db) * step_db)
            ticks_dbm = list(range(start, stop + 1, step_db))
            ticks_r = [dbm - DBM_MIN for dbm in ticks_dbm]
            ax.set_yticks(ticks_r)
            ax.set_yticklabels([f"{dbm:d}" for dbm in ticks_dbm])
            ax.set_rlabel_position(22.5)

            angles = sorted(results.keys())
            angles_closed = angles + [angles[0] + 360]
            dbm_vals = [results[a] for a in angles] + [results[angles[0]]]
            r = []
            for v in dbm_vals:
                v2 = float(v)
                if v2 < DBM_MIN:
                    v2 = DBM_MIN
                if v2 > DBM_MAX:
                    v2 = DBM_MAX
                r.append(v2 - DBM_MIN)

            theta = [math.radians(a % 360) for a in angles_closed]
            ax.plot(theta, r, marker="o", linewidth=1.5)
            ax.set_rlim(0, DBM_MAX - DBM_MIN)

            beam = compute_beamwidth_info(results, SWEEP_ANGLES)
            if beam is not None:
                peak_angle = beam["peak_angle"]
                peak_dbm = beam["peak_dbm"]
                left_angle = beam["left_angle"]
                right_angle = beam["right_angle"]
                thr = beam["half_power_dbm"]
                bw = beam["beamwidth_deg"]

                th_best = math.radians(peak_angle)
                r_best = max(DBM_MIN, min(DBM_MAX, peak_dbm)) - DBM_MIN
                ax.plot([th_best], [r_best], marker="o", markersize=9, color="red", linestyle="None")

                r_thr = max(DBM_MIN, min(DBM_MAX, thr)) - DBM_MIN
                ax.plot([math.radians(left_angle), math.radians(left_angle)], [0.0, r_thr], color="black", linewidth=1.6)
                ax.plot([math.radians(right_angle), math.radians(right_angle)], [0.0, r_thr], color="black", linewidth=1.6)

                ang = peak_angle % 360.0
                if 90.0 < ang < 270.0:
                    xytext = (-90, 10)
                    ha = "right"
                else:
                    xytext = (10, 10)
                    ha = "left"

                ax.annotate(
                    f"({int(round(ang))}, {peak_dbm:.1f}, {bw:.1f})",
                    xy=(th_best, r_best),
                    xytext=xytext,
                    textcoords="offset points",
                    color="red",
                    fontsize=10,
                    ha=ha,
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="red", alpha=0.7),
                )

            fig.savefig(png_path)

        self.lbl_status.config(text=f"Logs guardados: {os.path.basename(csv_path)} (y PNG)")

    def on_close(self):
        try:
            self.sweep_stop.set()
        except Exception:
            pass
        self.node.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    _app = RadioChatApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
