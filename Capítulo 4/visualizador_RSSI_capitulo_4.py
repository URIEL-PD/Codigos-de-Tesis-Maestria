
import os
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox
from queue import Queue, Empty

import serial

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


PORT = "COM8"
BAUD = 57600

PC_MAC = 0xF021
REMOTE_RADIO_MAC = 0x3022 #Radio al que se hace la consulta del RSSI

RSSI_IM = 0x73
REGISTER_EVERY_S = 2.0
RSSI_QUERY_EVERY_S = 0.1
RSSI_ALPHA = 0.25 #Valor de alpha para el filtro (entre 0 y 1)

DEFAULT_SAVE_FMT = "SVG"
PLOT_REFRESH_MS = 100 #Tiempo de muestreo cada 100ms
PLOT_WINDOW_S = 30.0 #Muestra 30 muestras del RSSI en la gráfica
DEFAULT_DBM_MAX = -30.0#Límite máximo
DEFAULT_DBM_MIN = -60.0#Mínimo


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
    I = payload[4]
    N = payload[5]
    R = payload[6]
    M = payload[7:-2]
    c_recv = (payload[-1] << 8) | payload[-2]
    c_calc = crc16_ibm(internal[:-2], init=0xFFFF)
    crc_ok = (c_calc == c_recv)
    return crc_ok, do, dd, I, N, R, M


def build_rssi_request_frame(pc_mac: int, mrapc_mac: int, remote_radio_mac: int, seqn: int) -> bytes:
    internal_wo_crc = bytes([
        0x07,
        seqn & 0xFF,
        RSSI_IM,
        (remote_radio_mac >> 8) & 0xFF,
        remote_radio_mac & 0xFF
    ])
    c = crc16_ibm(internal_wo_crc, init=0xFFFF)
    crc_bytes = bytes([c & 0xFF, (c >> 8) & 0xFF])
    inner_pkt = bytes([0xFE]) + internal_wo_crc + crc_bytes + bytes([0xEF])

    M = bytes([
        (PC_MAC >> 8) & 0xFF, PC_MAC & 0xFF,
        (mrapc_mac >> 8) & 0xFF, mrapc_mac & 0xFF
    ]) + inner_pkt

    return build_frame(do=pc_mac, dd=mrapc_mac, I=0x04, N=(seqn & 0xFF), R=0x00, M=M)


def try_parse_data_packet(m: bytes):
    if len(m) < 9:
        return None
    if m[4] != 0xFE:
        return None
    Lm = m[5]
    total_len = 2 + 2 + 1 + Lm + 1
    if len(m) < total_len or m[total_len - 1] != 0xEF:
        return None
    internal = m[5:5 + Lm]
    body = internal[1:]
    if len(body) < 4:
        return None
    nm = body[0]
    im = body[1]
    mm = body[2:-2]
    c_recv = (body[-1] << 8) | body[-2]
    c_calc = crc16_ibm(internal[:-2], init=0xFFFF)
    if c_calc != c_recv:
        return None
    return nm, im, mm


class RSSINode:
    def __init__(self, port, baud, gui_queue: Queue):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.gui_q = gui_queue
        self.rx = b""
        self.running = True
        self.tx_lock = threading.Lock()
        self.mrapc_mac = None
        self.ready = False
        self._seqn = 0
        self.rssi_filt = None

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
        self._write(build_frame(do=PC_MAC, dd=self.mrapc_mac, I=0x02, M=bytes([0x02])))

    def send_ping_response(self):
        if self.mrapc_mac is None:
            return
        self._write(build_frame(do=PC_MAC, dd=self.mrapc_mac, I=0xFF, M=bytes([0x02])))

    def request_rssi(self):
        if self.mrapc_mac is None:
            return
        self._seqn = (self._seqn + 1) & 0xFF
        self._write(build_rssi_request_frame(PC_MAC, self.mrapc_mac, REMOTE_RADIO_MAC, self._seqn))

    def _update_rssi_from_mm(self, mm: bytes):
        if len(mm) < 5:
            return
        raw = mm[4]
        dbm = int(raw) - 128
        if self.rssi_filt is None:
            self.rssi_filt = float(dbm)
        else:
            self.rssi_filt = (1.0 - RSSI_ALPHA) * self.rssi_filt + RSSI_ALPHA * float(dbm)
        self.gui_q.put(("rssi", raw, dbm, self.rssi_filt))

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
                    continue
                if I == 0xFE:
                    if not self.ready:
                        self.ready = True
                        self.gui_q.put(("ready", True))
                    continue
                if I == 0x01:
                    self.send_ping_response()
                    continue
                if I == 0x04:
                    parsed = try_parse_data_packet(M)
                    if parsed is None:
                        continue
                    _, im, mm = parsed
                    if im == RSSI_IM:
                        self._update_rssi_from_mm(mm)

            now = time.time()
            if (self.mrapc_mac is not None) and ((now - last_reg) > REGISTER_EVERY_S):
                self.send_register()
                last_reg = now
            time.sleep(0.01)


class RSSIThesisApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Visualizador RSSI para tesis")
        self.q = Queue()
        self.node = RSSINode(PORT, BAUD, self.q)
        self.worker = threading.Thread(target=self.node.loop, daemon=True)
        self.worker.start()

        self.plot_enabled = False
        self.paused = False

        self.samples = []
        self.start_wall = None
        self.zero_ref = 0.0

        self.show_raw_var = tk.BooleanVar(value=True)
        self.show_filt_var = tk.BooleanVar(value=True)
        self.save_fmt_var = tk.StringVar(value=DEFAULT_SAVE_FMT)

        self._build_ui()
        self._build_plot()

        self.root.after(50, self.ui_poll)
        self.root.after(int(RSSI_QUERY_EVERY_S * 1000), self.rssi_tick)
        self.root.after(PLOT_REFRESH_MS, self.refresh_plot)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        top = tk.Frame(self.root)
        top.pack(fill="x", padx=8, pady=8)

        tk.Button(top, text="Graficar", command=self.on_graficar).pack(side="left")
        tk.Button(top, text="Reiniciar cero", command=self.on_reset_zero).pack(side="left", padx=(8, 0))
        self.btn_pause = tk.Button(top, text="Pausar", command=self.on_pause_resume)
        self.btn_pause.pack(side="left", padx=(8, 0))
        tk.Button(top, text="Guardar", command=self.on_save).pack(side="left", padx=(8, 0))

        tk.Label(top, text="Formato:").pack(side="left", padx=(12, 4))
        ttk.Combobox(top, width=7, state="readonly",
                     values=["SVG", "PNG", "CSV"], textvariable=self.save_fmt_var).pack(side="left")

        tk.Label(top, text="RSSI máx:").pack(side="left", padx=(12, 4))
        self.entry_dbm_max = tk.Entry(top, width=6)
        self.entry_dbm_max.pack(side="left")
        self.entry_dbm_max.insert(0, f"{DEFAULT_DBM_MAX:.0f}")

        tk.Label(top, text="RSSI mín:").pack(side="left", padx=(8, 4))
        self.entry_dbm_min = tk.Entry(top, width=6)
        self.entry_dbm_min.pack(side="left")
        self.entry_dbm_min.insert(0, f"{DEFAULT_DBM_MIN:.0f}")

        opts = tk.Frame(self.root)
        opts.pack(fill="x", padx=8, pady=(0, 6))
        tk.Checkbutton(opts, text="Mostrar RSSI", variable=self.show_raw_var,
                       command=self.refresh_plot_now).pack(side="left")
        tk.Checkbutton(opts, text="Mostrar RSSI filtrado", variable=self.show_filt_var,
                       command=self.refresh_plot_now).pack(side="left", padx=(12, 0))

        info = tk.Frame(self.root)
        info.pack(fill="x", padx=8, pady=(0, 6))
        self.lbl_status = tk.Label(info, text="Iniciando enlace...")
        self.lbl_status.pack(side="left")
        self.lbl_rssi = tk.Label(info, text="RSSI: (sin dato)")
        self.lbl_rssi.pack(side="right")

    def _read_manual_ylim(self):
        try:
            ymax = float(self.entry_dbm_max.get().strip())
            ymin = float(self.entry_dbm_min.get().strip())
        except Exception:
            raise ValueError("Los límites de RSSI deben ser numéricos.")
        if ymax <= ymin:
            raise ValueError("El RSSI máximo debe ser mayor que el RSSI mínimo.")
        return ymin, ymax

    def _build_plot(self):
        self.fig = Figure(figsize=(8.8, 4.8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("RSSI (dBm) vs Tiempo")
        self.ax.set_xlabel("Tiempo (s)")
        self.ax.set_ylabel("RSSI (dBm)")
        self.ax.grid(True)

        (self.line_raw,) = self.ax.plot([], [], label="RSSI (dBm)")
        (self.line_filt,) = self.ax.plot([], [], color="red", label="RSSI filtrado (dBm)")
        self.ax.legend(loc="upper left")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)

    def on_graficar(self):
        try:
            self._read_manual_ylim()
        except ValueError as e:
            messagebox.showerror("Error", str(e))
            return
        self.plot_enabled = True
        self.refresh_plot_now()

    def on_reset_zero(self):
        if self.samples:
            self.zero_ref = self.samples[-1]["logical_t"]
        else:
            self.zero_ref = 0.0
        self.refresh_plot_now()
        self.lbl_status.config(text="Cero de tiempo reiniciado.")

    def on_pause_resume(self):
        self.paused = not self.paused
        self.btn_pause.config(text=("Reanudar" if self.paused else "Pausar"))
        self.lbl_status.config(text=("Gráfica pausada." if self.paused else "Gráfica reanudada."))

    def on_save(self):
        if not self.samples:
            messagebox.showwarning("Sin datos", "Aún no hay datos para guardar.")
            return
        base_dir = os.path.abspath(os.path.dirname(__file__))
        ts = time.strftime("%Y%m%d_%H%M%S")
        fmt = self.save_fmt_var.get().strip().upper()
        try:
            if fmt == "CSV":
                out_path = os.path.join(base_dir, f"rssi_tesis_{ts}.csv")
                import csv
                with open(out_path, "w", newline="", encoding="utf-8") as f:
                    w = csv.writer(f)
                    w.writerow(["wall_t_s", "logical_t_s", "time_display_s", "rssi_dbm", "rssi_filtrado_dbm"])
                    for s in self.samples:
                        w.writerow([
                            f"{s['wall_t']:.6f}",
                            f"{s['logical_t']:.6f}",
                            f"{(s['logical_t'] - self.zero_ref):.6f}",
                            s["raw"],
                            f"{s['filt']:.6f}",
                        ])
            elif fmt == "SVG":
                out_path = os.path.join(base_dir, f"rssi_tesis_{ts}.svg")
                self.fig.savefig(out_path, dpi=250, facecolor="white", edgecolor="none",
                                 bbox_inches="tight", pad_inches=0.08, format="svg")
            else:
                out_path = os.path.join(base_dir, f"rssi_tesis_{ts}.png")
                self.fig.savefig(out_path, dpi=250, facecolor="white", edgecolor="none",
                                 bbox_inches="tight", pad_inches=0.08, format="png")
            self.lbl_status.config(text=f"Guardado: {os.path.basename(out_path)}")
            messagebox.showinfo("Guardado", f"Archivo guardado en:\n{out_path}")
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo guardar.\n{e}")

    def rssi_tick(self):
        if (not self.paused) and (self.node.mrapc_mac is not None):
            self.node.request_rssi()
        self.root.after(int(RSSI_QUERY_EVERY_S * 1000), self.rssi_tick)

    def ui_poll(self):
        try:
            while True:
                ev = self.q.get_nowait()
                if ev[0] == "mrapc":
                    self.lbl_status.config(text=f"Detectado MRAPC=0x{ev[1]:04X}")
                elif ev[0] == "ready":
                    self.lbl_status.config(text="Enlace listo.")
                elif ev[0] == "rssi":
                    raw, dbm, filt = ev[1], ev[2], ev[3]
                    self.lbl_rssi.config(text=f"RSSI: {dbm} dBm (raw {raw}) (filt {filt:.1f})")
                    if not self.paused:
                        now = time.time()
                        if self.start_wall is None:
                            self.start_wall = now
                            self.zero_ref = 0.0
                        logical_t = now - self.start_wall
                        self.samples.append({
                            "wall_t": now,
                            "logical_t": logical_t,
                            "raw": dbm,
                            "filt": float(filt),
                        })
        except Empty:
            pass
        self.root.after(50, self.ui_poll)

    def refresh_plot_now(self):
        self._update_plot()
        self.canvas.draw_idle()

    def refresh_plot(self):
        if self.plot_enabled:
            self._update_plot()
            self.canvas.draw_idle()
        self.root.after(PLOT_REFRESH_MS, self.refresh_plot)

    def _update_plot(self):
        t = [s["logical_t"] - self.zero_ref for s in self.samples]
        y_raw = [s["raw"] for s in self.samples]
        y_filt = [s["filt"] for s in self.samples]

        if self.show_raw_var.get():
            self.line_raw.set_data(t, y_raw)
            self.line_raw.set_visible(True)
        else:
            self.line_raw.set_visible(False)

        if self.show_filt_var.get():
            self.line_filt.set_data(t, y_filt)
            self.line_filt.set_visible(True)
        else:
            self.line_filt.set_visible(False)

        if len(t) >= 1:
            xmax = max(t)
            xmin = max(0.0, xmax - PLOT_WINDOW_S)
            if abs(xmax - xmin) < 1e-9:
                xmax = xmin + 1.0
            self.ax.set_xlim(xmin, xmax)

            try:
                ymin, ymax = self._read_manual_ylim()
                self.ax.set_ylim(ymin, ymax)
            except ValueError:
                self.ax.set_ylim(DEFAULT_DBM_MIN, DEFAULT_DBM_MAX)

        handles = []
        labels = []
        if self.show_raw_var.get():
            handles.append(self.line_raw)
            labels.append("RSSI (dBm)")
        if self.show_filt_var.get():
            handles.append(self.line_filt)
            labels.append("RSSI filtrado (dBm)")
        if handles:
            self.ax.legend(handles, labels, loc="upper left")

    def on_close(self):
        self.node.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    root.geometry("980x700")
    RSSIThesisApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
