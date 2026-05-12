import serial
import time
import threading
import tkinter as tk
from tkinter.scrolledtext import ScrolledText
from queue import Queue, Empty

# ================== CONFIG (Windows) ==================
PORT = "COM8" #o bien /dev/ttyUSB0 si se va a utilizar en Raspberry
BAUD = 57600

PC_MAC   = 0xF021          # esta PC (Windows)
DEST_MAC = 0xF022          # PC remota (Raspberry)

LOCAL_RADIO_MAC = 0x3021   # radio conectado por USB a esta PC

CHAT_IM = 0x20
ACK_IM  = 0x21

RETRY_COUNT = 5 # reintentos
ACK_TIMEOUT_S = 0.35 # espera total por cada intento

REGISTER_EVERY_S = 2.0 # registrarse en la red
# =========================================================


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
                        text = mm[2:].decode("utf-8", errors="replace")
                        self.gui_q.put(("chat_in", df, text))
                        self.send_ack(mid)
                        continue

            now = time.time()
            if (self.mrapc_mac is not None) and ((now - last_reg) > REGISTER_EVERY_S):
                self.send_register()
                last_reg = now


class RadioChatApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Radio Chat (Tkinter)")

        self.q = Queue()
        self.node = ReliableChatNode(PORT, BAUD, self.q)

        top = tk.Frame(root)
        top.pack(fill="x", padx=8, pady=6)

        self.entry = tk.Entry(top)
        self.entry.pack(side="left", fill="x", expand=True)

        self.btn_send = tk.Button(top, text="Enviar", command=self.on_send, state="disabled")
        self.btn_send.pack(side="left", padx=(8, 0))

        self.lbl_status = tk.Label(root, text="Iniciando enlace...")
        self.lbl_status.pack(anchor="w", padx=8)

        self.log = ScrolledText(root, height=18)
        self.log.pack(fill="both", expand=True, padx=8, pady=(6, 8))
        self.log.configure(state="disabled")

        self.entry.bind("<Return>", lambda _e: self.on_send())

        self.worker = threading.Thread(target=self.node.loop, daemon=True)
        self.worker.start()

        self.root.after(50, self.ui_poll)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def append_log(self, line: str):
        self.log.configure(state="normal")
        self.log.insert("end", line + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def on_send(self):
        text = self.entry.get().strip()
        if not text:
            return
        self.entry.delete(0, "end")

        ok = self.node.send_chat_reliable(text)
        if ok:
            self.append_log(f"yo: {text}")
        else:
            self.append_log(f"yo: {text} (sin ACK)")

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
                    self.btn_send.config(state="normal")

                elif et == "chat_in":
                    df, text = ev[1], ev[2]
                    self.append_log(f"remoto 0x{df:04X}: {text}")

        except Empty:
            pass

        self.root.after(50, self.ui_poll)

    def on_close(self):
        self.node.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    app = RadioChatApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
