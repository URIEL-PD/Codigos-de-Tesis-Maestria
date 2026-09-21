import time
import threading
from queue import Queue, Empty

import serial
from gpiozero import DigitalOutputDevice
from bluedot.btcomm import BluetoothServer


# ===================== CONFIG (Raspberry antena) =====================
PORT = "/dev/ttyUSB0"
BAUD = 57600

# PCs (0xFxxx)
PC_MAC   = 0xF022      # esta Raspberry (antena)
DEST_MAC = 0xF021      # Windows

# Radios (0x30xx)
LOCAL_RADIO_MAC = 0x3022   # radio conectado por USB a Raspberry

# IMs
CHAT_IM = 0x20
ACK_IM  = 0x21
TELE_IM = 0x30             # telemetría "az,flag" (sin ACK)

REGISTER_EVERY_S = 2.0
TELEMETRY_EVERY_S = 0.10   # 100 ms

# Motor
STEP_PIN = 23
DIR_PIN  = 24
PULSOS_POR_VUELTA = 6400
DELAY = 0.01  # s

# ----------------- Control iterativo (tunable) -----------------
TOL_DEG = 1.0              # termina si |error| <= esto
MAX_ITERS = 20             # max iteraciones por objetivo
KP = 0.75                  # fracción del error por iteración
STEP_MIN = 1               # pasos mínimos por iteración
STEP_MAX_DEG = 20.0        # máximo grados por iteración
SETTLE_S = 0.25            # espera tras mover

AZ_SAMPLES = 7             # muestras para estimar azimuth estable
AZ_SAMPLE_DT = 0.10        # tu app manda cada 100 ms
# ---------------------------------------------------------------
# =====================================================================


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
    crc_bytes = bytes([c & 0xFF, (c >> 8) & 0xFF])  # LSB primero

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


def build_data_packet(df_u16, ds_u16, im, mm: bytes, nm=0x00):
    df = bytes([(df_u16 >> 8) & 0xFF, df_u16 & 0xFF])
    ds = bytes([(ds_u16 >> 8) & 0xFF, ds_u16 & 0xFF])
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


def wrap180(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


def robust_azimuth(get_az_func):
    vals = []
    for _ in range(AZ_SAMPLES):
        az = get_az_func()
        if az is not None:
            vals.append(float(az) % 360.0)
        time.sleep(AZ_SAMPLE_DT)

    if len(vals) < 3:
        return None

    vals.sort()
    return vals[len(vals) // 2]


class Stepper:
    def __init__(self, step_pin: int, dir_pin: int):
        self.step = DigitalOutputDevice(step_pin)
        self.direction = DigitalOutputDevice(dir_pin)

    def mover_grados(self, grados: float):
        dir_val = 1 if grados >= 0 else 0
        self.direction.value = dir_val

        pasos = int(abs(grados) * (PULSOS_POR_VUELTA / 360.0))
        print(f"(motor) Moviendo {grados:.2f} deg -> {pasos} pasos")

        for _ in range(pasos):
            self.step.on()
            time.sleep(DELAY)
            self.step.off()
            time.sleep(DELAY)


class AntenaController:
    def __init__(self):
        self.stepper = Stepper(STEP_PIN, DIR_PIN)

        self.az_lock = threading.Lock()
        self.last_azimuth = None

        self.move_lock = threading.Lock()
        self.moving = False

        self.target_lock = threading.Lock()
        self.target_event = threading.Event()
        self.latest_target = None  # 0..360

        self.running = True

    def set_azimuth(self, az: float):
        with self.az_lock:
            self.last_azimuth = az

    def get_azimuth(self):
        with self.az_lock:
            return self.last_azimuth

    def set_target(self, target_deg: float):
        t = float(target_deg) % 360.0
        with self.target_lock:
            self.latest_target = t
        self.target_event.set()
        print(f"(cmd) Nuevo objetivo absoluto: {t:.2f} deg")

    def is_moving(self) -> int:
        with self.move_lock:
            return 1 if self.moving else 0

    def _set_moving(self, v: bool):
        with self.move_lock:
            self.moving = v

    def goto_target_iterative(self, target_deg: float) -> bool:
        step_deg = 360.0 / float(PULSOS_POR_VUELTA)
        min_deg = STEP_MIN * step_deg

        for k in range(1, MAX_ITERS + 1):
            if self.target_event.is_set():
                return False  # llegó un nuevo objetivo, aborta este

            az = robust_azimuth(self.get_azimuth)
            if az is None:
                print("(goto) No hay azimuth estable, abortando")
                return False

            # Convención que ya te funcionó en home:
            # err = (az - target) y luego camino corto (-180..180)
            err = wrap180(az - target_deg)
            print(f"(goto) iter {k}: az={az:.2f} target={target_deg:.2f} err={err:+.2f}")

            if abs(err) <= TOL_DEG:
                print(f"(goto) Listo: |err| <= {TOL_DEG} deg")
                return True

            corr = KP * err
            corr = max(-STEP_MAX_DEG, min(STEP_MAX_DEG, corr))

            if abs(corr) < min_deg:
                corr = min_deg if corr >= 0 else -min_deg

            self.stepper.mover_grados(corr)
            time.sleep(SETTLE_S)

        print("(goto) No convergió dentro del máximo de iteraciones")
        return False

    def motor_worker(self):
        while self.running:
            self.target_event.wait(timeout=0.2)
            if not self.running:
                break
            if not self.target_event.is_set():
                continue

            self.target_event.clear()
            with self.target_lock:
                tgt = self.latest_target

            if tgt is None:
                continue

            if self.get_azimuth() is None:
                print("(goto) Aún no tengo azimuth por Bluetooth, no puedo posicionar")
                continue

            self._set_moving(True)
            try:
                print("(goto) Posicionamiento iniciado")
                done = self.goto_target_iterative(tgt)
                if done:
                    print("(goto) Posicionamiento terminado")
            finally:
                self._set_moving(False)


class RadioNode:
    def __init__(self, port: str, baud: int, controller: AntenaController):
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.rx = b""

        self.controller = controller

        self.mrapc_mac = None
        self.ready = False

        self.running = True
        self.tx_lock = threading.Lock()

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

    def send_telemetry(self, az: float | None, moving_flag: int):
        if not self.ready:
            return
        if az is None:
            payload = f"nan,{moving_flag}".encode("ascii", errors="replace")
        else:
            payload = f"{az:.2f},{moving_flag}".encode("ascii", errors="replace")
        self._send_data_im(TELE_IM, payload)

    def telemetry_loop(self):
        while self.running:
            az = self.controller.get_azimuth()
            mv = self.controller.is_moving()
            self.send_telemetry(az, mv)
            time.sleep(TELEMETRY_EVERY_S)

    def loop(self):
        print(f"(radio) Abierto {PORT} @ {BAUD}")
        print("(radio) Esperando detectar MRAPC y completar registro")

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
                    print(f"(radio) MRAPC detectado: 0x{self.mrapc_mac:04X}")

                if I == 0xFE:
                    if not self.ready:
                        self.ready = True
                        print(f"(radio) Enlace listo (MRAPC=0x{self.mrapc_mac:04X})")

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
                        text = mm[2:].decode("utf-8", errors="replace").strip()
                        print(f"(radio) RX de 0x{df:04X}: {text}")

                        self.send_ack(mid)
                        self._handle_command(text)

            now = time.time()
            if (not self.ready) and (self.mrapc_mac is not None) and ((now - last_reg) > REGISTER_EVERY_S):
                self.send_register()
                last_reg = now

            time.sleep(0.01)

    def _handle_command(self, text: str):
        if not text:
            return

        # Ahora solo aceptamos objetivo absoluto 0..360 (también acepta negativos pero normaliza)
        try:
            tgt = float(text)
        except ValueError:
            print(f"(cmd) Ignorado (no es número): {text}")
            return

        self.controller.set_target(tgt)


def main():
    controller = AntenaController()

    t_motor = threading.Thread(target=controller.motor_worker, daemon=True)
    t_motor.start()

    def bt_rx(data: str):
        s = (data or "").strip()
        try:
            az = float(s) % 360.0
            controller.set_azimuth(az)
        except Exception:
            pass

    print("(bt) Iniciando servidor Bluetooth (esperando conexión del celular)")
    bt = BluetoothServer(bt_rx)

    node = RadioNode(PORT, BAUD, controller)

    t_tel = threading.Thread(target=node.telemetry_loop, daemon=True)
    t_tel.start()

    try:
        node.loop()
    except KeyboardInterrupt:
        print("\n(salir) Ctrl+C")
    finally:
        controller.running = False
        node.close()
        try:
            bt.stop()
        except Exception:
            pass
        time.sleep(0.2)


if __name__ == "__main__":
    main()
