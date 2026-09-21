#!/usr/bin/env python3
"""
El comando compacto S ejecuta:
    1. Validación de condiciones PX4.
    2. Armado.
    3. Despegue a la altura indicada por la estación terrena.
    4. Espera breve de estabilización.
    5. Entrada a OFFBOARD manteniendo una posición local fija.
    6. Orientación inicial a norte geográfico/local (0 deg).
    7. Barrido de yaw continuo o discreto manteniendo posición y altura.
    7. Mantener OFFBOARD y esperar AoA calculado por la estación terrena.
    8. Orientarse suavemente al AoA absoluto respecto al norte.
    9. Salir de OFFBOARD y aterrizar.
    10. Cierre lógico solo cuando PX4 reporta EN TIERRA y DESARMADO.
"""

import asyncio
import json
import math
import queue
import serial
import threading
import time
import zlib
from typing import Optional

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw


# =========================
# Puertos
# =========================

# Radio SiK conectado por USB a la Raspberry.
RADIO_PORT = "/dev/ttyUSB0"
RADIO_BAUD = 57600

# Enlace MAVLink hacia PX4 por TELEM2.
PX4_SYSTEM_ADDRESS = "serial:///dev/ttyAMA0:57600"


# =========================
# Parámetros de enlace
# =========================

# Cada cuánto la Raspberry reporta estado compacto a la estación terrena.
STATUS_PERIOD_S = 0.20

# Timeout corto para reportar pérdida de enlace cuando el dron está en tierra.
# Durante misión se usa ACTIVE_LINK_TIMEOUT_S para evitar abortos por ráfagas breves
# de tramas inválidas o pérdidas momentáneas de heartbeat.
LINK_TIMEOUT_S = 5.0
ACTIVE_LINK_TIMEOUT_S = 20.0


# =========================
# Parámetros de vuelo y seguridad
# =========================

# Tolerancia para considerar que el despegue alcanzó la altura solicitada.
TAKEOFF_ALT_TOL_M = 0.5
TAKEOFF_TIMEOUT_S = 45.0
MIN_TAKEOFF_ALT_M = 1.0
MAX_TAKEOFF_ALT_M = 15.0

# Espera breve después de alcanzar altura, antes de iniciar OFFBOARD.
WAIT_AFTER_TAKEOFF_S = 2.0

# Antes de iniciar el barrido se orienta el dron a 0 grados (norte)
# y se mantiene un instante para estabilizar la referencia angular.
# Esta velocidad se separa de la velocidad del barrido continuo. Así puedes barrer
# lento, por ejemplo 5 deg/s, sin que la orientación inicial tarde 12 s o más.
NORTH_ALIGN_HOLD_S = 1.0
NORTH_ALIGN_YAW_RATE_DEG_S = 45.0

# Velocidad de orientación final hacia el AoA estimado. También se separa del barrido.
AOA_ORIENT_YAW_RATE_DEG_S = 45.0

# Después del barrido la Raspberry espera el AoA de la estación terrena mientras
# mantiene posición/altura en OFFBOARD. Si no llega, aterriza por seguridad.
AOA_WAIT_TIMEOUT_S = 20.0
AOA_FINAL_HOLD_S = 1.0

# Frecuencia de setpoints OFFBOARD. PX4 necesita recibir setpoints de forma continua.
OFFBOARD_SETPOINT_RATE_HZ = 20.0
OFFBOARD_PRESTREAM_S = 3.0
OFFBOARD_ATTEMPTS = 3

# Límites de parámetros de barrido.
MIN_YAW_RATE_DEG_S = 0.05
MAX_YAW_RATE_DEG_S = 45.0
MIN_STEP_DEG = 1.0
MAX_STEP_DEG = 180.0
MIN_HOLD_S = 0.1
MAX_HOLD_S = 120.0
MIN_ROTATIONS = 1
MAX_ROTATIONS = 3


# =========================
# Protocolo compacto
# =========================
#
# Formato físico de trama:
#     $JSON*CRC32\n
#
# Campos compactos generales:
#     t   tipo de mensaje
#     u   ID de sesión generado por la estación terrena
#     q   número de secuencia local del emisor
#     c   command id dentro de la sesión u
#     a   tipo de comando confirmado por ACK
#     ok  1 si el comando fue aceptado, 0 si fue rechazado
#     e   código breve de error o razón de rechazo
#
# Campos del comando S (comenzar barrido):
#     b   modo de barrido: "C" continuo, "D" discreto
#     h   altura objetivo en metros
#     w   velocidad angular de yaw en grados/segundo
#     n   número de vueltas completas de yaw, de 1 a 3
#     p   paso angular en grados, solo usado en modo discreto
#     d   tiempo de permanencia por ángulo en segundos, solo usado en modo discreto
#
# Campos de status/telemetría:
#     s   estado compacto del nodo Raspberry
#     r   ready, 1 si PX4 está conectado, 0 si no
#     lk  link_ok desde la perspectiva de la Raspberry, 1/0
#     lc  último comando/acción reportado por la Raspberry
#     ec  último command id aceptado/ejecutado dentro de la sesión u
#     x   contador de tramas inválidas recibidas en la Raspberry
#     px  1 si MAVSDK está conectado a PX4, 0 si no
#     ar  1 si PX4 reporta vehículo armado, 0 si no
#     m   modo de vuelo compacto
#     y   yaw/azimut estimado en grados [0, 360)
#     z   altura relativa en metros, respecto a home/despegue, null si no disponible
#     hl  health local_position_ok, 1/0
#     hg  health global_position_ok, 1/0
#     hh  health home_position_ok, 1/0
#     ls  landed_state compacto
#     pr  progreso de la rutina en porcentaje [0, 100]
#     pn  posición local NED norte [m], redondeada, útil para verificar deriva
#     pe  posición local NED este [m], redondeada, útil para verificar deriva
#     pd  posición local NED down [m], redondeada, útil para verificar altura local
#     la  latitud global [deg], si PX4 la entrega
#     lo  longitud global [deg], si PX4 la entrega
#     yr  yaw de referencia enviado por la Raspberry [deg]
#     ba  ángulo/bin de barrido asociado a la muestra [deg]
#     ma  1 si la muestra RSSI debe considerarse activa para patrón, 0 si está en transición
#     bm  modo de barrido activo: C continuo, D discreto, vacío si no aplica
#     er  último error compacto, si existe
#
# Tipos de mensaje (t):
#     H  heartbeat
#     S  comenzar_barrido
#     L  aterrizar, comando prioritario
#     Y  orientar al AoA estimado; campo g=ángulo absoluto [0,360)
#     R  reboot_request, deshabilitado; usar QGroundControl
#     M  deshabilitado; la prueba de armado se retiró
#     A  ack
#     T  status/telemetry
#
# Estados compactos (s):
#     I    espera
#     TK   despegando
#     HT   altura alcanzada
#     OF   entrando a offboard
#     N0   orientando y estabilizando en norte (0 deg)
#     BC   barrido continuo
#     BD   barrido discreto
#     QA   barrido terminado, esperando AoA de la estación terrena
#     YA   orientando al AoA estimado
#     LND  aterrizando
#     LL   enlace perdido
#     ER   error
#
# Confirmación doble:
#     1. ACK explícito: {"t":"A","u":58231,"a":"S","ok":1,"c":47}
#     2. STATUS implícito: {"t":"T","u":58231,...,"ec":47}
#
# El comando único se identifica por la pareja (u, c).
# =========================


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


def yaw_to_0_360(yaw_deg: Optional[float]) -> Optional[float]:
    if yaw_deg is None:
        return None
    try:
        return round(float(yaw_deg) % 360.0, 1)
    except Exception:
        return None


def clean_altitude(value) -> Optional[float]:
    """Convierte relative_altitude_m a un número compacto o None."""
    try:
        val = float(value)
        if math.isnan(val) or math.isinf(val):
            return None
        return round(val, 1)
    except Exception:
        return None


def enum_name(value) -> str:
    if value is None:
        return "UNKNOWN"
    if hasattr(value, "name"):
        return str(value.name)
    text = str(value)
    if "." in text:
        text = text.split(".")[-1]
    return text.upper()


def compact_flight_mode(mode) -> str:
    name = enum_name(mode)
    if "OFFBOARD" in name:
        return "O"
    if "POSCTL" in name or "POSITION" in name:
        return "P"
    if "STABILIZED" in name:
        return "S"
    if "ALTCTL" in name or "ALTITUDE" in name:
        return "A"
    if "MANUAL" in name:
        return "M"
    if "HOLD" in name:
        return "H"
    if "RETURN" in name or "RTL" in name:
        return "R"
    if "LAND" in name:
        return "L"
    if "TAKEOFF" in name:
        return "T"
    if "MISSION" in name:
        return "N"
    if "ACRO" in name:
        return "C"
    return "U"


def compact_landed_state(state) -> str:
    name = enum_name(state)
    if "ON_GROUND" in name:
        return "G"
    if "IN_AIR" in name:
        return "A"
    if "TAKING_OFF" in name:
        return "T"
    if "LANDING" in name:
        return "L"
    return "U"


def wrap_360(angle_deg: float) -> float:
    return float(angle_deg) % 360.0


def wrap_180(angle_deg: float) -> float:
    """Devuelve un ángulo equivalente en [-180, 180)."""
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def clean_local_value(value) -> Optional[float]:
    try:
        val = float(value)
        if math.isnan(val) or math.isinf(val):
            return None
        return round(val, 2)
    except Exception:
        return None


def clean_global_coord(value) -> Optional[float]:
    try:
        val = float(value)
        if math.isnan(val) or math.isinf(val):
            return None
        return round(val, 7)
    except Exception:
        return None


class DroneRadioPX4Node:
    def __init__(self, radio_port: str, radio_baud: int, px4_address: str):
        self.ser = serial.Serial(radio_port, radio_baud, timeout=0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(2)

        self.px4_address = px4_address
        self.drone = System()

        self.seq = 0
        self.running = True
        self.rx_queue = queue.Queue()
        self.rx_buffer = bytearray()
        self.tx_lock = threading.Lock()

        self.last_ground_msg_time = None
        self.link_ok = False
        self.rx_invalid_count = 0
        self.current_session_id = 0

        self.state = "I"
        self.ready = False
        self.last_cmd = "N"
        self.executed_cmd_id = 0
        self.last_error = ""
        self.progress_pct = 0.0

        self.ack_cache = {}
        self.mission_running = False
        self.mission_task = None
        self.land_requested = False
        self.landing_command_sent = False
        self.offboard_active = False

        self.telemetry_lock = threading.Lock()
        self.px4_connected = False
        self.armed = False
        self.flight_mode = "U"
        self.yaw_deg = None
        self.relative_alt_m = None
        self.local_north_m = None
        self.local_east_m = None
        self.local_down_m = None
        self.latitude_deg = None
        self.longitude_deg = None

        # Variables de sincronización para fase 4.0.
        # La estación terrena usa estas referencias para asociar RSSI con yaw.
        self.yaw_ref_deg = None
        self.sweep_angle_bin_deg = None
        self.measurement_active = False
        self.active_sweep_mode = ""

        # Fase 4.1: objetivo AoA recibido desde la estación terrena.
        self.aoa_target_deg = None
        self.aoa_target_cmd_id = None

        self.offboard_hold_north_m = None
        self.offboard_hold_east_m = None
        self.offboard_hold_down_m = None
        self.health_local = False
        self.health_global = False
        self.health_home = False
        self.landed_state = "U"

    def next_seq(self) -> int:
        self.seq += 1
        return self.seq

    def send_message(self, msg: dict, verbose: bool = True):
        msg = dict(msg)
        msg.setdefault("u", self.current_session_id)
        msg["q"] = self.next_seq()
        frame = make_frame(msg)
        with self.tx_lock:
            self.ser.write(frame)
            self.ser.flush()
        if verbose:
            print(f"[TX] {msg}")

    def start_new_session_if_needed(self, session_id):
        """Limpia la memoria lógica del protocolo cuando Windows abre una sesión nueva."""
        if session_id is None:
            return
        try:
            session_id = int(session_id)
        except Exception:
            return
        if session_id <= 0 or session_id == self.current_session_id:
            return

        old = self.current_session_id
        self.current_session_id = session_id
        self.ack_cache.clear()
        self.executed_cmd_id = 0
        self.last_error = ""

        # Solo se limpia la memoria lógica del protocolo. No se toca PX4.
        if not self.mission_running:
            self.state = "I"
            self.last_cmd = "N"
            self.ready = self.px4_connected
            self.progress_pct = 0.0

        print(f"[SESSION] Nueva sesión u={session_id} (antes u={old}). ACK cache limpiada.")

    def send_ack(self, ack_type: str, accepted: bool, reason: str = "", ack_cmd_id=None, session_id=None):
        if session_id is None:
            session_id = self.current_session_id
        ack = {
            "t": "A",
            "u": session_id,
            "a": ack_type,
            "ok": 1 if accepted else 0,
            "c": ack_cmd_id,
        }
        if reason:
            ack["e"] = reason
        if ack_cmd_id is not None and session_id is not None:
            self.ack_cache[(session_id, ack_cmd_id)] = ack
        self.send_message(ack)

    def resend_cached_ack(self, session_id: int, cmd_id: int) -> bool:
        ack = self.ack_cache.get((session_id, cmd_id))
        if ack is not None:
            print(f"[DUP] u={session_id} cmd_id={cmd_id}. Reenviando ACK sin ejecutar de nuevo.")
            self.send_message(ack)
            return True
        return False

    def set_progress(self, value: float):
        self.progress_pct = max(0.0, min(100.0, round(float(value), 1)))

    def receive_loop(self):
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                self.rx_buffer.extend(chunk)
                if len(self.rx_buffer) > 4096:
                    last_start = self.rx_buffer.rfind(b"$")
                    if last_start >= 0:
                        self.rx_buffer = self.rx_buffer[last_start:]
                    else:
                        self.rx_buffer.clear()
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
                print(f"[ERROR RX] {e}")
                self.running = False
                break

    def snapshot_telemetry(self) -> dict:
        with self.telemetry_lock:
            return {
                "px": 1 if self.px4_connected else 0,
                "ar": 1 if self.armed else 0,
                "m": self.flight_mode,
                "y": self.yaw_deg,
                "z": self.relative_alt_m,
                "pn": self.local_north_m,
                "pe": self.local_east_m,
                "pd": self.local_down_m,
                "la": self.latitude_deg,
                "lo": self.longitude_deg,
                "yr": self.yaw_ref_deg,
                "ba": self.sweep_angle_bin_deg,
                "ma": 1 if self.measurement_active else 0,
                "bm": self.active_sweep_mode,
                "hl": 1 if self.health_local else 0,
                "hg": 1 if self.health_global else 0,
                "hh": 1 if self.health_home else 0,
                "ls": self.landed_state,
            }

    def send_status(self):
        telem = self.snapshot_telemetry()
        msg = {
            "t": "T",
            "u": self.current_session_id,
            "s": self.state,
            "r": 1 if self.ready else 0,
            "lk": 1 if self.link_ok else 0,
            "lc": self.last_cmd,
            "ec": self.executed_cmd_id,
            "x": self.rx_invalid_count,
            "pr": self.progress_pct,
            **telem,
        }
        if self.last_error:
            msg["er"] = self.last_error
        self.send_message(msg, verbose=False)

    async def status_loop(self):
        while self.running:
            self.send_status()
            await asyncio.sleep(STATUS_PERIOD_S)

    async def process_rx_loop(self):
        while self.running:
            now = time.monotonic()
            try:
                while True:
                    msg = self.rx_queue.get_nowait()
                    if msg.get("t") == "invalid_frame":
                        self.rx_invalid_count += 1
                        if self.rx_invalid_count <= 5 or self.rx_invalid_count % 10 == 0:
                            print(f"[RX INVALIDO #{self.rx_invalid_count}] {msg.get('e')}")
                        continue

                    session_id = msg.get("u")
                    self.start_new_session_if_needed(session_id)

                    self.last_ground_msg_time = now
                    if not self.link_ok:
                        self.link_ok = True
                        if self.state == "LL" and not self.mission_running:
                            self.state = "I"
                            self.last_cmd = "N"
                        print("[LINK] Comunicación con estación terrena establecida")

                    await self.handle_message(msg)
            except queue.Empty:
                pass

            if self.last_ground_msg_time is not None:
                with self.telemetry_lock:
                    active_or_airborne = self.mission_running or self.armed or self.landed_state != "G"
                timeout_s = ACTIVE_LINK_TIMEOUT_S if active_or_airborne else LINK_TIMEOUT_S
                if now - self.last_ground_msg_time > timeout_s:
                    if self.link_ok:
                        self.link_ok = False
                        self.state = "LL"
                        print(f"[LINK] Comunicación con estación terrena perdida por más de {timeout_s:.1f} s")
                        if active_or_airborne and self.px4_connected:
                            self.last_error = "link_lost"
                            print("[SAFE] Enlace perdido durante estado activo. Solicitando aterrizaje.")
                            asyncio.create_task(self.request_land_internal("link_lost"))
                        else:
                            print("[SAFE] Enlace perdido en tierra/desarmado. Solo se reporta.")
            await asyncio.sleep(0.05)

    async def handle_message(self, msg: dict):
        msg_type = msg.get("t", "?")
        session_id = msg.get("u", self.current_session_id)

        if msg_type == "H":
            return

        print(f"[RX] {msg}")
        cmd_id = msg.get("c")
        if cmd_id is not None and self.resend_cached_ack(session_id, cmd_id):
            return

        if msg_type == "S":
            await self.handle_start_sweep(msg, session_id)
        elif msg_type == "Y":
            await self.handle_aoa_target(msg, session_id)
        elif msg_type == "L":
            await self.handle_land(msg, session_id)
        elif msg_type == "R":
            await self.handle_reboot_request(msg, session_id)
        elif msg_type == "M":
            await self.handle_arm_test(msg, session_id)
        else:
            self.send_ack(msg_type, False, "cmd_unknown", ack_cmd_id=cmd_id, session_id=session_id)

    def check_sweep_preconditions(self):
        if self.mission_running:
            return False, "busy"

        with self.telemetry_lock:
            px4_connected = self.px4_connected
            armed = self.armed
            flight_mode = self.flight_mode
            landed_state = self.landed_state
            health_ok = self.health_local and self.health_global and self.health_home

        if not px4_connected:
            return False, "no_px4"
        if armed:
            return False, "armed"
        if flight_mode != "P":
            return False, "need_position"
        if landed_state != "G":
            return False, "not_on_ground"
        if not health_ok:
            return False, "health_not_ok"
        if self.state not in ["I", "LL"]:
            return False, "busy"
        return True, ""

    async def handle_start_sweep(self, msg: dict, session_id: int):
        cmd_id = msg.get("c")
        self.last_cmd = "S"
        self.last_error = ""

        try:
            sweep_mode = str(msg.get("b", "C")).upper()
            height_m = float(msg["h"])
            yaw_rate = float(msg["w"])
            rotations = int(msg["n"])
            step_deg = float(msg.get("p", 30.0))
            hold_s = float(msg.get("d", 2.0))
        except (KeyError, ValueError, TypeError):
            self.last_error = "bad_params"
            self.send_ack("S", False, "bad_params", ack_cmd_id=cmd_id, session_id=session_id)
            return

        if sweep_mode not in ["C", "D"]:
            self.last_error = "bad_mode"
            self.send_ack("S", False, "bad_mode", ack_cmd_id=cmd_id, session_id=session_id)
            return

        if not (MIN_TAKEOFF_ALT_M <= height_m <= MAX_TAKEOFF_ALT_M):
            self.last_error = "bad_height"
            self.send_ack("S", False, "bad_height", ack_cmd_id=cmd_id, session_id=session_id)
            return

        if not (MIN_YAW_RATE_DEG_S <= yaw_rate <= MAX_YAW_RATE_DEG_S):
            self.last_error = "bad_yaw_rate"
            self.send_ack("S", False, "bad_yaw_rate", ack_cmd_id=cmd_id, session_id=session_id)
            return

        if not (MIN_ROTATIONS <= rotations <= MAX_ROTATIONS):
            self.last_error = "bad_rotations"
            self.send_ack("S", False, "bad_rotations", ack_cmd_id=cmd_id, session_id=session_id)
            return

        if sweep_mode == "D":
            if not (MIN_STEP_DEG <= step_deg <= MAX_STEP_DEG) or not (MIN_HOLD_S <= hold_s <= MAX_HOLD_S):
                self.last_error = "bad_discrete"
                self.send_ack("S", False, "bad_discrete", ack_cmd_id=cmd_id, session_id=session_id)
                return

        ok, reason = self.check_sweep_preconditions()
        if not ok:
            self.last_error = reason
            self.send_ack("S", False, reason, ack_cmd_id=cmd_id, session_id=session_id)
            print(f"[SAFE] Comenzar barrido rechazado: {reason}")
            return

        self.state = "TK"
        self.ready = self.px4_connected
        self.executed_cmd_id = cmd_id or self.executed_cmd_id
        self.land_requested = False
        self.landing_command_sent = False
        self.offboard_active = False
        self.yaw_ref_deg = None
        self.sweep_angle_bin_deg = None
        self.measurement_active = False
        self.active_sweep_mode = sweep_mode
        self.aoa_target_deg = None
        self.aoa_target_cmd_id = None
        self.mission_running = True
        self.set_progress(0.0)
        self.send_ack("S", True, ack_cmd_id=cmd_id, session_id=session_id)

        mode_text = "continuo" if sweep_mode == "C" else "discreto"
        print(
            f"[PX4] Barrido {mode_text} aceptado: altura={height_m:.1f} m, "
            f"yaw_rate={yaw_rate:.3f} deg/s, vueltas={rotations}, "
            f"paso={step_deg:.1f} deg, tiempo={hold_s:.1f} s"
        )
        self.mission_task = asyncio.create_task(
            self.sweep_mission_task(height_m, sweep_mode, yaw_rate, rotations, step_deg, hold_s)
        )

    async def handle_aoa_target(self, msg: dict, session_id: int):
        """Recibe el AoA estimado por la estación terrena para la orientación final."""
        cmd_id = msg.get("c")
        try:
            target = float(msg["g"]) % 360.0
        except (KeyError, ValueError, TypeError):
            self.last_error = "bad_aoa"
            self.send_ack("Y", False, "bad_aoa", ack_cmd_id=cmd_id, session_id=session_id)
            return

        if not self.mission_running or self.state != "QA":
            self.send_ack("Y", False, "not_waiting_aoa", ack_cmd_id=cmd_id, session_id=session_id)
            print(f"[SAFE] AoA rechazado: estado actual={self.state}, mission_running={self.mission_running}")
            return

        self.aoa_target_deg = target
        self.aoa_target_cmd_id = cmd_id
        self.executed_cmd_id = cmd_id or self.executed_cmd_id
        self.last_cmd = "Y"
        self.last_error = ""
        self.send_ack("Y", True, ack_cmd_id=cmd_id, session_id=session_id)
        print(f"[AOA] Objetivo recibido desde estación terrena: {target:.3f} deg absolutos respecto al norte")

    async def handle_land(self, msg: dict, session_id: int):
        cmd_id = msg.get("c")
        self.last_cmd = "L"
        self.last_error = ""
        if not self.px4_connected:
            self.state = "ER"
            self.last_error = "no_px4"
            self.send_ack("L", False, "no_px4", ack_cmd_id=cmd_id, session_id=session_id)
            return

        self.state = "LND"
        self.executed_cmd_id = cmd_id or self.executed_cmd_id
        self.land_requested = True
        self.send_ack("L", True, ack_cmd_id=cmd_id, session_id=session_id)
        print("[PX4] Comando Aterrizar recibido. Solicitando aterrizaje prioritario.")
        await self.request_land_internal("ground_cmd")

        if not self.mission_running:
            asyncio.create_task(self.finish_external_land_task())

    async def handle_reboot_request(self, msg: dict, session_id: int):
        cmd_id = msg.get("c")
        self.last_cmd = "R"
        self.last_error = "reboot_disabled"
        self.send_ack("R", False, "disabled", ack_cmd_id=cmd_id, session_id=session_id)
        print("[SAFE] Reboot request rechazado: en fase 3 el reinicio se realiza desde QGroundControl.")

    async def handle_arm_test(self, msg: dict, session_id: int):
        cmd_id = msg.get("c")
        self.last_cmd = "M"
        self.last_error = "arm_test_disabled"
        self.send_ack("M", False, "disabled", ack_cmd_id=cmd_id, session_id=session_id)
        print("[SAFE] Comando M rechazado: prueba de armado deshabilitada.")


    async def stop_offboard_if_needed(self):
        """
        Detiene OFFBOARD solo si la rutina cree que OFFBOARD está activo.

        En la versión 3.2 original esta función quedó referenciada pero no fue
        incluida, por eso la misión podía terminar bien físicamente y aun así
        reportar mission_fail al intentar cerrar OFFBOARD. Esta función corrige
        ese cierre lógico sin cambiar la rutina de vuelo.
        """
        if not self.offboard_active:
            return

        try:
            await self.drone.offboard.stop()
            print("[PX4] OFFBOARD detenido correctamente.")
        except OffboardError as e:
            # No lo tratamos como fallo crítico si PX4 ya salió de OFFBOARD
            # por LAND u otro cambio de modo. Solo se reporta en terminal.
            print(f"[PX4 WARN] No se pudo detener OFFBOARD explícitamente: {e}")
        except Exception as e:
            print(f"[PX4 WARN] Error no crítico al detener OFFBOARD: {e}")
        finally:
            self.offboard_active = False

    async def request_land_internal(self, reason: str):
        self.land_requested = True
        self.measurement_active = False
        if self.landing_command_sent:
            return
        self.landing_command_sent = True
        if not self.px4_connected:
            self.last_error = "no_px4"
            return

        if self.offboard_active:
            try:
                await self.drone.offboard.stop()
                self.offboard_active = False
                print(f"[PX4] OFFBOARD detenido antes de land() ({reason}).")
            except OffboardError as e:
                print(f"[PX4 WARN] No se pudo detener OFFBOARD antes de land(): {e}")
            except Exception as e:
                print(f"[PX4 WARN] Error al detener OFFBOARD antes de land(): {e}")

        try:
            await self.drone.action.land()
            print(f"[PX4] land() enviado ({reason}).")
        except Exception as e:
            self.last_error = "land_fail"
            print(f"[PX4 ERROR] No se pudo enviar land() ({reason}): {e}")

    async def wait_relative_altitude(self, target_alt_m: float) -> bool:
        t0 = time.monotonic()
        last_print = 0.0
        print(f"[PX4] Esperando altura relativa {target_alt_m:.1f} m...")
        while self.running and not self.land_requested:
            with self.telemetry_lock:
                alt = self.relative_alt_m
            now = time.monotonic()
            if now - last_print >= 1.0:
                print(f"[PX4] Altura relativa: {alt} m, objetivo={target_alt_m:.1f} m")
                last_print = now
            if alt is not None:
                ratio = max(0.0, min(1.0, alt / max(target_alt_m, 0.1)))
                self.set_progress(2.0 + 10.0 * ratio)
                if alt >= target_alt_m - TAKEOFF_ALT_TOL_M:
                    print("[PX4] Altura objetivo alcanzada.")
                    return True
            if now - t0 > TAKEOFF_TIMEOUT_S:
                print("[PX4] Timeout esperando altura objetivo.")
                return False
            await asyncio.sleep(0.2)
        return False

    async def wait_on_ground_by_telemetry(self, timeout_s: Optional[float] = None) -> bool:
        t0 = time.monotonic()
        while self.running:
            with self.telemetry_lock:
                on_ground = self.landed_state == "G"
                armed = self.armed
            if on_ground and not armed:
                print("[PX4] Vehículo en tierra y desarmado.")
                self.set_progress(100.0)
                return True
            if timeout_s is not None and time.monotonic() - t0 > timeout_s:
                return False
            await asyncio.sleep(0.3)

    def get_current_yaw(self) -> float:
        with self.telemetry_lock:
            yaw = self.yaw_deg
        if yaw is None:
            return 0.0
        return float(yaw)

    def get_current_flight_mode(self) -> str:
        with self.telemetry_lock:
            return str(self.flight_mode or "U")

    async def wait_offboard_mode_for_sweep(self, hold_yaw: float, timeout_s: float = 8.0) -> bool:
        """
        Espera a que la telemetría confirme OFFBOARD antes de avanzar el barrido.

        En algunas pruebas PX4 puede reportar HOLD durante unos segundos aunque ya
        se estén enviando setpoints. En continuo, si el ángulo de referencia sigue
        avanzando durante ese lapso, al recuperar OFFBOARD el dron intenta alcanzar
        de golpe una referencia adelantada. Por eso aquí se congela el yaw de
        barrido hasta que el modo reportado vuelva a OFFBOARD.
        """
        t0 = time.monotonic()
        self.measurement_active = False
        self.sweep_angle_bin_deg = None
        self.yaw_ref_deg = round(wrap_360(hold_yaw), 1)
        while self.running and not self.land_requested:
            if self.get_current_flight_mode() == "O":
                return True
            if time.monotonic() - t0 > timeout_s:
                print("[PX4 WARN] OFFBOARD no confirmado por telemetría; reintentando start().")
                try:
                    await self.drone.offboard.start()
                    self.offboard_active = True
                    t0 = time.monotonic()
                except OffboardError as e:
                    print(f"[PX4 WARN] Reintento OFFBOARD rechazado durante barrido: {e}")
                    t0 = time.monotonic()
                except Exception as e:
                    print(f"[PX4 WARN] Error en reintento OFFBOARD durante barrido: {e}")
                    t0 = time.monotonic()
            await self.send_offboard_setpoint(hold_yaw)
            await asyncio.sleep(1.0 / OFFBOARD_SETPOINT_RATE_HZ)
        return False

    def get_current_local_position(self):
        """Devuelve la última posición local NED disponible como (north, east, down)."""
        with self.telemetry_lock:
            n = self.local_north_m
            e = self.local_east_m
            d = self.local_down_m
        if n is None or e is None or d is None:
            return None
        return float(n), float(e), float(d)

    async def wait_local_position(self, timeout_s: float = 5.0):
        """Espera a que MAVSDK entregue position_velocity_ned()."""
        t0 = time.monotonic()
        while self.running and not self.land_requested:
            pos = self.get_current_local_position()
            if pos is not None:
                return pos
            if time.monotonic() - t0 > timeout_s:
                return None
            await asyncio.sleep(0.05)
        return None

    async def capture_offboard_hold_position(self) -> bool:
        """
        Captura la posición local actual para mantenerla durante todo el barrido.

        Se usa PositionNedYaw en OFFBOARD, no solo VelocityNedYaw(0,0,0,yaw),
        porque para levantar patrones RSSI conviene que el dron conserve tanto
        la posición horizontal como la altura local mientras gira en yaw.
        """
        pos = await self.wait_local_position(timeout_s=5.0)
        if pos is None:
            self.last_error = "no_local_ned"
            print("[SAFE] No hay posición local NED para mantener posición en OFFBOARD.")
            return False
        n, e, d = pos
        self.offboard_hold_north_m = n
        self.offboard_hold_east_m = e
        self.offboard_hold_down_m = d
        print(
            "[PX4] Posición local fija para barrido: "
            f"N={n:.2f} m, E={e:.2f} m, D={d:.2f} m"
        )
        return True

    async def send_offboard_setpoint(self, yaw_deg: float):
        """
        Envía setpoint OFFBOARD manteniendo posición local fija y cambiando yaw.

        PositionNedYaw(north, east, down, yaw) mantiene el dron alrededor de la
        posición NED capturada justo antes de iniciar OFFBOARD. Esto reduce la
        pérdida de altura y la deriva horizontal durante barridos continuo y discreto.
        """
        if (
            self.offboard_hold_north_m is not None
            and self.offboard_hold_east_m is not None
            and self.offboard_hold_down_m is not None
        ):
            await self.drone.offboard.set_position_ned(
                PositionNedYaw(
                    self.offboard_hold_north_m,
                    self.offboard_hold_east_m,
                    self.offboard_hold_down_m,
                    wrap_360(yaw_deg),
                )
            )
        else:
            # En esta fase evitamos caer a VelocityNedYaw. Si no hay referencia NED
            # fija, no conviene seguir el barrido porque se pierde la garantía de
            # mantener posición y altura. La misión lo capturará como excepción y aterrizará.
            raise RuntimeError("offboard_hold_position_missing")

    async def hold_yaw_for(self, yaw_deg: float, duration_s: float):
        dt = 1.0 / OFFBOARD_SETPOINT_RATE_HZ
        steps = max(1, int(duration_s / dt))
        for _ in range(steps):
            if self.land_requested:
                return False
            await self.send_offboard_setpoint(yaw_deg)
            await asyncio.sleep(dt)
        return True

    async def start_offboard_with_retry(self, yaw_deg: float) -> bool:
        self.state = "OF"
        for attempt in range(1, OFFBOARD_ATTEMPTS + 1):
            if self.land_requested:
                return False
            print(
                f"[PX4] Preparando OFFBOARD intento {attempt}/{OFFBOARD_ATTEMPTS}: "
                f"setpoints de posición fija durante {OFFBOARD_PRESTREAM_S:.1f} s"
            )
            ok = await self.hold_yaw_for(yaw_deg, OFFBOARD_PRESTREAM_S)
            if not ok:
                return False
            try:
                await self.drone.offboard.start()
                self.offboard_active = True
                print("[PX4] OFFBOARD aceptado.")
                await self.hold_yaw_for(yaw_deg, 0.5)
                return True
            except OffboardError as e:
                print(f"[PX4 WARN] OFFBOARD rechazado intento {attempt}: {e}")
                await asyncio.sleep(0.5)
        return False

    async def rotate_shortest_to_yaw(
        self,
        current_yaw: float,
        target_yaw: float,
        yaw_rate: float,
        hold_s: float = NORTH_ALIGN_HOLD_S,
        label: str = "referencia",
    ) -> bool:
        """Orienta el dron a un yaw absoluto usando el camino angular más corto."""
        self.measurement_active = False
        self.sweep_angle_bin_deg = None
        current = wrap_360(current_yaw)
        target = wrap_360(target_yaw)
        self.yaw_ref_deg = round(target, 1)
        delta = wrap_180(target - current)
        if abs(delta) <= 0.5:
            print(f"[PX4] Yaw ya próximo a {label}: {current:.1f} -> {target:.1f} deg")
            return await self.hold_yaw_for(target, hold_s)

        direction = 1.0 if delta >= 0.0 else -1.0
        duration_s = abs(delta) / yaw_rate
        dt = 1.0 / OFFBOARD_SETPOINT_RATE_HZ
        steps = max(1, int(duration_s / dt))
        print(
            f"[PX4] Orientando a {label}: {current:.1f} deg -> {target:.1f} deg "
            f"({abs(delta):.1f} deg a {yaw_rate:.1f} deg/s)"
        )
        for i in range(steps + 1):
            if self.land_requested:
                return False
            frac = i / steps
            yaw_cmd = current + direction * abs(delta) * frac
            self.yaw_ref_deg = round(wrap_360(yaw_cmd), 1)
            await self.send_offboard_setpoint(yaw_cmd)
            await asyncio.sleep(dt)
        self.yaw_ref_deg = round(target, 1)
        return await self.hold_yaw_for(target, hold_s)

    async def wait_for_aoa_target(self, timeout_s: float = AOA_WAIT_TIMEOUT_S) -> Optional[float]:
        """Espera el AoA sin dejar de transmitir setpoints OFFBOARD."""
        t0 = time.monotonic()
        hold_yaw = self.yaw_ref_deg if self.yaw_ref_deg is not None else self.get_current_yaw()
        hold_yaw = wrap_360(hold_yaw)
        self.state = "QA"
        self.measurement_active = False
        self.sweep_angle_bin_deg = None
        self.yaw_ref_deg = round(hold_yaw, 1)
        print(f"[AOA] Esperando AoA de estación terrena hasta {timeout_s:.1f} s; manteniendo OFFBOARD.")

        while self.running and not self.land_requested:
            if self.aoa_target_deg is not None:
                return float(self.aoa_target_deg) % 360.0
            if time.monotonic() - t0 > timeout_s:
                return None
            await self.send_offboard_setpoint(hold_yaw)
            await asyncio.sleep(1.0 / OFFBOARD_SETPOINT_RATE_HZ)
        return None

    async def continuous_sweep(self, yaw_rate: float, rotations: int) -> bool:
        self.state = "BC"
        self.active_sweep_mode = "C"
        self.measurement_active = False
        total_angle = 360.0 * rotations
        dt = 1.0 / OFFBOARD_SETPOINT_RATE_HZ
        sweep_angle = 0.0
        print(
            f"[PX4] Barrido continuo desde norte: {rotations} vuelta(s), "
            f"{yaw_rate:.3f} deg/s. El avance se pausa si PX4 no reporta OFFBOARD."
        )

        # No empieces a medir ni a avanzar la referencia hasta que PX4 confirme OFFBOARD.
        ok = await self.wait_offboard_mode_for_sweep(0.0, timeout_s=8.0)
        if not ok:
            return False

        last_t = time.monotonic()
        while self.running and not self.land_requested and sweep_angle <= total_angle:
            now = time.monotonic()
            flight_mode = self.get_current_flight_mode()

            if flight_mode != "O":
                # Pausa segura: mantener la última referencia y NO adquirir RSSI para patrón.
                self.measurement_active = False
                self.sweep_angle_bin_deg = None
                self.yaw_ref_deg = round(wrap_360(sweep_angle), 1)
                await self.send_offboard_setpoint(sweep_angle)
                ok = await self.wait_offboard_mode_for_sweep(sweep_angle, timeout_s=8.0)
                if not ok:
                    return False
                last_t = time.monotonic()
                continue

            elapsed = max(0.0, now - last_t)
            last_t = now
            sweep_angle = min(total_angle, sweep_angle + yaw_rate * elapsed)
            yaw_cmd = sweep_angle
            self.yaw_ref_deg = round(wrap_360(yaw_cmd), 1)
            self.sweep_angle_bin_deg = self.yaw_ref_deg
            self.measurement_active = True
            await self.send_offboard_setpoint(yaw_cmd)
            self.set_progress(18.0 + 70.0 * (sweep_angle / total_angle))
            if sweep_angle >= total_angle:
                break
            await asyncio.sleep(dt)

        self.measurement_active = False
        self.sweep_angle_bin_deg = None
        return not self.land_requested

    async def rotate_positive_smooth(self, current_cmd_yaw: float, target_cmd_yaw: float, yaw_rate: float) -> bool:
        """Avanza en yaw positivo desde current_cmd_yaw hasta target_cmd_yaw."""
        self.measurement_active = False
        delta = max(0.0, target_cmd_yaw - current_cmd_yaw)
        if delta <= 1e-6:
            return True
        duration_s = delta / yaw_rate
        dt = 1.0 / OFFBOARD_SETPOINT_RATE_HZ
        steps = max(1, int(duration_s / dt))
        for i in range(steps + 1):
            if self.land_requested:
                return False
            frac = i / steps
            yaw_cmd = current_cmd_yaw + delta * frac
            self.yaw_ref_deg = round(wrap_360(yaw_cmd), 1)
            self.sweep_angle_bin_deg = None
            await self.send_offboard_setpoint(yaw_cmd)
            await asyncio.sleep(dt)
        return True

    async def discrete_sweep(self, yaw_rate: float, rotations: int, step_deg: float, hold_s: float) -> bool:
        self.state = "BD"
        self.active_sweep_mode = "D"
        self.measurement_active = False
        total_angle = 360.0 * rotations
        current_cmd = 0.0
        print(
            f"[PX4] Barrido discreto desde norte: {rotations} vuelta(s), paso={step_deg:.1f} deg, "
            f"hold={hold_s:.1f} s, transición={yaw_rate:.3f} deg/s"
        )

        # Secuencia angular absoluta: 0, paso, 2*paso, ..., 360*vueltas.
        # El punto final 360 equivale a 0 deg y sirve para cerrar la vuelta.
        targets = [0.0]
        a = step_deg
        while a < total_angle:
            targets.append(a)
            a += step_deg
        targets.append(total_angle)

        for target_delta in targets:
            if self.land_requested:
                return False
            ok = await self.rotate_positive_smooth(current_cmd, target_delta, yaw_rate)
            if not ok:
                return False
            current_cmd = target_delta
            angle_bin = round(wrap_360(current_cmd), 1)
            self.yaw_ref_deg = angle_bin
            self.sweep_angle_bin_deg = angle_bin
            self.measurement_active = True
            self.set_progress(18.0 + 70.0 * (target_delta / total_angle))
            print(f"[PX4] Punto discreto yaw={wrap_360(current_cmd):.1f} deg, hold={hold_s:.1f} s")
            ok = await self.hold_yaw_for(current_cmd, hold_s)
            self.measurement_active = False
            if not ok:
                return False
        return True

    async def sweep_mission_task(
        self,
        target_alt_m: float,
        sweep_mode: str,
        yaw_rate: float,
        rotations: int,
        step_deg: float,
        hold_s: float,
    ):
        try:
            self.state = "TK"
            self.last_cmd = "S"
            self.last_error = ""
            self.set_progress(0.0)

            print("[PX4] Configurando altura de despegue...")
            await self.drone.action.set_takeoff_altitude(target_alt_m)

            print("[PX4] Armando...")
            await self.drone.action.arm()

            print("[PX4] Takeoff usando PX4 interno...")
            await self.drone.action.takeoff()

            reached = await self.wait_relative_altitude(target_alt_m)
            if self.land_requested:
                print("[PX4] Aterrizaje solicitado durante despegue/ascenso.")
                self.state = "LND"
                await self.request_land_internal("requested_during_takeoff")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                self.last_cmd = "LC"
                return

            if not reached:
                self.last_error = "alt_timeout"
                self.last_cmd = "SE"
                self.state = "LND"
                print("[SAFE] No se alcanzó la altura. Aterrizando por seguridad.")
                await self.request_land_internal("alt_timeout")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                return

            self.state = "HT"
            self.set_progress(12.0)
            print(f"[PX4] Altura alcanzada. Esperando {WAIT_AFTER_TAKEOFF_S:.1f} s antes del barrido...")
            t0 = time.monotonic()
            while time.monotonic() - t0 < WAIT_AFTER_TAKEOFF_S:
                if self.land_requested:
                    break
                await asyncio.sleep(0.1)

            if self.land_requested:
                self.state = "LND"
                await self.request_land_internal("requested_before_sweep")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                self.last_cmd = "LC"
                return

            current_yaw = self.get_current_yaw()
            captured = await self.capture_offboard_hold_position()
            if not captured:
                self.last_error = "no_local_ned"
                self.last_cmd = "SE"
                self.state = "LND"
                print("[SAFE] No fue posible capturar posición local. Aterrizando.")
                await self.request_land_internal("no_local_ned")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                return

            started = await self.start_offboard_with_retry(current_yaw)
            if not started:
                self.last_error = "offboard_fail"
                self.last_cmd = "SE"
                self.state = "LND"
                print("[SAFE] No fue posible entrar a OFFBOARD. Aterrizando.")
                await self.request_land_internal("offboard_fail")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                return

            self.state = "N0"
            self.set_progress(15.0)
            aligned = await self.rotate_shortest_to_yaw(
                self.get_current_yaw(),
                0.0,
                NORTH_ALIGN_YAW_RATE_DEG_S,
                hold_s=NORTH_ALIGN_HOLD_S,
                label="norte (0 deg)",
            )
            if not aligned or self.land_requested:
                print("[PX4] Barrido interrumpido durante orientación a norte. Aterrizando.")
                self.state = "LND"
                await self.request_land_internal("north_align_interrupted")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                self.last_cmd = "LC"
                return

            print(f"[PX4] Norte estabilizado durante {NORTH_ALIGN_HOLD_S:.1f} s. Iniciando barrido.")
            self.set_progress(18.0)
            if sweep_mode == "C":
                sweep_ok = await self.continuous_sweep(yaw_rate, rotations)
            else:
                sweep_ok = await self.discrete_sweep(yaw_rate, rotations, step_deg, hold_s)

            if not sweep_ok or self.land_requested:
                print("[PX4] Barrido interrumpido. Aterrizando.")
                self.state = "LND"
                await self.request_land_internal("sweep_interrupted")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                self.last_cmd = "LC"
                return

            # Fase 4.1: no aterrizamos todavía. Mantenemos OFFBOARD y esperamos el AoA
            # calculado por la estación terrena a partir del patrón RSSI final.
            self.measurement_active = False
            self.sweep_angle_bin_deg = None
            self.set_progress(90.0)
            aoa_target = await self.wait_for_aoa_target(AOA_WAIT_TIMEOUT_S)

            if self.land_requested:
                print("[PX4] Aterrizaje prioritario solicitado mientras se esperaba el AoA.")
                self.state = "LND"
                await self.request_land_internal("land_during_aoa_wait")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                self.last_cmd = "LC"
                return

            if aoa_target is None:
                self.last_error = "aoa_timeout"
                self.last_cmd = "SE"
                self.state = "LND"
                print("[SAFE] No llegó AoA válido dentro del tiempo límite. Aterrizando.")
                await self.request_land_internal("aoa_timeout")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                return

            self.state = "YA"
            self.set_progress(94.0)
            oriented = await self.rotate_shortest_to_yaw(
                self.get_current_yaw(),
                aoa_target,
                AOA_ORIENT_YAW_RATE_DEG_S,
                hold_s=AOA_FINAL_HOLD_S,
                label=f"AoA estimado ({aoa_target:.1f} deg)",
            )
            if not oriented or self.land_requested:
                self.state = "LND"
                await self.request_land_internal("aoa_orientation_interrupted")
                await self.wait_on_ground_by_telemetry(timeout_s=90.0)
                self.state = "I"
                self.last_cmd = "LC"
                return

            self.last_cmd = "YC"
            self.set_progress(96.0)
            print(f"[AOA] Orientación final completada a {aoa_target:.2f} deg. Aterrizando...")

            await self.stop_offboard_if_needed()
            self.state = "LND"
            await self.request_land_internal("aoa_oriented")
            await self.wait_on_ground_by_telemetry(timeout_s=90.0)

            # Cierre lógico: solo se marca completo cuando PX4 reporta tierra y desarmado.
            self.state = "I"
            self.ready = self.px4_connected
            self.last_cmd = "SC"
            self.last_error = ""
            self.measurement_active = False
            self.yaw_ref_deg = None
            self.sweep_angle_bin_deg = None
            self.active_sweep_mode = ""
            self.set_progress(100.0)
            print("[PX4] Fase 4.1 terminada correctamente: barrido, AoA recibido, orientación final y aterrizaje.")

        except ActionError as e:
            self.last_error = "action_error"
            self.last_cmd = "SE"
            self.state = "LND"
            print(f"[PX4 ERROR] PX4 rechazó una acción: {e}")
            await self.request_land_internal("action_error")
            await self.wait_on_ground_by_telemetry(timeout_s=90.0)
            self.state = "I"

        except OffboardError as e:
            self.last_error = "offboard_error"
            self.last_cmd = "SE"
            self.state = "LND"
            print(f"[PX4 ERROR] Error de OFFBOARD: {e}")
            await self.request_land_internal("offboard_error")
            await self.wait_on_ground_by_telemetry(timeout_s=90.0)
            self.state = "I"

        except Exception as e:
            self.last_error = "mission_fail"
            self.last_cmd = "SE"
            self.state = "LND"
            print(f"[PX4 ERROR] Falló la rutina de barrido: {e}")
            await self.request_land_internal("mission_fail")
            await self.wait_on_ground_by_telemetry(timeout_s=90.0)
            self.state = "I"

        finally:
            await self.stop_offboard_if_needed()
            self.mission_running = False
            self.land_requested = False
            self.landing_command_sent = False
            self.offboard_active = False
            self.measurement_active = False
            self.active_sweep_mode = ""
            self.aoa_target_deg = None
            self.aoa_target_cmd_id = None
            self.ready = self.px4_connected

    async def finish_external_land_task(self):
        self.state = "LND"
        await self.wait_on_ground_by_telemetry(timeout_s=90.0)
        self.state = "I"
        self.ready = self.px4_connected
        self.last_cmd = "LC"
        self.land_requested = False
        self.landing_command_sent = False
        self.set_progress(100.0)

    async def connect_px4_and_telemetry(self):
        print(f"[PX4] Conectando con PX4 por {self.px4_address} ...")
        try:
            await self.drone.connect(system_address=self.px4_address)
        except Exception as e:
            self.last_error = "px4_connect_fail"
            print(f"[PX4 ERROR] No se pudo iniciar conexión MAVSDK: {e}")
            return

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                with self.telemetry_lock:
                    self.px4_connected = True
                self.ready = True
                print("[PX4] Conectado correctamente a PX4.")
                break

        tasks = [
            asyncio.create_task(self.telemetry_attitude_loop()),
            asyncio.create_task(self.telemetry_position_loop()),
            asyncio.create_task(self.telemetry_position_velocity_ned_loop()),
            asyncio.create_task(self.telemetry_health_loop()),
            asyncio.create_task(self.telemetry_flight_mode_loop()),
            asyncio.create_task(self.telemetry_armed_loop()),
            asyncio.create_task(self.telemetry_landed_state_loop()),
        ]
        await asyncio.gather(*tasks)

    async def telemetry_attitude_loop(self):
        async for att in self.drone.telemetry.attitude_euler():
            with self.telemetry_lock:
                self.yaw_deg = yaw_to_0_360(att.yaw_deg)

    async def telemetry_position_loop(self):
        async for pos in self.drone.telemetry.position():
            with self.telemetry_lock:
                self.relative_alt_m = clean_altitude(pos.relative_altitude_m)
                self.latitude_deg = clean_global_coord(pos.latitude_deg)
                self.longitude_deg = clean_global_coord(pos.longitude_deg)

    async def telemetry_position_velocity_ned_loop(self):
        async for pv in self.drone.telemetry.position_velocity_ned():
            p = pv.position
            with self.telemetry_lock:
                self.local_north_m = clean_local_value(p.north_m)
                self.local_east_m = clean_local_value(p.east_m)
                self.local_down_m = clean_local_value(p.down_m)

    async def telemetry_health_loop(self):
        async for h in self.drone.telemetry.health():
            with self.telemetry_lock:
                self.health_local = bool(h.is_local_position_ok)
                self.health_global = bool(h.is_global_position_ok)
                self.health_home = bool(h.is_home_position_ok)

    async def telemetry_flight_mode_loop(self):
        async for mode in self.drone.telemetry.flight_mode():
            with self.telemetry_lock:
                self.flight_mode = compact_flight_mode(mode)

    async def telemetry_armed_loop(self):
        async for armed in self.drone.telemetry.armed():
            with self.telemetry_lock:
                self.armed = bool(armed)

    async def telemetry_landed_state_loop(self):
        async for ls in self.drone.telemetry.landed_state():
            with self.telemetry_lock:
                self.landed_state = compact_landed_state(ls)

    def close(self):
        self.running = False
        time.sleep(0.2)
        try:
            self.ser.close()
        except Exception:
            pass


async def async_main():
    node = DroneRadioPX4Node(RADIO_PORT, RADIO_BAUD, PX4_SYSTEM_ADDRESS)
    print(f"Nodo Raspberry abierto en {RADIO_PORT} a {RADIO_BAUD} baudios")
    print("Formato de trama: $JSON_COMPACTO*CRC32")
    print(f"Status: {STATUS_PERIOD_S} s | timeout enlace: {LINK_TIMEOUT_S} s")
    print(f"PX4: {PX4_SYSTEM_ADDRESS}")
    print("Fase 4.9: despegue, norte, barrido, espera de AoA, orientación final al AoA y aterrizaje.")

    threading.Thread(target=node.receive_loop, daemon=True).start()

    tasks = [
        asyncio.create_task(node.connect_px4_and_telemetry()),
        asyncio.create_task(node.status_loop()),
        asyncio.create_task(node.process_rx_loop()),
    ]
    try:
        await asyncio.gather(*tasks)
    except KeyboardInterrupt:
        print("\nInterrupción manual.")
    finally:
        node.close()


def main():
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("\nInterrupción manual.")


if __name__ == "__main__":
    main()
