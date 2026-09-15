#!/usr/bin/env python3
"""

Raspberry Pi 5 + PX4/MAVSDK.
Despega a 5 m, entra a OFFBOARD, realiza un cuadro de 10 m con yaw absoluto
0/90/180/270 y aterriza.

Configuración:
- Raspberry Pi 5: serial:///dev/ttyAMA0:57600
- PX4 + MAVSDK
- Altura: 5 m
- Cuadro: 10 m por lado
- Velocidad máxima: 1 m/s
"""

import asyncio
import math
import time
from typing import Optional, Tuple

from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError, PositionNedYaw
from mavsdk.telemetry import LandedState


SERIAL = "serial:///dev/ttyAMA0:57600"

TAKEOFF_ALT_M = 5.0
SIDE_M = 10.0

# Velocidad máxima de la referencia durante cada lado.
SPEED_M_S = 1.0

WAIT_AFTER_TAKEOFF_S = 2.0
WAIT_AFTER_YAW_S = 0.5
WAIT_AFTER_MOVE_S = 2.0

OFFBOARD_ATTEMPTS = 3
OFFBOARD_PRESTREAM_S = 3.0
OFFBOARD_SETPOINT_RATE_HZ = 20.0

YAW_TURN_RATE_DEG_S = 45.0
YAW_SETPOINT_RATE_HZ = 20.0

# Aceleración/desaceleración horizontal de la referencia.
HORIZONTAL_ACCEL_LIMIT_M_S2 = 0.40



def wrap_180(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def wrap_360(angle_deg: float) -> float:
    return float(angle_deg) % 360.0


async def wait_connection(drone: System):
    print("Esperando conexión MAVSDK/PX4...")
    async for cs in drone.core.connection_state():
        if cs.is_connected:
            print("-- Conectado")
            return


async def wait_health_ready(drone: System, timeout_s: float = 60.0) -> bool:
    print("Esperando estimación local/global y home position...")
    t0 = time.monotonic()
    last_print = 0.0

    async for h in drone.telemetry.health():
        now = time.monotonic()
        if now - last_print >= 1.0:
            print(
                "Health: "
                f"local={h.is_local_position_ok}, "
                f"global={h.is_global_position_ok}, "
                f"home={h.is_home_position_ok}"
            )
            last_print = now

        if h.is_local_position_ok and h.is_global_position_ok and h.is_home_position_ok:
            print("-- PX4 listo para Position/Offboard")
            return True

        if now - t0 > timeout_s:
            print("-- La estimación no quedó lista dentro del tiempo límite")
            return False

        await asyncio.sleep(0.2)

    return False


async def wait_altitude_timeout(
    drone: System,
    target_alt_m: float,
    tol_m: float = 0.5,
    timeout_s: float = 30.0,
) -> bool:
    print(f"Esperando alcanzar aproximadamente {target_alt_m:.1f} m...")
    t0 = time.monotonic()
    last_print = 0.0

    async for pos in drone.telemetry.position():
        alt_m = float(pos.relative_altitude_m)
        now = time.monotonic()

        if now - last_print >= 1.0:
            print(
                f"Altura relativa estimada: {alt_m:.2f} m "
                f"(objetivo {target_alt_m:.2f} m)"
            )
            last_print = now

        if alt_m >= target_alt_m - tol_m:
            print("-- Altura alcanzada")
            return True

        if now - t0 > timeout_s:
            print("-- No se alcanzó la altura dentro del tiempo límite")
            return False

        await asyncio.sleep(0.2)

    return False


async def wait_on_ground(drone: System):
    print("Esperando detección de tierra...")
    async for ls in drone.telemetry.landed_state():
        if ls == LandedState.ON_GROUND:
            print("-- Vehículo en tierra")
            return
        await asyncio.sleep(0.5)


async def get_current_local_position(
    drone: System,
) -> Optional[Tuple[float, float, float]]:
    async for pv in drone.telemetry.position_velocity_ned():
        p = pv.position
        return float(p.north_m), float(p.east_m), float(p.down_m)
    return None


async def get_current_yaw_deg(drone: System) -> float:
    async for att in drone.telemetry.attitude_euler():
        return wrap_360(att.yaw_deg)
    return 0.0


async def stop_offboard_if_needed(drone: System):
    try:
        await drone.offboard.stop()
        print("-- OFFBOARD detenido")
    except OffboardError as e:
        print(f"No se pudo detener OFFBOARD explícitamente: {e}")
    except Exception as e:
        print(f"No se pudo detener OFFBOARD explícitamente: {e}")


async def land_safely(drone: System):
    print("Aterrizando por seguridad...")
    await stop_offboard_if_needed(drone)
    try:
        await drone.action.land()
    except ActionError as e:
        print(f"No se pudo enviar land(): {e}")
    await wait_on_ground(drone)


async def send_position_setpoint(
    drone: System,
    north_m: float,
    east_m: float,
    down_m: float,
    yaw_deg: float,
):
    await drone.offboard.set_position_ned(
        PositionNedYaw(
            float(north_m),
            float(east_m),
            float(down_m),
            wrap_360(yaw_deg),
        )
    )


async def hold_position_for(
    drone: System,
    north_m: float,
    east_m: float,
    down_m: float,
    yaw_deg: float,
    duration_s: float,
    rate_hz: float = OFFBOARD_SETPOINT_RATE_HZ,
) -> bool:
    dt = 1.0 / rate_hz
    t_end = time.monotonic() + duration_s

    while time.monotonic() < t_end:
        loop_start = time.monotonic()

        await send_position_setpoint(
            drone, north_m, east_m, down_m, yaw_deg
        )

        elapsed = time.monotonic() - loop_start
        sleep_s = dt - elapsed
        if sleep_s > 0.0:
            await asyncio.sleep(sleep_s)

    return True


async def start_offboard_with_retry(
    drone: System,
    north_m: float,
    east_m: float,
    down_m: float,
    yaw_deg: float,
) -> bool:
    for attempt in range(1, OFFBOARD_ATTEMPTS + 1):
        print(
            f"Preparando OFFBOARD, intento {attempt}/{OFFBOARD_ATTEMPTS}: "
            f"posición fija durante {OFFBOARD_PRESTREAM_S:.1f} s..."
        )

        await hold_position_for(
            drone,
            north_m,
            east_m,
            down_m,
            yaw_deg,
            OFFBOARD_PRESTREAM_S,
        )

        try:
            print("Solicitando OFFBOARD...")
            await drone.offboard.start()
            print("-- OFFBOARD aceptado por PX4")

            await hold_position_for(
                drone, north_m, east_m, down_m, yaw_deg, 0.5
            )
            return True

        except OffboardError as e:
            print(f"OFFBOARD rechazado en intento {attempt}: {e}")

            await hold_position_for(
                drone, north_m, east_m, down_m, yaw_deg, 1.0
            )
            await asyncio.sleep(0.5)

    return False


async def yaw_smooth_position(
    drone: System,
    north_m: float,
    east_m: float,
    down_m: float,
    current_yaw_deg: float,
    target_yaw_deg: float,
) -> float:
    current = wrap_360(current_yaw_deg)
    target = wrap_360(target_yaw_deg)
    delta = wrap_180(target - current)

    if abs(delta) <= 0.5:
        await hold_position_for(
            drone,
            north_m,
            east_m,
            down_m,
            target,
            WAIT_AFTER_YAW_S,
        )
        return target

    direction = 1.0 if delta > 0.0 else -1.0
    dt = 1.0 / YAW_SETPOINT_RATE_HZ
    yaw_step_deg = max(0.1, YAW_TURN_RATE_DEG_S * dt)
    steps = max(1, int(math.ceil(abs(delta) / yaw_step_deg)))

    print(
        f"Giro suave: {current:.1f} deg -> {target:.1f} deg "
        f"({abs(delta):.1f} deg)"
    )

    for i in range(1, steps + 1):
        loop_start = time.monotonic()

        yaw_cmd = current + direction * min(
            i * yaw_step_deg,
            abs(delta),
        )

        await send_position_setpoint(
            drone, north_m, east_m, down_m, yaw_cmd
        )

        elapsed = time.monotonic() - loop_start
        sleep_s = dt - elapsed
        if sleep_s > 0.0:
            await asyncio.sleep(sleep_s)

    await hold_position_for(
        drone,
        north_m,
        east_m,
        down_m,
        target,
        WAIT_AFTER_YAW_S,
    )

    return target



def motion_profile_parameters(
    distance_m: float,
    speed_m_s: float,
    accel_m_s2: float,
):
    """
    Calcula los parámetros de un perfil trapezoidal o triangular.

    Retorna:
        profile_type, v_peak, t_acc, t_cruise, t_total, d_acc
    """
    if distance_m <= 0.0:
        raise ValueError("distance_m debe ser mayor que cero")
    if speed_m_s <= 0.0:
        raise ValueError("speed_m_s debe ser mayor que cero")
    if accel_m_s2 <= 0.0:
        raise ValueError("accel_m_s2 debe ser mayor que cero")

    t_acc_nom = speed_m_s / accel_m_s2
    d_acc_nom = 0.5 * accel_m_s2 * t_acc_nom**2

    if 2.0 * d_acc_nom >= distance_m:
        # El segmento es demasiado corto para alcanzar speed_m_s.
        t_acc = math.sqrt(distance_m / accel_m_s2)
        v_peak = accel_m_s2 * t_acc
        t_cruise = 0.0
        t_total = 2.0 * t_acc
        d_acc = 0.5 * accel_m_s2 * t_acc**2
        profile_type = "triangular"

    else:
        t_acc = t_acc_nom
        v_peak = speed_m_s
        d_acc = d_acc_nom

        d_cruise = distance_m - 2.0 * d_acc
        t_cruise = d_cruise / speed_m_s
        t_total = 2.0 * t_acc + t_cruise
        profile_type = "trapezoidal"

    return (
        profile_type,
        v_peak,
        t_acc,
        t_cruise,
        t_total,
        d_acc,
    )


def distance_from_elapsed_time(
    elapsed_s: float,
    distance_m: float,
    accel_m_s2: float,
    v_peak_m_s: float,
    t_acc_s: float,
    t_cruise_s: float,
    t_total_s: float,
    d_acc_m: float,
) -> float:
    """
    Posición recorrida sobre el segmento en función del tiempo real.
    """

    t = max(0.0, min(elapsed_s, t_total_s))

    # Aceleración.
    if t <= t_acc_s:
        return min(
            distance_m,
            0.5 * accel_m_s2 * t**2,
        )

    # Velocidad constante.
    t_cruise_end = t_acc_s + t_cruise_s

    if t <= t_cruise_end:
        return min(
            distance_m,
            d_acc_m + v_peak_m_s * (t - t_acc_s),
        )

    # Desaceleración.
    remaining_time = max(0.0, t_total_s - t)

    return min(
        distance_m,
        distance_m
        - 0.5 * accel_m_s2 * remaining_time**2,
    )


async def move_reference_line(
    drone: System,
    start_n: float,
    start_e: float,
    end_n: float,
    end_e: float,
    down_m: float,
    yaw_deg: float,
    speed_m_s: float = SPEED_M_S,
    accel_limit_m_s2: float = HORIZONTAL_ACCEL_LIMIT_M_S2,
):
    dn = end_n - start_n
    de = end_e - start_e
    distance = math.hypot(dn, de)

    if distance < 1e-6:
        await hold_position_for(
            drone,
            end_n,
            end_e,
            down_m,
            yaw_deg,
            WAIT_AFTER_MOVE_S,
        )
        return True

    un = dn / distance
    ue = de / distance

    (
        profile_type,
        v_peak,
        t_acc,
        t_cruise,
        t_total,
        d_acc,
    ) = motion_profile_parameters(
        distance,
        speed_m_s,
        accel_limit_m_s2,
    )

    print(
        f"Moviendo referencia: distancia={distance:.2f} m, "
        f"perfil={profile_type}, "
        f"velocidad máxima={v_peak:.2f} m/s, "
        f"tiempo previsto={t_total:.2f} s"
    )

    dt = 1.0 / OFFBOARD_SETPOINT_RATE_HZ
    motion_start = time.monotonic()

    while True:
        loop_start = time.monotonic()


        elapsed = loop_start - motion_start

        s = distance_from_elapsed_time(
            elapsed_s=elapsed,
            distance_m=distance,
            accel_m_s2=accel_limit_m_s2,
            v_peak_m_s=v_peak,
            t_acc_s=t_acc,
            t_cruise_s=t_cruise,
            t_total_s=t_total,
            d_acc_m=d_acc,
        )

        await send_position_setpoint(
            drone,
            start_n + un * s,
            start_e + ue * s,
            down_m,
            yaw_deg,
        )

        if elapsed >= t_total:
            break

        iteration_elapsed = time.monotonic() - loop_start
        sleep_s = dt - iteration_elapsed

        if sleep_s > 0.0:
            await asyncio.sleep(sleep_s)

    # Se fija explícitamente el vértice exacto.
    await send_position_setpoint(
        drone,
        end_n,
        end_e,
        down_m,
        yaw_deg,
    )

    await hold_position_for(
        drone,
        end_n,
        end_e,
        down_m,
        yaw_deg,
        WAIT_AFTER_MOVE_S,
    )

    return True


async def main():
    drone = System()
    await drone.connect(system_address=SERIAL)
    await wait_connection(drone)

    ready = await wait_health_ready(drone, timeout_s=60.0)

    if not ready:
        print("La estimación local/global no quedó lista a tiempo. No se arma.")
        return

    try:
        print("Configurando altura de despegue...")
        await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)

        print("Armando...")
        await drone.action.arm()

        print("Takeoff usando PX4 interno...")
        await drone.action.takeoff()

        reached = await wait_altitude_timeout(
            drone,
            TAKEOFF_ALT_M,
            tol_m=0.5,
            timeout_s=30.0,
        )

        if not reached:
            print(
                "No se alcanzó la altitud de despegue. "
                "Se cancela la rutina."
            )
            await land_safely(drone)
            return

        print(
            f"Esperando {WAIT_AFTER_TAKEOFF_S:.1f} s "
            "después del despegue..."
        )
        await asyncio.sleep(WAIT_AFTER_TAKEOFF_S)

        local_pos = await get_current_local_position(drone)

        if local_pos is None:
            print("No fue posible obtener posición local NED. Aterrizando.")
            await land_safely(drone)
            return

        n0, e0, d0 = local_pos
        yaw = await get_current_yaw_deg(drone)

        print(
            f"Referencia local: "
            f"N={n0:.2f} m, "
            f"E={e0:.2f} m, "
            f"D={d0:.2f} m, "
            f"yaw={yaw:.1f} deg"
        )

        started = await start_offboard_with_retry(
            drone, n0, e0, d0, yaw
        )

        if not started:
            print(
                "No fue posible entrar a OFFBOARD "
                "después de varios intentos."
            )
            await land_safely(drone)
            return

        # La altura se mantiene mediante la coordenada D local capturada al inicio
        # de OFFBOARD. No se realiza un aterrizaje automático por relative_altitude_m.

        # Vértices del cuadro en marco local NED.
        p0 = (n0, e0)
        p1 = (n0 + SIDE_M, e0)
        p2 = (n0 + SIDE_M, e0 + SIDE_M)
        p3 = (n0, e0 + SIDE_M)
        p4 = (n0, e0)

        print("Orientando a 0 deg respecto al norte...")
        yaw = await yaw_smooth_position(
            drone,
            p0[0],
            p0[1],
            d0,
            yaw,
            0.0,
        )

        print(f"Lado 1: norte, {SIDE_M:.1f} m")
        ok = await move_reference_line(
            drone,
            p0[0],
            p0[1],
            p1[0],
            p1[1],
            d0,
            yaw,
        )

        if not ok:
            print("Movimiento interrumpido por seguridad. Aterrizando.")
            await land_safely(drone)
            return

        print("Orientando a 90 deg...")
        yaw = await yaw_smooth_position(
            drone,
            p1[0],
            p1[1],
            d0,
            yaw,
            90.0,
        )

        print(f"Lado 2: este, {SIDE_M:.1f} m")
        ok = await move_reference_line(
            drone,
            p1[0],
            p1[1],
            p2[0],
            p2[1],
            d0,
            yaw,
        )

        if not ok:
            print("Movimiento interrumpido por seguridad. Aterrizando.")
            await land_safely(drone)
            return

        print("Orientando a 180 deg...")
        yaw = await yaw_smooth_position(
            drone,
            p2[0],
            p2[1],
            d0,
            yaw,
            180.0,
        )

        print(f"Lado 3: sur, {SIDE_M:.1f} m")
        ok = await move_reference_line(
            drone,
            p2[0],
            p2[1],
            p3[0],
            p3[1],
            d0,
            yaw,
        )

        if not ok:
            print("Movimiento interrumpido por seguridad. Aterrizando.")
            await land_safely(drone)
            return

        print("Orientando a 270 deg...")
        yaw = await yaw_smooth_position(
            drone,
            p3[0],
            p3[1],
            d0,
            yaw,
            270.0,
        )

        print(f"Lado 4: oeste, {SIDE_M:.1f} m")
        ok = await move_reference_line(
            drone,
            p3[0],
            p3[1],
            p4[0],
            p4[1],
            d0,
            yaw,
        )

        if not ok:
            print("Movimiento interrumpido por seguridad. Aterrizando.")
            await land_safely(drone)
            return

        print(
            "Cuadro completo. Manteniendo posición final "
            "antes de salir de OFFBOARD..."
        )

        await hold_position_for(
            drone,
            p4[0],
            p4[1],
            d0,
            yaw,
            duration_s=1.0,
        )


        print("Deteniendo OFFBOARD...")
        await stop_offboard_if_needed(drone)

        print("Aterrizando...")
        await drone.action.land()
        await wait_on_ground(drone)

        print("Rutina terminada correctamente.")

    except ActionError as e:
        print(f"PX4 rechazó una acción: {e}")
        await land_safely(drone)

    except OffboardError as e:
        print(f"Error de OFFBOARD: {e}")
        await land_safely(drone)

    except KeyboardInterrupt:
        print("Interrupción manual. Intentando aterrizar...")
        await land_safely(drone)



if __name__ == "__main__":
    asyncio.run(main())
