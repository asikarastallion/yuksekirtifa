#!/usr/bin/env python3
# type: ignore

from pymavlink import mavutil
import time

connection_string = "udp:127.0.0.1:14550"
print(f"[INFO] Connecting to {connection_string}")
master = mavutil.mavlink_connection(connection_string)

print("[INFO] Waiting for heartbeat...")
master.wait_heartbeat()
print("[OK] Heartbeat received")

target_system = master.target_system
target_component = master.target_component
if target_component == 0:
    print("[WARN] COMPID=0 geldi. AUTOPILOT1 (compid=1) hedefliyorum.")
    target_component = 1

print(f"[INFO] SYSID: {target_system}")
print(f"[INFO] COMPID: {target_component}")

def drain_statustext(seconds=1.0):
    t0 = time.time()
    while time.time() - t0 < seconds:
        m = master.recv_match(type="STATUSTEXT", blocking=False)
        if m:
            txt = m.text.decode("utf-8", errors="ignore") if isinstance(m.text, (bytes, bytearray)) else m.text
            print(f"[STATUSTEXT] {txt}")
        time.sleep(0.05)

def set_param(name: str, value: float):
    master.mav.param_set_send(
        target_system, target_component,
        name.encode("utf-8"),
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)

def set_mode(mode_name, timeout=6.0):
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        raise ValueError(f"Unknown mode: {mode_name}. Available: {list(mode_map.keys())}")
    mode_id = mode_map[mode_name]

    master.mav.set_mode_send(
        target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and hb.custom_mode == mode_id:
            print(f"[OK] Mode set to {mode_name}")
            return
    print(f"[WARN] Mode {mode_name} not confirmed via heartbeat")

def arm(timeout=8.0):
    master.mav.command_long_send(
        target_system, target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0,0,0,0,0,0
    )
    print("[INFO] ARM command sent")

    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("[OK] Armed")
            return
        drain_statustext(0.05)
    raise RuntimeError("ARM olmadı (STATUSTEXT'e bak)")

def disarm():
    master.mav.command_long_send(
        target_system, target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0,0,0,0,0,0
    )
    print("[INFO] DISARM command sent")

def takeoff(altitude_m: float):
    master.mav.command_long_send(
        target_system, target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0,0,0,0, 0,0, float(altitude_m)
    )
    print(f"[INFO] Takeoff command: {altitude_m:.2f} m")

def get_global_int(timeout=1.0):
    return master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)

def get_relative_alt_m(timeout=1.0):
    gp = get_global_int(timeout=timeout)
    if gp is None:
        return None
    return gp.relative_alt / 1000.0

def send_global_hold_and_climb(target_rel_alt_m: float):
    gp = get_global_int(timeout=1.0)
    if gp is None:
        return
    lat_int = gp.lat
    lon_int = gp.lon

    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    master.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        target_system, target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        lat_int, lon_int,
        float(target_rel_alt_m),
        0,0,0, 0,0,0, 0,0
    )

def read_user_altitude():
    raw = input("Hedef irtifa (metre): ").strip().replace(",", ".")
    alt = float(raw)
    if alt <= 0.2:
        raise ValueError("En az 0.2 m gir.")
    return alt

try:
    target_alt = read_user_altitude()
    tolerance = 0.3

    # SITL debug (gerçek uçuş değil)
    set_param("FS_THR_ENABLE", 0)
    set_param("FS_GCS_ENABLE", 0)
    set_param("ARMING_CHECK", 0)

    print("[INFO] Waiting 8s for EKF/GPS settle...")
    drain_statustext(8.0)

    set_mode("GUIDED")
    arm()
    time.sleep(1.0)

    takeoff(target_alt)

    # Takeoff sonrası sebep yakalamak için
    print("[INFO] After TAKEOFF, dumping STATUSTEXT for 6s...")
    drain_statustext(6.0)

    print("[INFO] Climbing...")
    while True:
        send_global_hold_and_climb(target_alt)

        rel = get_relative_alt_m(timeout=1.0)
        if rel is not None:
            print(f"[REL_ALT] {rel:.2f} m / target {target_alt:.2f} m")
            if rel >= (target_alt - tolerance):
                print("[OK] Target reached")
                break

        time.sleep(0.2)

    set_mode("LAND")
    print("[INFO] Landing...")
    time.sleep(8.0)
    disarm()

except KeyboardInterrupt:
    print("\n[INFO] Ctrl+C")
    try:
        set_mode("LAND")
    except Exception:
        pass
    try:
        disarm()
    except Exception:
        pass

except Exception as e:
    print(f"[ERROR] {e}")
    drain_statustext(5.0)
