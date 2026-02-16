# ArduPilot SITL & MAVLink Temelleri (Drone / ArduCopter)

## Video 1 -- Detaylı Teknik Notlar

------------------------------------------------------------------------

# 1) SITL Mimarisi

## SITL (Software In The Loop) Nedir?

SITL, ArduPilot yazılımının gerçek uçuş kartı olmadan bilgisayarda
çalıştırılan versiyonudur.

Gerçek sistemde:

Sensörler → Flight Controller → ESC → Motor

SITL'de:

Simüle edilmiş sensörler → ArduPilot binary → MAVLink → GCS

Yani fiziksel donanım yerine yazılım simülasyonu kullanılır.

------------------------------------------------------------------------

## sim_vehicle.py Ne Yapar?

`sim_vehicle.py`, ArduPilot'un simülasyon başlatma scriptidir.

Yaptıkları:

1.  Gerekirse firmware derler (waf build)
2.  SITL binary'yi başlatır (ör: arducopter)
3.  MAVLink portlarını ayarlar
4.  MAVProxy'yi başlatabilir
5.  Harita ve console açabilir

Yani tek komutla tüm sistemi ayağa kaldırır.

------------------------------------------------------------------------

## MAVProxy'nin Rolü

MAVProxy:

-   MAVLink router
-   Komut arayüzü
-   Telemetry görüntüleyici
-   Çoklu bağlantı yönlendirici

Bağlantı yapısı genelde:

SITL (TCP 5760) → MAVProxy → UDP 14550 → Python / GCS

------------------------------------------------------------------------

## Port 5760 Nedir?

SITL'in varsayılan TCP portudur.

MAVProxy genelde buraya master olarak bağlanır.

tcp:127.0.0.1:5760

------------------------------------------------------------------------

## Port 14550 Nedir?

Genellikle dış dünyaya yayınlanan MAVLink UDP portudur.

Python scriptler veya QGC genelde buraya bağlanır.

udp:127.0.0.1:14550

------------------------------------------------------------------------

# 2) MAVLink Bağlantı Yapısı

## Heartbeat Nedir?

Araçtan düzenli gelen "hayattayım" mesajıdır.

İçerir:

-   Mode bilgisi
-   Armed durumu
-   Araç tipi
-   Sistem durumu

Bağlantı kurarken ilk adım: heartbeat beklemek.

------------------------------------------------------------------------

## SYSID ve COMPID

https://mavlink.io/en/messages/common.html#MAV_COMPONENT

SYSID: Araç kimliği (genelde 1)\
COMPID: Bileşen kimliği (genelde 1 autopilot)

Birden fazla araç varsa her biri farklı SYSID kullanır.

| Bileşen          | COMPID |
| ---------------- | ------ |
| Autopilot        | 1      |
| Camera           | 100    |
| Gimbal           | 154    |
| GPS              | 220    |
| Onboard Computer | 191    |
| Telemetry Radio  | 192    |
| Servo Output     | 140    |


------------------------------------------------------------------------

## Message Rate Nedir?

Bir mesajın saniyedeki gönderim sayısıdır (Hz).

Örnek:

-   ATTITUDE → 10 Hz
-   HEARTBEAT → 1 Hz
-   GLOBAL_POSITION_INT → 5 Hz

Rate fazla olursa sistem yükü artar.

------------------------------------------------------------------------

# 3) Python ile Bağlantı (pymavlink)



#!/usr/bin/env python3

from pymavlink import mavutil
import time

# ---------------------------------------------------------
# 1) BAĞLANTI
# ---------------------------------------------------------
connection_string = "udp:127.0.0.1:14550"
print(f"[INFO] Connecting to {connection_string}")
master = mavutil.mavlink_connection(connection_string)

print("[INFO] Waiting for heartbeat...")
master.wait_heartbeat()
print("[OK] Heartbeat received")

target_system = master.target_system
target_component = master.target_component
print(f"[INFO] SYSID: {target_system}")
print(f"[INFO] COMPID: {target_component}")

# (İsteğe bağlı) bazı stream'leri daha düzenli almak için:
master.mav.request_data_stream_send(
    target_system, target_component,
    mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1
)

# ---------------------------------------------------------
# 2) MODE DEĞİŞTİRME (ACK'li)
# ---------------------------------------------------------
def set_mode(mode_name, timeout=5.0):
    mode_map = master.mode_mapping()
    if mode_name not in mode_map:
        raise ValueError(f"Unknown mode: {mode_name}. Available: {list(mode_map.keys())}")

    mode_id = mode_map[mode_name]

    master.mav.set_mode_send(
        target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # ACK bekle (kalite için)
    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is None:
            continue
        # custom_mode ArduPilot'ta mode_id ile uyumlu olur
        if hb.custom_mode == mode_id:
            print(f"[OK] Mode set to {mode_name}")
            return

    print(f"[WARN] Mode change to {mode_name} not confirmed via HEARTBEAT (devam ediyorum).")

# ---------------------------------------------------------
# 3) ARM / DISARM (ACK'li)
# ---------------------------------------------------------
def arm(timeout=5.0):
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )
    print("[INFO] ARM command sent")

    t0 = time.time()
    while time.time() - t0 < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb is None:
            continue
        if (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0:
            print("[OK] Armed")
            return
    print("[WARN] ARM not confirmed via HEARTBEAT (devam ediyorum).")

def disarm():
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("[INFO] DISARM command sent")

# ---------------------------------------------------------
# 4) TAKEOFF
# ---------------------------------------------------------
def takeoff(altitude_m):
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0,
        0, 0,
        float(altitude_m)
    )
    print(f"[INFO] Takeoff command: {altitude_m:.2f} m")

# ---------------------------------------------------------
# 5) YÜKSEKLİK / SETPOINT
# ---------------------------------------------------------
def get_relative_alt(timeout=1.0):
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=timeout)
    if msg is None:
        return None
    return msg.relative_alt / 1000.0  # mm -> m

def send_altitude_setpoint(target_alt_m):
    """
    LOCAL_NED: z axis 'down' (+) so upward is negative z.
    target_alt_m (up) => z = -target_alt_m
    """
    z_down = -float(target_alt_m)

    # type_mask: ignore everything except position x,y,z
    # Bits: ignore vx,vy,vz, ax,ay,az, yaw, yaw_rate
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

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,   # time_boot_ms (yaklaşık)
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0, 0.0, z_down,   # x, y, z
        0.0, 0.0, 0.0,      # vx, vy, vz (ignored)
        0.0, 0.0, 0.0,      # ax, ay, az (ignored)
        0.0, 0.0            # yaw, yaw_rate (ignored)
    )

# ---------------------------------------------------------
# ANA
# ---------------------------------------------------------
def read_user_altitude():
    raw = input("Hedef irtifa (metre): ").strip().replace(",", ".")
    alt = float(raw)
    if alt <= 0.2:
        raise ValueError("Hedef irtifa çok küçük. En az 0.2 m gir.")
    if alt > 50:
        print("[WARN] 50m üzeri girdin, SITL için bile gereksiz olabilir.")
    return alt

try:
    target_alt = read_user_altitude()
    tolerance = 0.15  # metre (istersen 0.2 yap)

    set_mode("GUIDED")
    arm()
    time.sleep(1)

    takeoff(target_alt)
    print("[INFO] Climbing with GUIDED altitude setpoints...")

    # setpoint'i düzenli göndermek, 2.66'da takılma gibi durumları genelde çözer
    while True:
        # 5 Hz setpoint
        send_altitude_setpoint(target_alt)

        alt = get_relative_alt(timeout=1.0)
        if alt is not None:
            print(f"[ALTITUDE] {alt:.2f} m / target {target_alt:.2f} m")

            if alt >= (target_alt - tolerance):
                print("[OK] Target altitude reached (within tolerance)")
                break

        time.sleep(0.2)

    time.sleep(2)

    set_mode("LAND")
    print("[INFO] Landing...")
    time.sleep(8)
    disarm()

except Exception as e:
    print(f"[ERROR] {e}")
