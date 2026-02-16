#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# type: ignore

import threading
import time
from typing import List, Optional, Tuple

from pymavlink import mavutil, mavwp


CONN = "udp:127.0.0.1:14550"
# Kalkışta önce bu relative irtifaya çıkılacak
TAKEOFF_ALT_M = 3.0
# Mission waypoint irtifası (relative)
WP_ALT_M = 5.0
# Terminale irtifa yazdırma frekansı (Hz)
ALT_PRINT_HZ = 1.0


def wait_heartbeat(master, timeout: float = 20.0):
    # Araca gerçekten bağlandığımızı doğrulamak için ilk heartbeat'i bekliyoruz.
    # Heartbeat gelmeden komut göndermek hatalı target/otopilot durumlarına yol açabilir.
    start = time.time()
    while time.time() - start < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            return hb
    raise TimeoutError("HEARTBEAT gelmedi. UDP bağlantısını/SITL'i kontrol et.")


def command_long(
    master,
    target_system: int,
    target_component: int,
    command: int,
    p1: float = 0,
    p2: float = 0,
    p3: float = 0,
    p4: float = 0,
    p5: float = 0,
    p6: float = 0,
    p7: float = 0,
    timeout: float = 8.0,
):
    # Genel amaçlı COMMAND_LONG gönderici.
    # Tüm parametreleri float'a çevirerek dialect/implementasyon farklılıklarını azaltıyoruz.
    master.mav.command_long_send(
        target_system,
        target_component,
        command,
        0,
        float(p1),
        float(p2),
        float(p3),
        float(p4),
        float(p5),
        float(p6),
        float(p7),
    )

    # Bazı komutlar anında ACCEPTED döner, bazıları IN_PROGRESS dönüp sonradan tamamlanır.
    accepted_results = {
        mavutil.mavlink.MAV_RESULT_ACCEPTED,
        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
    }

    start = time.time()
    while time.time() - start < timeout:
        ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
        if not ack:
            continue
        # Kuyrukta başka komutların ACK'i olabilir; sadece ilgili komutu yakalıyoruz.
        if int(ack.command) != int(command):
            continue
        if int(ack.result) not in accepted_results:
            raise RuntimeError(f"COMMAND_ACK reject: cmd={command}, result={ack.result}")
        return ack

    raise TimeoutError(f"COMMAND_ACK timeout: cmd={command}")


def set_mode(master, target_system: int, mode_name: str, timeout: float = 10.0):
    mapping = master.mode_mapping()
    if not mapping or mode_name not in mapping:
        raise ValueError(f"Mode bulunamadı: {mode_name}. Mevcut modlar: {list(mapping.keys()) if mapping else []}")

    mode_id = mapping[mode_name]
    master.mav.set_mode_send(
        target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    # custom_mode üzerinden doğrulama yapıyoruz.
    # Timeout durumunda kullanıcı isteği gereği sadece uyarı verip devam ediyoruz.
    start = time.time()
    while time.time() - start < timeout:
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb and int(getattr(hb, "custom_mode", -1)) == int(mode_id):
            print(f"[OK] Mod aktif: {mode_name}")
            return

    print(f"[WARN] {mode_name} mode doğrulama timeout. Devam ediliyor.")


def arm(master, target_system: int, target_component: int):
    # ARM işlemi, MAV_CMD_COMPONENT_ARM_DISARM ile yapılır (p1=1 => arm).
    command_long(
        master,
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        p1=1,
    )


def get_best_altitude_once(master) -> Tuple[Optional[float], str]:
    """
    Öncelik:
      1) GLOBAL_POSITION_INT.relative_alt (m)
      2) LOCAL_POSITION_NED altitude = -z
    VFR_HUD kesinlikle kullanılmaz.
    """
    # Non-blocking okuma: anlık olarak kuyrukta ne varsa alır.
    # Böylece hem kontrol döngüleri hem de alt thread akıcı kalır.
    msg = master.recv_match(type=["GLOBAL_POSITION_INT", "LOCAL_POSITION_NED"], blocking=False)
    if not msg:
        return None, "NA"

    mtype = msg.get_type()
    if mtype == "GLOBAL_POSITION_INT":
        return float(msg.relative_alt) / 1000.0, "REL_ALT(GLOBAL)"
    if mtype == "LOCAL_POSITION_NED":
        return float(-msg.z), "ALT(LOCAL_NED)"
    return None, "NA"


def wait_gps_and_ekf(master, timeout: float = 60.0):
    """
    Dialect-safe EKF flag kontrolü (sayısal bit mask):
      ATTITUDE=1, HORIZ_POS_REL=8, HORIZ_POS_ABS=16, VERT_POS=32 
    HORIZONTAL için 8|16 kabul edilir.
    """
    # EKF_STATUS_REPORT.flags alanı için MAVLink common bitleri
    ATTITUDE = 1
    HORIZ_POS_REL = 8
    HORIZ_POS_ABS = 16
    VERT_POS = 32

    gps_ok = False
    ekf_ok = False

    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(
            type=["GPS_RAW_INT", "EKF_STATUS_REPORT", "STATUSTEXT"],
            blocking=True,
            timeout=1,
        )
        if not msg:
            continue

        mtype = msg.get_type()
        if mtype == "GPS_RAW_INT":
            gps_ok = int(getattr(msg, "fix_type", 0)) >= 3
        elif mtype == "EKF_STATUS_REPORT":
            flags = int(getattr(msg, "flags", 0))
            # Yatay pozisyon için REL veya ABS'ten biri yeterlidir.
            has_att = (flags & ATTITUDE) != 0
            has_vert = (flags & VERT_POS) != 0
            has_horiz = (flags & (HORIZ_POS_REL | HORIZ_POS_ABS)) != 0
            ekf_ok = has_att and has_vert and has_horiz

        if gps_ok and ekf_ok:
            print("[OK] GPS 3D fix + EKF hazır.")
            return

    print("[WARN] GPS/EKF doğrulaması timeout. Devam ediliyor.")


def set_message_interval(master, target_system: int, target_component: int, message_id: int, hz: float):
    # MAV_CMD_SET_MESSAGE_INTERVAL: param2 mikro-saniye cinsinden periyottur.
    # Örn 10Hz => 100000us
    interval_us = int(1_000_000 / float(hz))
    command_long(
        master,
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        p1=message_id,
        p2=interval_us,
    )


def send_local_ned_alt_target(master, target_system: int, target_component: int, alt_m: float):
    """
    MAV_FRAME_LOCAL_NED için z aşağı pozitiftir.
    3m yukarı hedef = z = -3.0
    """
    # Sadece pozisyon-z setpoint'i aktif, diğer alanlar ignore.
    # Böylece GUIDED fallback sırasında aracı doğrudan hedef irtifaya zorlayabiliyoruz.
    type_mask = (
        (1 << 0) | (1 << 1) |  # x,y ignore
        (1 << 3) | (1 << 4) | (1 << 5) |  # velocity ignore
        (1 << 6) | (1 << 7) | (1 << 8) |  # accel ignore
        (1 << 10) | (1 << 11)  # yaw / yaw_rate ignore
    )

    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0,
        0.0,
        -float(alt_m),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def robust_takeoff(master, target_system: int, target_component: int, target_alt_m: float):
    """
    1) MAV_CMD_NAV_TAKEOFF ile kalkış başlat
    2) Hedefe çıkış takılırsa SET_POSITION_TARGET_LOCAL_NED z hedefiyle fallback uygula
    """
    print(f"[INFO] Takeoff başlatılıyor: {target_alt_m:.1f} m")
    # 1) Standart takeoff komutu
    command_long(
        master,
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        p7=float(target_alt_m),
    )

    def reached(min_alt: float) -> Tuple[bool, Optional[float], str]:
        # Kısa pencere içinde hedef eşiğe ulaşıldı mı kontrolü.
        # Kaynak bilgisi (GLOBAL/LOCAL) debug ve güven analizi için döndürülür.
        deadline = time.time() + 2.0
        last_alt = None
        last_src = "NA"
        while time.time() < deadline:
            alt, src = get_best_altitude_once(master)
            if alt is not None:
                last_alt = alt
                last_src = src
                if alt >= min_alt:
                    return True, last_alt, last_src
            time.sleep(0.1)
        return False, last_alt, last_src

    # 2) Önce normal takeoff akışına fırsat tanıyoruz.
    hard_deadline = time.time() + 14.0
    while time.time() < hard_deadline:
        ok, alt, src = reached(target_alt_m - 0.2)
        if ok:
            print(f"[OK] Takeoff tamam: {alt:.2f} m ({src})")
            return
        if alt is not None and alt > 0.6:
            continue

    # 3) Takılma varsa fallback: düzenli z-target göndererek yükselmeyi zorluyoruz.
    print("[WARN] Takeoff 3m civarına ulaşamadı, LOCAL_NED fallback uygulanıyor...")
    fallback_deadline = time.time() + 20.0
    while time.time() < fallback_deadline:
        send_local_ned_alt_target(master, target_system, target_component, target_alt_m)
        ok, alt, src = reached(target_alt_m - 0.2)
        if ok:
            print(f"[OK] Fallback ile hedef irtifa: {alt:.2f} m ({src})")
            return
        time.sleep(0.2)

    raise TimeoutError("Takeoff başarısız: relative 3.0m hedefine ulaşılamadı.")


def build_mission_from_user_wps(wps_lat_lon: List[Tuple[float, float]], wp_alt_m: float):
    """
    ArduPilot bazı durumlarda ilk mission item'i anında geçebilir.
    Bu yüzden WP1'in bir kopyasını en başa "guard" olarak ekliyoruz.
    Olası skip guard üzerinde kalır, kullanıcı WP1 yine mutlaka uçulur.

    Mission dizilimi:
      GUARD(WP1) + WP1 + WP2 + WP3 + RTL
    """
    mission_items = []
    seq = 0

    # İlk kullanıcı waypoint'ini kaçırma riskine karşı başa guard kopyası eklenir.
    guarded_wps: List[Tuple[float, float]] = [wps_lat_lon[0]] + list(wps_lat_lon)

    for lat, lon in guarded_wps:
        mission_items.append(
            mavutil.mavlink.MAVLink_mission_item_int_message(
                target_system=0,
                target_component=0,
                seq=seq,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                # current=1 sadece ilk item'da olacak şekilde işaretlenir.
                current=1 if seq == 0 else 0,
                autocontinue=1,
                param1=0.0,
                param2=2.0,
                param3=0.0,
                param4=0.0,
                x=int(lat * 1e7),
                y=int(lon * 1e7),
                z=float(wp_alt_m),
            )
        )
        seq += 1

    mission_items.append(
        mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=0,
            target_component=0,
            seq=seq,
            frame=mavutil.mavlink.MAV_FRAME_MISSION,
            command=mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            current=0,
            autocontinue=1,
            param1=0.0,
            param2=0.0,
            param3=0.0,
            param4=0.0,
            x=0,
            y=0,
            z=0.0,
        )
    )
    return mission_items


def upload_mission(master, target_system: int, target_component: int, mission_items, timeout: float = 30.0):
    """
    clear_all -> count -> request/request_int -> item gönder -> ack
    """
    # MAVWPLoader mission item'ları doğru formatta saklar ve seq erişimini kolaylaştırır.
    loader = mavwp.MAVWPLoader()
    for item in mission_items:
        loader.add(item)

    # 1) Eski mission temizlenir
    master.mav.mission_clear_all_send(target_system, target_component)
    time.sleep(0.2)

    # 2) Yeni mission item sayısı bildirilir
    master.mav.mission_count_send(target_system, target_component, loader.count())

    sent = set()
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(
            type=["MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"],
            blocking=True,
            timeout=1,
        )
        if not msg:
            continue

        mtype = msg.get_type()
        # 3) Araç hangi seq'i isterse onu gönderiyoruz (request-driven upload).
        if mtype in ("MISSION_REQUEST", "MISSION_REQUEST_INT"):
            seq = int(msg.seq)
            if seq in sent:
                continue
            master.mav.send(loader.wp(seq))
            sent.add(seq)
            continue

        # 4) Sonunda ACK beklenir
        if mtype == "MISSION_ACK":
            if int(msg.type) != int(mavutil.mavlink.MAV_MISSION_ACCEPTED):
                raise RuntimeError(f"MISSION_ACK reject: type={msg.type}")
            print(f"[OK] Mission upload tamam. item_count={loader.count()}")
            return

    raise TimeoutError("Mission upload timeout (request/ack akışı tamamlanamadı).")


def check_statustext_fail_after_auto(master, window_s: float = 4.0):
    # AUTO'ya geçişin hemen ardından kritik hata metinlerini yakalıyoruz.
    # Özellikle "init failed" kullanıcı isteğine göre hard fail olmalı.
    deadline = time.time() + window_s
    while time.time() < deadline:
        st = master.recv_match(type="STATUSTEXT", blocking=True, timeout=0.5)
        if not st:
            continue
        text = str(getattr(st, "text", "")).strip()
        low = text.lower()
        if "init failed" in low or " failed" in low or low.endswith("failed"):
            raise RuntimeError(f"AUTO sonrası STATUSTEXT fail: {text}")


def monitor_mission(master, last_seq: int, timeout: float = 300.0):
    print("[INFO] Mission ilerlemesi izleniyor...")
    start = time.time()
    current_seen = -1

    while time.time() - start < timeout:
        msg = master.recv_match(type="MISSION_CURRENT", blocking=True, timeout=1)
        if not msg:
            continue

        seq = int(getattr(msg, "seq", -1))
        if seq != current_seen:
            current_seen = seq
            print(f"[INFO] MISSION_CURRENT: {seq}/{last_seq}")

        # last_seq'e ulaşıldığında mission tamamlandı kabul edilir.
        if seq >= last_seq:
            print("[OK] Mission son item'e ulaştı.")
            return

    print("[WARN] Mission monitor timeout. Araç uçuşunu GCS üzerinden doğrula.")


def altitude_printer_thread(master, stop_event: threading.Event, hz: float = ALT_PRINT_HZ):
    # Arka planda 1Hz (veya verilen hızda) son bilinen irtifayı yazdırır.
    # Kaynak önceliği get_best_altitude_once içinde korunur (GLOBAL -> LOCAL).
    period = 1.0 / float(hz)
    next_tick = time.time()
    last_alt = None
    last_src = "NA"

    while not stop_event.is_set():
        alt, src = get_best_altitude_once(master)
        if alt is not None:
            last_alt = alt
            last_src = src

        now = time.time()
        if now >= next_tick:
            if last_alt is None:
                print("[ALT] NA")
            else:
                print(f"[ALT] {last_alt:.2f} m ({last_src})")
            next_tick = now + period

        time.sleep(0.05)


def parse_waypoint_input(line: str) -> Tuple[float, float]:
    # Kullanıcı hem "lat lon" hem "lat,lon" girebilir.
    # Burada normalize edip doğruluyoruz.
    clean = line.strip().replace(",", " ")
    parts = [p for p in clean.split() if p]
    if len(parts) != 2:
        raise ValueError("Format: lat lon")

    lat = float(parts[0])
    lon = float(parts[1])
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError("Lat/Lon aralığı geçersiz")

    return lat, lon


def ask_user_waypoints() -> List[Tuple[float, float]]:
    print("\n[INPUT] Uçuş başlamadan 3 waypoint girilecek (lat lon veya lat,lon).")
    print("[INPUT] Mission Planner/QGC haritadan kopyalayıp terminale yapıştırabilirsiniz.\n")

    # Uçuş komutu göndermeden önce 3 waypoint toplanır.
    # Böylece uçuş sırasında interactive bekleme yaşanmaz.
    waypoints: List[Tuple[float, float]] = []
    for index in range(1, 4):
        while True:
            raw = input(f"WP{index}: ")
            try:
                lat, lon = parse_waypoint_input(raw)
                waypoints.append((lat, lon))
                break
            except Exception as exc:
                print(f"[ERR] {exc}. Tekrar deneyin.")

    print("\n[OK] Waypointler:")
    for idx, (lat, lon) in enumerate(waypoints, start=1):
        print(f"  WP{idx}: {lat:.7f}, {lon:.7f}")
    print("")

    return waypoints


def resolve_target_ids_from_hb(hb) -> Tuple[int, int]:
    # Bazı ortamlarda component id 0 gelebilir; pratikte autopilot için 1 güvenlidir.
    target_system = int(hb.get_srcSystem())
    target_component = int(hb.get_srcComponent())
    if target_component == 0:
        target_component = 1
    return target_system, target_component


def main():
    # 0) Önce waypointleri al
    user_wps = ask_user_waypoints()

    # 1) Bağlantı + heartbeat
    master = mavutil.mavlink_connection(CONN)
    print("[INFO] Heartbeat bekleniyor...")
    hb = wait_heartbeat(master)
    target_system, target_component = resolve_target_ids_from_hb(hb)
    print(f"[OK] Bağlandı: sys={target_system} comp={target_component}")

    # 2) İstenen telemetry mesajlarını sabitle
    set_message_interval(
        master,
        target_system,
        target_component,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        10,
    )
    set_message_interval(
        master,
        target_system,
        target_component,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        10,
    )
    set_message_interval(
        master,
        target_system,
        target_component,
        mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT,
        2,
    )

    # 3) Arka planda sürekli irtifa çıktısı başlat
    stop_event = threading.Event()
    alt_thread = threading.Thread(
        target=altitude_printer_thread,
        args=(master, stop_event, ALT_PRINT_HZ),
        daemon=True,
    )
    alt_thread.start()

    try:
        # 4) GPS/EKF hazır olmasını bekle (timeout olursa sadece uyarı) 
        wait_gps_and_ekf(master, timeout=60) 

        # 5) GUIDED -> ARM -> robust takeoff
        set_mode(master, target_system, "GUIDED")
        print("[INFO] ARM komutu gönderiliyor...")
        arm(master, target_system, target_component)

        robust_takeoff(master, target_system, target_component, TAKEOFF_ALT_M)

        # 6) Mission oluştur + upload
        mission_items = build_mission_from_user_wps(user_wps, WP_ALT_M)
        upload_mission(master, target_system, target_component, mission_items)

        # 7) AUTO başlat + mission index 0 + fail text kontrolü
        set_mode(master, target_system, "AUTO")
        master.mav.mission_set_current_send(target_system, target_component, 0)
        check_statustext_fail_after_auto(master, window_s=4)

        # 8) Mission ilerlemesini izleme
        monitor_mission(master, last_seq=len(mission_items) - 1, timeout=300)
        print("[DONE] Mission tamamlandı (WP1->WP2->WP3->RTL).")

    finally:
        # Thread'i temiz kapat
        stop_event.set()
        time.sleep(0.2)


if __name__ == "__main__":
    main()
