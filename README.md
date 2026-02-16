🚀 Yüksek İrtifa — ArduPilot & Otonom Uçuş Serisi -- https://www.youtube.com/@yuksekirtifaa

Bu repo, SITL → Gazebo → MAVLink → Python → Otonomi zincirini
adım adım, mühendislik temelli bir yaklaşımla öğrenmek isteyenler için hazırlanmıştır.

Amaç:

ArduPilot mimarisini anlamak

MAVLink protokolünü kavramak

Python ile otonom kontrol yazmak

Simülasyondan profesyonel seviyeye ilerlemek

Bu repo, YouTube kanalındaki eğitim serisinin teknik arşividir.

🧭 Yol Haritası
🔵 AŞAMA 1 — Sistem ve Bağlantı Temeli
1. SITL mimarisini anlamak

sim_vehicle.py ne yapar?

MAVProxy rolü nedir?

Port 14550 nedir?

TCP 5760 nedir?

UDP vs TCP farkı

MAVLink routing mantığı

2. MAVLink bağlantı yapısı

Heartbeat nedir?

SYSID / COMPID nedir?

Message rate nedir?

MAVLink paket yapısı (header + payload + CRC)

MAVLink v1 vs v2

3. Python ile bağlantı

wait_heartbeat()

Mode değiştirme

arm/disarm

Komut gönderme (COMMAND_LONG)

✔ Bu aşamada sadece bağlantı + temel komut.

🔵 AŞAMA 2 — Uçuş Modları ve Mantık
4. Flight mode’lar

STABILIZE

ALT_HOLD

LOITER

GUIDED

AUTO

RTL

LAND

Her mod için:

Hangi kontrol katmanı aktif?

Hangi sensöre bağlı?

Pilot vs autopilot kontrol sınırı

5. GUIDED Mode detay

simple_goto

set_position_target_local_ned

velocity command

NED frame nedir?

6. AUTO Mode

Mission upload

Mission start

Mission item

Waypoint parametreleri

Mission state machine mantığı

🔵 AŞAMA 3 — Uçuş Kontrol Katmanları
7. Attitude kontrol

Roll / Pitch / Yaw nedir?

Body frame vs Earth frame

Rate loop vs Angle loop

Inner loop / Outer loop mantığı

8. Throttle & Altitude

Throttle control mantığı

ALT_HOLD nasıl çalışır?

Barometre etkisi

Vertical speed control

9. Position control

GPS tabanlı kontrol

EKF rolü

Local position vs Global position

Drift neden olur?

🔵 AŞAMA 4 — Parametre ve PID
10. PID parametreleri

ATC_RAT_RLL_P

ATC_RAT_PIT_P

PSC_POSXY_P

PSC_POSZ_P

Feedforward kavramı

11. Parametre okuma & yazma

param show

param set

Python ile param değiştirme

Param persistence

12. PID tuning simülasyonu

Overshoot

Oscillation

Settling time

Step response analizi

🔵 AŞAMA 5 — Telemetry & Veri
13. Telemetry okuma

GLOBAL_POSITION_INT

ATTITUDE

VFR_HUD

HIGHRES_IMU

14. Log analizi

DataFlash log

MAVProxy log

CSV çıkarma

ArduPilot log message tipleri

15. Gerçek zamanlı veri işleme

Python ile telemetry logger yazma

Matplotlib ile grafik

Basit filtreleme (moving average)

🔵 AŞAMA 6 — Otonomi
16. Otomatik kalkış

Arm

GUIDED

Takeoff

Hedef irtifa kilitleme

17. Waypoint görevi

3 nokta görev

Geri dönüş

Land

18. Dinamik görev değiştirme

Uçuş sırasında mission update

Guided override

19. Velocity control

Sabit hızda ileri git

Yaw kontrolü

Body frame velocity

🔵 AŞAMA 7 — Güvenlik ve Fail-Safe
20. RTL davranışı

Batarya düşük

RC kaybı

GPS kaybı

21. EKF fail-safe

Innovation failure

Position lost

Mode fallback

22. Link kesilmesi

MAVLink drop

Telemetry loss handling

23. Geofence

Soft vs hard fence

Fence breach davranışı

🔵 AŞAMA 8 — Fizik ve Simülasyon (Gelişmiş)
24. Wind ekleme

Gazebo rüzgar

Drift analizi

25. Sensor noise

IMU noise

GPS jitter

Noise modelleme

26. Delay simülasyonu

Telemetry delay

Command latency

Network simulation

🔵 AŞAMA 9 — Gelişmiş Kontrol
27. Offboard control

Sürekli velocity komut

Position override

28. Custom controller (Python tarafında)

Basit PID yazımı

Dış kontrolcü tasarımı

29. Basit obstacle avoidance mantığı

Mesafe kontrolü

Reaktif kaçınma

🔵 AŞAMA 10 — Profesyonel Seviye
30. Çoklu araç simülasyonu

2 drone aynı anda

SYSID yönetimi

31. Companion computer mantığı

Raspberry Pi / SBC entegrasyonu

MAVLink bridge

32. MAVLink router

Router mantığı

Çoklu GCS

UDP routing

🎯 Bu Repo Kime Hitap Ediyor?

ArduPilot öğrenmek isteyenlere

Otonom uçuş yazmak isteyenlere

MAVLink protokolünü anlamak isteyenlere

Simülasyon üzerinden mühendislik yapmak isteyenlere

⚙️ Kullanılan Teknolojiler

ArduPilot

SITL

Gazebo Harmonic

MAVProxy

pymavlink

Python 3