# 🚀 AŞAMA 2 — Uçuş Modları ve Mantık (ArduCopter)
**Yüksek İrtifa Serisi — Video Notları**

Bu bölümün hedefi: ArduCopter’te uçuş modlarının **hangi kontrol katmanlarını** (attitude/altitude/position) devreye aldığını, **hangi sensörlere** dayandığını ve **pilot vs autopilot** sınırını netleştirmek.  
Sonunda da bunları SITL üzerinde **Python (pymavlink)** ile pratikte göstereceğiz:  
- GUIDED modda **kalkış** (otonom komut)  
- AUTO modda **waypoint görevi** (mission upload + start)

---

## 0) Kontrol katmanları
ArduCopter kontrolü pratikte katmanlıdır:

- **Rate Control (iç döngü)**: Gövde açısal hızlarını (p, q, r) tutar. (Gyro ağırlıklı)
- **Attitude / Angle Control (dış döngü)**: Roll/Pitch/Yaw açılarını tutar. (IMU + filtre)
- **Altitude Control (Z)**: İrtifayı tutar / değiştirir. (Baro + IMU, bazen rangefinder)
- **Position Control (XY)**: Konumu tutar / hedefe gider. (GPS + EKF + IMU)

Bir uçuş modu, bu katmanların bazılarını açıp bazılarını kapatır.

---

# 🔵 1) Flight Mode’lar

Aşağıda her mod için 3 şeyi anlatıyoruz:
1) **Hangi kontrol katmanı aktif?**  
2) **Hangi sensöre bağımlı?**  
3) **Pilot vs autopilot sınırı** (pilot neyi “doğrudan” kontrol eder?)

---

## 1. STABILIZE
### Aktif kontrol katmanı
- ✅ **Rate + Attitude** aktif
- ❌ Position/Altitude tutma yok

### Sensör bağımlılığı
- ✅ IMU (gyro + accel)

### Pilot vs autopilot
- Pilot **roll/pitch/yaw** komut verir (araç açıları stabil tutar)
- Pilot **throttle’ı doğrudan** yönetir (yükseklik tutma yok)

**Ne zaman?** Manüel uçuş eğitimi, temel kontrol.

---

## 2. ALT_HOLD
### Aktif kontrol katmanı
- ✅ Rate + Attitude
- ✅ **Altitude control (Z)** aktif
- ❌ Position hold yok (XY pilotta)

### Sensör bağımlılığı
- ✅ Barometre (ana)
- ✅ IMU (destek)
- (Varsa rangefinder daha iyi iniş/hover)

### Pilot vs autopilot
- Pilot roll/pitch/yaw ile yatay hareketi yönetir
- Throttle stick “gaz” değil, **irtifa isteği** gibi davranır

**Ne zaman?** Hover, daha konforlu manüel uçuş.

---

## 3. LOITER
### Aktif kontrol katmanı
- ✅ Rate + Attitude
- ✅ Altitude control
- ✅ **Position control (XY)** aktif

### Sensör bağımlılığı
- ✅ GPS + EKF (konum)
- ✅ IMU + Baro

### Pilot vs autopilot
- Stick bırakınca araç **konumu korur**
- Pilot stick ile hedef konumu “sürükler” gibi komut verir

**Ne zaman?** Konum sabitleme, yavaş manevralar, güvenli bekleme.

---

## 4. GUIDED
### Aktif kontrol katmanı
- ✅ Rate + Attitude
- ✅ Altitude control
- ✅ Position/Velocity control (komuta göre)

### Sensör bağımlılığı
- ✅ GPS + EKF (konum/ hız komutları için)
- ✅ IMU + Baro

### Pilot vs autopilot
- Komutlar pilot stick’ten değil **MAVLink üzerinden dışarıdan** gelir:
  - “Kalk”, “Şu noktaya git”, “Şu hızla ilerle” gibi

**Ne zaman?** Python ile otonom komut, dış bilgisayar kontrolü, basit görevler.

---

## 5. AUTO
### Aktif kontrol katmanı
- ✅ Rate + Attitude
- ✅ Altitude control
- ✅ Position control
- ✅ **Mission state machine** (görev motoru)

### Sensör bağımlılığı
- ✅ GPS + EKF (görev/waypoint için kritik)
- ✅ IMU + Baro

### Pilot vs autopilot
- Araç, yüklenen görev planını **kendisi yürütür**
- Pilot genelde sadece “görevi iptal et / RTL” gibi müdahale eder

**Ne zaman?** Waypoint görevi, otomatik rota uçuşu, senaryo çalışmaları.

---

## 6. RTL (Return To Launch)
### Aktif kontrol katmanı
- ✅ Position + Altitude control
- ✅ RTL state machine (yükseğe çık → eve dön → iniş/loiter)

### Sensör bağımlılığı
- ✅ GPS + EKF

### Pilot vs autopilot
- Pilot devre dışı, araç eve dönmeye öncelik verir

**Ne zaman?** Güvenli dönüş, fail-safe senaryoları.

---

## 7. LAND
### Aktif kontrol katmanı
- ✅ Kontrollü alçalma (altitude descent)
- Yatay kontrol mod/sensöre göre değişebilir

### Sensör bağımlılığı
- ✅ Baro + IMU
- (Rangefinder varsa daha iyi)

### Pilot vs autopilot
- Mode’a göre değişir; LAND’de iniş önceliklidir

---

# 🔵 2) GUIDED Mode Detay

## simple_goto nedir?
“Şu konuma git” yaklaşımıdır. Genelde daha yüksek seviyeli yardımcı fonksiyon/komut akışıyla yapılır.  
Pratikte MAVLink tarafında “pozisyon hedefi” veya “mission benzeri komut” olarak düşün.

## set_position_target_local_ned
En profesyonel yöntemlerden biri. Araca **yerel koordinat sisteminde** hedef verirsin.

## velocity command
Araca “şu hızla ilerle” dersin. Özellikle takip/kaçınma gibi işlerde kullanılır.

## NED frame nedir?
**N**orth – **E**ast – **D**own (Kuzey–Doğu–Aşağı).  
Önemli: NED’de **Z aşağı pozitiftir**. Yukarı çıkmak için Z **negatif** olur.

---

# 🔵 3) AUTO Mode (Waypoint Görevi)

## Mission upload
Araç hafızasına waypoint listesini gönderme işlemidir. MAVLink’te MISSION_* mesajları ile yapılır.

## Mission start
Genelde aracı **AUTO** moda alıp “görev yürüt” dediğinde başlar.

## Mission item nedir?
Görevdeki tek bir adım. Örn:
- TAKEOFF
- WAYPOINT
- LOITER_TIME
- RTL
- LAND

## Waypoint parametreleri
Komuta göre değişir ama tipik olarak:
- Latitude / Longitude (veya yerel koordinat)
- Altitude
- Acceptance radius (kabul yarıçapı)
- Hold time (bekleme)

## Mission state machine mantığı
AUTO modu bir “durum makinesi” gibi çalışır:
- Current item → “tamamlandı mı?” kontrolü → next item
- Gerekirse bekler, hız/irtifa ayarlar, sonra sonraki adıma geçer

---

# ✅ Bu videonun demo planı 

1) **GUIDED**: Arm + Takeoff (3 m)  
2) **AUTO**: 3 waypoint’li basit görev yükle + başlat  
3) Görev bitince **RTL** veya **LAND**

---

# 🧪 Ön koşullar (kısaca)
- Gazebo açık (iris world)
- SITL + MAVProxy açık (gazebo-iris)
- Python script: UDP 14550’ye bağlanır


