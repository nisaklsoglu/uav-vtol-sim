# Çoklu Ajanlı VTOL Simülasyon ve Otonom Görev Altyapısı

![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%20(GZ%20Sim%208)-orange)
![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)
![MAVROS](https://img.shields.io/badge/MAVROS-ROS%202-green)

Bu depo, **Gazebo Harmonic (GZ Sim 8)** üzerinde iki bağımsız VTOL (QuadPlane) modelinin fizik simülasyonunu, ArduPilot SITL otopilot süreçlerini, ROS 2 Humble üzerindeki bağımsız MAVROS telemetri/kontrol köprülerini ve **SUAS 2027 & TEKNOFEST** isterlerine göre geliştirilmiş otonom görev yöneticisini (`multi_vtol_commander`) içerir.

Sistem, iki İHA'nın birbirinden bağımsız durum makineleri (state machine) olarak uçmasını sağlayarak otonom rota takibi, dinamik hedef kesmesi (target interrupt), faydalı yük bırakma (payload drop) ve otonom `QRTL` iniş senaryolarının geliştirilmesi ve test edilmesi için uçtan uca bir ortam sunar.

## İçindekiler

1. [Mimari ve Ağ Yapısı](#1-mimari-ve-ağ-yapısı)
2. [Otonom Görev Mimarisi ve Tasarım Kararları](#2-otonom-görev-mimarisi-multi_vtol_commander-ve-tasarım-kararları)
3. [Ön Koşullar ve Bağımlılıklar](#3-ön-koşullar-ve-bağımlılıklar)
4. [Kurulum, Derleme ve Çalıştırma](#4-kurulum-derleme-ve-çalıştırma)
5. [Hedef Kesmesi ve Faydalı Yük Bırakma Testi](#5-hedef-kesmesi-ve-faydalı-yük-bırakma-testi-task-4--task-5)
6. [ROS 2 Topic ve Servis Arayüzleri](#6-ros-2-topic-ve-servis-arayüzleri)
7. [Simülasyonu Durdurma ve Süreç Temizliği](#7-simülasyonu-durdurma-ve-süreç-temizliği)
8. [Sorun Giderme](#8-sorun-giderme)
9. [Ek Dokümantasyon](#9-kamera-entegrasyonu-ve-ek-dokümantasyon)

---

## 1. Mimari ve Ağ Yapısı

Her iki İHA da bağımsız ArduPilot SITL instance'ları olarak çalıştırılır. MAVROS köprüleri, TCP bağlantıları üzerinden ilgili otopilotlarla iletişim kurarak telemetri ve kontrol arayüzlerini ROS 2 ekosistemine aktarır.

### İHA Yapılandırması

| Parametre | VTOL 1 | VTOL 2 |
|---|---|---|
| Gazebo model adı | `vtol_1` | `vtol_2` |
| SITL instance | `-I0` | `-I1` |
| Gazebo JSON portu | `9002` | `9012` |
| MAVROS bağlantı URL'i | `tcp://127.0.0.1:5762` | `tcp://127.0.0.1:5773` |
| Target System ID | `1` | `1` |
| ROS 2 namespace | `/vtol_1` | `/vtol_2` |
| Kalkış / Seyir irtifası | `20.0 m` / `46.0 m` | `30.0 m` / `60.0 m` |

### Başlatma Sıralaması ve FastDDS Loopback

ArduPilot SITL'in TCP sunucusunu başlatması ve EKF sensör hizalamasını tamamlaması yaklaşık **10–12 saniye** sürebilir. `Connection refused` ve `link down` gibi bağlantı hatalarını önlemek amacıyla MAVROS düğümleri, launch dosyasında `TimerAction` kullanılarak **12 saniye** gecikmeli başlatılır.

Konteyner içi düğümler arası topic ve servis keşfinin kararlı çalışması için `FASTRTPS_DEFAULT_PROFILES_FILE` üzerinden UDP loopback profili (`fastdds_udp_loopback.xml`) kullanılır.

---

## 2. Otonom Görev Mimarisi (`multi_vtol_commander`) ve Tasarım Kararları

`vtol_bringup` paketi içerisindeki `multi_vtol_commander` düğümü, `vtol_1` ve `vtol_2` araçlarını birbirinden bağımsız `VehicleCommander` durum makineleri olarak yönetir.

### 2.1. Görev Yaşam Döngüsü (State Machine)

Her araç aşağıdaki fazları otonom olarak yürütür:

```text
WAIT_FCU -> SET_GUIDED -> ARM -> SPOOL_UP -> TAKEOFF -> WAIT_ALTITUDE
   -> NAVIGATING (Çoklu Tur Rota Takibi)
   -> [Hedef Tespiti] INTERCEPT_TARGET -> PAYLOAD_DROP -> NAVIGATING (Kaldığı WP'den Devam)
   -> RTL_LAND (QRTL) -> WAIT_LANDED -> MISSION_COMPLETE (GUIDED Bekleme Modu)
```

### 2.2. Tasarım Kararları

#### Neden çoklu ajan (multi-agent) simülasyon kullanılıyor?

- **Hava sahası gerçekçiliği:** Yarışma sahasında yalnızca kendi İHA'mız değil, diğer takımların İHA'ları da aynı anda havada bulunur. İkinci ajan (`vtol_2`), farklı irtifa ve rota katmanında uçarak sahadaki diğer hava araçlarını ve çoklu hava sahası trafiğini simüle eder.
- **Görev paralelliği:** İki farklı aracın aynı anda farklı irtifalarda ve rotalarda test edilebilmesi, geliştirme ve doğrulama süresini kısaltır.

#### Neden `MAV_CMD_DO_REPOSITION` (tek atımlık komut) kullanılıyor?

ArduPlane sabit kanat (Fixed-Wing) modunda sürekli setpoint basılması, otopilotun VTOL-FW geçiş (transition) durumunu sıfırlamasına yol açar. Bu nedenle her waypoint geçişinde tek seferlik `MAV_CMD_DO_REPOSITION` (192) komutu gönderilir.

#### Neden waypoint kabul yarıçapı (`wp_radius_m`) 55.0 m olarak belirlendi?

ArduPlane GUIDED modda hedef noktayı bir loiter (yörünge) merkezi olarak yorumlar ve dönüş yarıçapı (varsayılan `WP_LOITER_RAD`) nedeniyle merkeze ~50 metreden fazla yaklaşmadan teğet geçer. Yarıçapın 35.0 m gibi dar tutulması araçların waypoint etrafında sonsuz döngüye girmesine yol açtığından, akıcı ve kesintisiz tur geçişi için `suas_mission.json` içerisinde `wp_radius_m: 55.0` yapılandırılmıştır.

#### Neden dinamik hedef kesmesi (Target Interrupt — Task 4 & Task 5) eklendi?

Yarışmada İHA arama rotasında uçarken kamera/görüntü işleme hattı yerdeki hedefi (dağcı/valiz) tespit ettiğinde, aracın turunu bitirmesini beklemeden anında rotayı kesip hedefe yönelmesi gerekir.

`/<ns>/target_gps` topic'ine `NavSatFix` mesajı geldiğinde araç:

1. `NAVIGATING` fazını duraklatır,
2. SUAS Task 5 kuralı gereği en az **150 ft AGL (≥ 48.0 m)** güvenli irtifada hedefe yaklaşır (`INTERCEPT_TARGET`),
3. `DROP_RADIUS_M` (≤ 15 m / 50 ft) yarıçapı içinde faydalı yükü bırakır (`PAYLOAD_DROP`),
4. Kaldığı waypoint'ten ana rotasına devam eder.

#### Neden iniş sonrası (`MISSION_COMPLETE`) mod GUIDED yapılıyor?

Araç `QRTL` ile dikey iniş yapıp disarm olduktan sonra `QRTL` modunda kalırsa otopilot periyodik olarak `PreArm: QRTL mode not armable` uyarısı üretir. İniş doğrulandığında (`alt < 0.4 m` veya disarmed) otopilot tek seferlik `GUIDED` moduna çekilerek sistem temiz bir bekleme durumuna alınır.

---

## 3. Ön Koşullar ve Bağımlılıklar

Simülasyonu çalıştırmadan önce Docker konteynerinde aşağıdaki bileşenlerin kurulu ve yapılandırılmış olduğundan emin olun:

- Gazebo Harmonic (GZ Sim 8)
- ArduPilot SITL
- ROS 2 Humble
- MAVROS (`ros-humble-mavros`, `ros-humble-mavros-msgs`)
- GeographicLib veri setleri
- `colcon` derleme araçları
- `vtol_bringup` ROS 2 paketi

### MAVROS kurulumu

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-mavros ros-humble-mavros-msgs
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

---

## 4. Kurulum, Derleme ve Çalıştırma

Simülasyon ortamı, `vtol_sim` isimli Docker konteyneri içerisinde çalıştırılır.

### Adım 1: Konteynere bağlanma ve paketi derleme

```bash
docker exec -it vtol_sim bash
cd /home/uav/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vtol_bringup --symlink-install
source install/setup.bash
```

### Adım 2: Simülasyonu başlatma (Terminal 1)

```bash
source /opt/ros/humble/setup.bash
source /home/uav/ros2_ws/install/setup.bash
export GZ_IP=127.0.0.1
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uav/ros2_ws/install/vtol_bringup/share/vtol_bringup/config/fastdds_udp_loopback.xml

ros2 launch vtol_bringup sim.launch.py
```

**Doğrulama:** Gazebo penceresi açılır, iki VTOL uçağı pistte görünür ve yaklaşık 12 saniye sonra MAVROS bağlantıları kurulur.

### Adım 3: Otonom görev yöneticisini başlatma (Terminal 2)

Yeni bir terminal sekmesi açın (konteyner içindeyseniz doğrudan devam edin):

```bash
source /opt/ros/humble/setup.bash
source /home/uav/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uav/ros2_ws/install/vtol_bringup/share/vtol_bringup/config/fastdds_udp_loopback.xml

ros2 run vtol_bringup multi_vtol_commander
```

**Doğrulama:**

- Her iki araç `GUIDED` moda geçer, ARM olur ve hedef kalkış irtifalarına (20 m ve 30 m) tırmanır.
- Sabit kanat (Fixed-Wing) geçişi tamamlandıktan sonra `[Tur 1/2 | WP 1/4]` loglarıyla rota takibi başlar.
- 2 tur tamamlandığında `QRTL` aktif olur ve araçlar kalkış noktalarına dikey iniş yaparak `=== OTONOM İNİŞ TAMAMLANDI! GÖREV BAŞARIYLA BİTTİ ===` mesajını basar.

---

## 5. Hedef Kesmesi ve Faydalı Yük Bırakma Testi (Task 4 & Task 5)

Uçaklar havada rota turundayken (`[Tur 1/2 | WP ...]` logları akarken), görüntü işleme düğümünden gelen hedef tespitini simüle etmek için üçüncü bir terminal açıp `/vtol_1/target_gps` topic'ine koordinat yayınlayabilirsiniz.

> [!IMPORTANT]
> Koordinatın simülasyon sahasına (Home GPS: `lat=-35.363262`, `lon=149.165237`) yakın olması gerekir. Dünya dışı veya binlerce kilometre uzak koordinatlar uçağın görev sahasından uzaklaşmasına neden olur.

```bash
source /opt/ros/humble/setup.bash
source /home/uav/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uav/ros2_ws/install/vtol_bringup/share/vtol_bringup/config/fastdds_udp_loopback.xml

ros2 topic pub --once /vtol_1/target_gps sensor_msgs/msg/NavSatFix "{latitude: -35.3624, longitude: 149.1652, altitude: 50.0}"
```

### Doğrulama Adımları (Terminal 2 logları)

1. `[vtol_1] [HEDEF TESPİT EDİLDİ!] Rota kesiliyor -> Dağcı/Hedef koordinatına yöneliniyor...` uyarısı görünür.
2. `[HEDEF YAKLAŞMA - TASK 5]` satırlarında hedefe mesafenin azaldığı ve irtifanın `>= 48.0 m` (150 ft AGL) korunduğu izlenir.
3. Hedef yarıçapına girildiğinde `>>> HEDEF ÜZERİNDE! ... -> FAYDALI YÜK BIRAKILIYOR! <<<` mesajı basılır.
4. 2 saniye sonra `Faydalı yük başarıyla bırakıldı! Kaldığı yerden (WP X) rotaya devam ediliyor...` logu ile araç normal rota turuna geri döner.

---

## 6. ROS 2 Topic ve Servis Arayüzleri

### 6.1. Temel Topic'ler

| İşlev | VTOL 1 | VTOL 2 | Mesaj Türü |
|---|---|---|---|
| Otopilot durumu | `/vtol_1/mavros/state` | `/vtol_2/mavros/state` | `mavros_msgs/msg/State` |
| Yerel konum / odometri | `/vtol_1/mavros/global_position/local` | `/vtol_2/mavros/global_position/local` | `nav_msgs/msg/Odometry` |
| Küresel GPS konumu | `/vtol_1/mavros/global_position/global` | `/vtol_2/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` |
| Hedef tespit kesmesi | `/vtol_1/target_gps` | `/vtol_2/target_gps` | `sensor_msgs/msg/NavSatFix` |

### 6.2. Temel Servisler

| İşlev | VTOL 1 | VTOL 2 | Mesaj Türü |
|---|---|---|---|
| Arm / Disarm | `/vtol_1/mavros/cmd/arming` | `/vtol_2/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` |
| Uçuş modu değiştirme | `/vtol_1/mavros/set_mode` | `/vtol_2/mavros/set_mode` | `mavros_msgs/srv/SetMode` |
| Dikey kalkış (Takeoff) | `/vtol_1/mavros/cmd/takeoff` | `/vtol_2/mavros/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` |
| Komut gönderimi (`DO_REPOSITION`) | `/vtol_1/mavros/cmd/command_int` | `/vtol_2/mavros/cmd/command_int` | `mavros_msgs/srv/CommandInt` |

---

## 7. Simülasyonu Durdurma ve Süreç Temizliği

Simülasyon ve commander terminallerinde `Ctrl + C` kullandıktan sonra arka planda kalan süreçleri temizlemek için:

```bash
pkill -9 -f gz || true
pkill -9 -f arduplane || true
pkill -9 -f sim_vehicle || true
pkill -9 -f mavros || true
pkill -9 -f multi_vtol_commander || true
```

---

## 8. Sorun Giderme

| Hata / Belirti | Kontrol Edilmesi Gerekenler |
|---|---|
| `Connection refused` / `link down` | SITL'in 12 saniyelik başlangıç süresinin dolduğunu ve TCP portlarının (`5762`, `5773`) dinlendiğini doğrulayın. |
| Topic'ler terminaller arası görünmüyor | Her terminalde `FASTRTPS_DEFAULT_PROFILES_FILE` değişkeninin `fastdds_udp_loopback.xml` dosyasına işaret ettiğinden emin olun. |
| Araç waypoint etrafında dönüp ilerlemiyor | `suas_mission.json` içindeki `wp_radius_m` değerinin `55.0` olduğunu ve değişiklik sonrası `colcon build` alındığını kontrol edin. |
| Hedef testinde mesafe binlerce km görünüyor | `/vtol_1/target_gps` mesajında Home GPS (`-35.3632`, `149.1652`) çevresinde bir koordinat gönderdiğinizden emin olun. |
| `Address already in use` | [Bölüm 7](#7-simülasyonu-durdurma-ve-süreç-temizliği)'deki süreç temizleme komutlarını çalıştırıp eski SITL/Gazebo süreçlerini sonlandırın. |

---

## 9. Kamera Entegrasyonu ve Ek Dokümantasyon

- [Kamera Dokümantasyonu](docs/camera.md)
- [Çoklu VTOL Dokümantasyonu](docs/multi_vtol.md)