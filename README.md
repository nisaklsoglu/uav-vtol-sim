# Çoklu Ajanlı VTOL Simülasyon ve Otonom Görev Altyapısı

![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%20(GZ%20Sim%208)-orange)
![ArduPilot](https://img.shields.io/badge/ArduPilot-SITL-blue)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble-22314E)
![MAVROS](https://img.shields.io/badge/MAVROS-ROS%202-green)

Bu depo, **Gazebo Harmonic (GZ Sim 8)** üzerinde iki bağımsız VTOL (QuadPlane) modelinin fizik simülasyonunu, ArduPilot SITL otopilot süreçlerini, ROS 2 Humble üzerindeki bağımsız MAVROS telemetri/kontrol köprülerini ve **SUAS 2027 & TEKNOFEST** isterlerine göre geliştirilmiş otonom görev yöneticisini (`multi_vtol_commander`) içerir.

Sistem, iki İHA'nın birbirinden bağımsız durum makineleri (state machine) olarak uçmasını sağlayarak otonom rota takibi, engel ve hava sahası ayrımı, dinamik hedef kesmesi (target interrupt), faydalı yük bırakma (payload drop) ve otonom `QRTL` iniş senaryolarının geliştirilmesi ve test edilmesi için uçtan uca bir ortam sunar.

## İçindekiler

1. [Mimari ve Ağ Yapısı](#1-mimari-ve-ağ-yapısı)
2. [Otonom Görev Mimarisi ve Tasarım Kararları](#2-otonom-görev-mimarisi-multi_vtol_commander-ve-tasarım-kararları)
3. [Senaryolar (Cases) ve Konfigürasyon Yapısı](#3-senaryolar-cases-ve-konfigürasyon-yapısı)
4. [Ön Koşullar ve Bağımlılıklar](#4-ön-koşullar-ve-bağımlılıklar)
5. [Kurulum, Derleme ve Çalıştırma](#5-kurulum-derleme-ve-çalıştırma)
6. [Senaryo Bazlı Test Adımları (Case 1, Case 2, Case 3)](#6-senaryo-bazlı-test-adımları-case-1-case-2-case-3)
7. [ROS 2 Topic ve Servis Arayüzleri](#7-ros-2-topic-ve-servis-arayüzleri)
8. [Simülasyonu Durdurma ve Süreç Temizliği](#8-simülasyonu-durdurma-ve-süreç-temizliği)
9. [Sorun Giderme](#9-sorun-giderme)
10. [Kamera Entegrasyonu ve Ek Dokümantasyon](#10-kamera-entegrasyonu-ve-ek-dokümantasyon)

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
   -> NAVIGATING (Çoklu Tur Rota Takibi + İsteğe Bağlı NFZ Teğet Kaçınması)
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

ArduPlane GUIDED modda hedef noktayı bir loiter (yörünge) merkezi olarak yorumlar ve dönüş yarıçapı (varsayılan `WP_LOITER_RAD`) nedeniyle merkeze ~50 metreden fazla yaklaşmadan teğet geçer. Yarıçapın dar tutulması araçların waypoint veya kaçınma noktası etrafında dönmesine yol açtığından, akıcı geçiş için `wp_radius_m: 55.0` yapılandırılmıştır.

#### Neden statik NFZ kaçınması ve İHA ayrımı (Inter-UAV Separation) eklendi?

- **Statik No-Fly Zone (NFZ) Teğet Kaçınması:** Rota segmenti tanımlı bir yasaklı silindir bölgenin (`obstacles`) güvenlik mesafesine (`radius + 20.0 m`) yaklaştığında, doğrudan engelin içine girmek yerine engelin dışına doğru teğet bir kaçış noktası (`detour_wp`) üretilir. Nokta tamamlandığında (`prev_wp_enu` güncellenerek) ana waypoint'e geri dönülür.
- **İHA Ayrımı (`_check_inter_uav_separation`):** `vtol_1` ve `vtol_2` arasındaki yatay mesafe **45.0 m** altına ve dikey fark **12.0 m** altına indiğinde `vtol_2` güvenli bir irtifaya (`max(alt_1, target_alt_2) + 15.0 m`) tırmanır; yatay mesafe **65.0 m** üzerine çıktığında normal seyir irtifasına döner.

#### Neden dinamik hedef kesmesi (Target Interrupt — Task 4 & Task 5) eklendi?

Yarışmada İHA arama rotasında uçarken yerdeki hedef (dağcı/valiz) tespit edildiğinde, aracın turunu bitirmesini beklemeden anında rotayı kesip hedefe yönelmesi gerekir. `/<ns>/target_gps` topic'ine `NavSatFix` mesajı geldiğinde araç:

1. `NAVIGATING` fazını duraklatır,
2. SUAS Task 5 kuralı gereği en az **150 ft AGL (≥ 48.0 m)** güvenli irtifada hedefe yaklaşır (`INTERCEPT_TARGET`),
3. `DROP_RADIUS_M` yarıçapı içinde faydalı yükü bırakır (`PAYLOAD_DROP`),
4. Kaldığı waypoint'ten ana rotasına devam eder.

#### Neden iniş sonrası (`MISSION_COMPLETE`) mod `GUIDED` yapılıyor?

Araç `QRTL` ile dikey iniş yapıp disarm olduktan sonra `QRTL` modunda kalırsa otopilot periyodik olarak `PreArm: QRTL mode not armable` uyarısı üretir. İniş doğrulandığında (`alt < 0.4 m` veya disarmed) otopilot tek seferlik `GUIDED` moduna çekilerek sistem temiz bir bekleme durumuna alınır.

---

## 3. Senaryolar (Cases) ve Konfigürasyon Yapısı

Testlerin birbirini ezmemesi ve her görevin izole şekilde çalıştırılabilmesi için senaryo ayarları `config/cases/` altında ayrı JSON dosyalarına bölünmüştür ve `multi_vtol_commander` düğümündeki `case` parametresi ile seçilir:

| Senaryo Parametresi | Dosya (`config/cases/`) | `enable_nfz` | `enable_separation` | `drop_radius_m` | Amaç |
|---|---|---|---|---|---|
| `case1` (`case1_navigation`) | `case1_navigation.json` | `false` | `false` | `15.0` | Saf rota takibi (2 tur), XTE ölçümü ve QRTL iniş testi |
| `case2` (`case2_obstacles_separation`) | `case2_obstacles_separation.json` | `true` | `true` | `15.0` | Statik NFZ (`NFZ_1`) teğet dolanma ve `vtol_1`–`vtol_2` hava sahası ayrımı |
| `case3` (`case3_target_drop`) | `case3_target_drop.json` | `false` | `false` | `35.0` | Dinamik `/target_gps` kesmesi, 150 ft AGL irtifa kontrolü ve yük bırakma testi |
| `case4` (`case4_target_drop`) | `case4_target_drop.json` | `false` | `false` | — | Balistik lead/hız hesabı ile hedefe yaklaşma ve faydalı yük bırakma testi |

> **Not:** `setup.py` içerisinde `config/cases/*` dosyaları `share/<package_name>/config/cases` altına açıkça kurulacak şekilde tanımlanmıştır; böylece `colcon build` sırasında klasör kopyalama hatası oluşmaz.

---

## 4. Ön Koşullar ve Bağımlılıklar

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

## 5. Kurulum, Derleme ve Çalıştırma

### Adım 1: Paketi derleme ve ortamı hazırlama

```bash
cd /home/uav/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select vtol_bringup --symlink-install
source install/setup.bash
export GZ_IP=127.0.0.1
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uav/ros2_ws/install/vtol_bringup/share/vtol_bringup/config/fastdds_udp_loopback.xml
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

İkinci terminalde varsayılan konfigürasyon (`suas_mission.json`) veya istenen `case` parametresi ile düğümü başlatın:

```bash
source /opt/ros/humble/setup.bash
source /home/uav/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uav/ros2_ws/install/vtol_bringup/share/vtol_bringup/config/fastdds_udp_loopback.xml

ros2 run vtol_bringup multi_vtol_commander --ros-args -p case:=case2
```

---

## 6. Senaryo Bazlı Test Adımları (Case 1, Case 2, Case 3)

### 6.1. Case 1 — Rota Takibi ve XTE Doğrulama

```bash
ros2 run vtol_bringup multi_vtol_commander --ros-args -p case:=case1
```

**Beklenen çıktı:** İki araç kalkış ve sabit kanat geçişinden sonra `[Tur 1/2 | WP 1/4]` ... `WP 4/4` sırasıyla 2 tur tamamlar, ardından `QRTL` moduna geçip inişi bitirir.

### 6.2. Case 2 — Statik NFZ Kaçınması ve Inter-UAV Separation

```bash
ros2 run vtol_bringup multi_vtol_commander --ros-args -p case:=case2
```

**Beklenen çıktı (NFZ):** `vtol_1` yasaklı bölgeye yaklaştığında şu loglar görülür:

- `[NFZ KAÇINMA] NFZ_1 tespit edildi! Teğet noktaya yöneliniyor...`
- `[NFZ KAÇINMA] Engel geçildi (...) -> Ana WP ... hedefine dönülüyor.`

**Beklenen çıktı (Separation):** İki araç kritik yatay/dikey mesafe sınırının altına indiğinde şu loglar görülür:

- `[SEPARATION] vtol_1 - vtol_2 yakınlaşma! ... -> vtol_2 ...m irtifaya tırmanıyor.`
- `[SEPARATION] Güvenli ayrım sağlandı (...) -> vtol_2 normal irtifaya ... dönüyor.`

### 6.3. Case 3 — Hedef Kesmesi ve Faydalı Yük Bırakma Testi (Task 4 & Task 5)

Terminal 2'de `case3` ile commander'ı başlatın:

```bash
ros2 run vtol_bringup multi_vtol_commander --ros-args -p case:=case3
```

Araçlar `NAVIGATING` fazındayken üçüncü terminalden Home GPS (`lat=-35.363262`, `lon=149.165237`) yakınında bir hedef koordinatı yayınlayın:

```bash
source /opt/ros/humble/setup.bash
source /home/uav/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uav/ros2_ws/install/vtol_bringup/share/vtol_bringup/config/fastdds_udp_loopback.xml

ros2 topic pub --once /vtol_1/target_gps sensor_msgs/msg/NavSatFix "{latitude: -35.3624, longitude: 149.1652, altitude: 50.0}"
```

**Beklenen çıktı:**

1. `[HEDEF TESPİT EDİLDİ!]` ile rotanın kesilmesi,
2. `[HEDEF YAKLAŞMA - TASK 5]` ile `>= 48.0 m` (150 ft AGL) irtifada yaklaşma,
3. Hedef yarıçapına girildiğinde `FAYDALI YÜK BIRAKILIYOR!`,
4. 2 saniye sonra kalınan waypoint'ten rotaya devam.



### 6.4. Case 4 — Balistik Hedef Bırakma Testi

Terminal 2'de `case4` ile commander'ı başlatın:

```bash
ros2 run vtol_bringup multi_vtol_commander --ros-args -p case:=case4
```
`vtol_1` `NAVIGATING` fazındayken üçüncü terminalden hedef koordinatı yayınlayın:

```bash
source /opt/ros/humble/setup.bash
source /home/uav/ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/uav/ros2_ws/install/vtol_bringup/share/vtol_bringup/config/fastdds_udp_loopback.xml

ros2 topic pub --times 5 /vtol_1/target_gps sensor_msgs/msg/NavSatFix "{latitude: -35.3624, longitude: 149.1652, altitude: 50.0}"
```

---

## 7. ROS 2 Topic ve Servis Arayüzleri

### 7.1. Temel Topic'ler

| İşlev | VTOL 1 | VTOL 2 | Mesaj Türü |
|---|---|---|---|
| Otopilot durumu | `/vtol_1/mavros/state` | `/vtol_2/mavros/state` | `mavros_msgs/msg/State` |
| Yerel konum / odometri | `/vtol_1/mavros/global_position/local` | `/vtol_2/mavros/global_position/local` | `nav_msgs/msg/Odometry` |
| Küresel GPS konumu | `/vtol_1/mavros/global_position/global` | `/vtol_2/mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` |
| Hedef tespit kesmesi | `/vtol_1/target_gps` | `/vtol_2/target_gps` | `sensor_msgs/msg/NavSatFix` |

### 7.2. Temel Servisler

| İşlev | VTOL 1 | VTOL 2 | Mesaj Türü |
|---|---|---|---|
| Arm / Disarm | `/vtol_1/mavros/cmd/arming` | `/vtol_2/mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` |
| Uçuş modu değiştirme | `/vtol_1/mavros/set_mode` | `/vtol_2/mavros/set_mode` | `mavros_msgs/srv/SetMode` |
| Dikey kalkış (Takeoff) | `/vtol_1/mavros/cmd/takeoff` | `/vtol_2/mavros/cmd/takeoff` | `mavros_msgs/srv/CommandTOL` |
| Komut gönderimi (`DO_REPOSITION`) | `/vtol_1/mavros/cmd/command_int` | `/vtol_2/mavros/cmd/command_int` | `mavros_msgs/srv/CommandInt` |

---

## 8. Simülasyonu Durdurma ve Süreç Temizliği

Simülasyon ve commander terminallerinde `Ctrl + C` kullandıktan sonra arka planda kalan süreçleri temizlemek için:

```bash
pkill -9 -f gz || true
pkill -9 -f arduplane || true
pkill -9 -f sim_vehicle || true
pkill -9 -f mavros || true
pkill -9 -f multi_vtol_commander || true
```

---

## 9. Sorun Giderme

| Hata / Belirti | Kontrol Edilmesi Gerekenler |
|---|---|
| `can't copy 'config/cases'` (colcon build) | `setup.py` içinde `config/cases/*` dosyalarının `share/<pkg>/config/cases` altına ayrı listelendiğini ve `config/*` içinde alt klasör bulunmadığını doğrulayın. |
| `Connection refused` / `link down` | SITL'in 12 saniyelik başlangıç süresinin dolduğunu ve TCP portlarının (`5762`, `5773`) dinlendiğini doğrulayın. |
| Topic'ler terminaller arası görünmüyor | Her terminalde `FASTRTPS_DEFAULT_PROFILES_FILE` değişkeninin `fastdds_udp_loopback.xml` dosyasına işaret ettiğinden emin olun. |
| Araç waypoint veya kaçınma noktası etrafında dönüp ilerlemiyor | `wp_radius_m` değerinin `55.0` olduğunu ve değişiklik sonrası `colcon build` alındığını kontrol edin. |
| Hedef testinde mesafe binlerce km görünüyor | `/vtol_1/target_gps` mesajında Home GPS (`-35.3632, 149.1652`) çevresinde bir koordinat gönderdiğinizden emin olun. |
| `Address already in use` | Bölüm 8'deki süreç temizleme komutlarını çalıştırıp eski SITL/Gazebo süreçlerini sonlandırın. |

---

## 10. Kamera Entegrasyonu ve Ek Dokümantasyon

- [Kamera Dokümantasyonu](docs/camera.md)
- [Çoklu VTOL Dokümantasyonu](docs/multi_vtol.md)
