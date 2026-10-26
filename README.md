# 🚁 ArduPilot SITL – VTOL Simulation Environment

Apple Silicon (ARM64) ve Ubuntu üzerinde native olarak çalışan, **ArduPilot SITL (ArduPlane – QuadPlane)**, **Gazebo Harmonic (GZ Sim 8)** ve **ROS 2 Humble** tabanlı otonom VTOL İHA simülasyon ortamı.

Bu proje, dikey kalkış ve iniş (VTOL), sabit kanatlı ileri uçuş, uçuş modu geçişleri ve ROS 2 üzerinden sensör verilerinin aktarılması gibi temel otonom uçuş senaryolarını simüle etmek amacıyla geliştirilmiştir.

## 🛠️ Mimari ve Bileşenler

| Bileşen | Teknoloji |
|---|---|
| İşletim Sistemi | Ubuntu 22.04 LTS (`linux/arm64`) |
| Robotik Middleware | ROS 2 Humble Hawksbill |
| Fizik Simülatörü | Gazebo Harmonic (GZ Sim 8.x) |
| Fizik Motoru | DART Physics |
| Uçuş Kontrol Yazılımı | ArduPilot SITL – ArduPlane |
| İHA Konfigürasyonu | QuadPlane VTOL |
| VTOL Modeli | `alti_transition_quad` |
| Gazebo – ArduPilot Haberleşmesi | `ardupilot_gazebo` JSON/Lockstep UDP |
| Yer Kontrol ve Telemetri | MAVProxy |
| ROS 2 – Gazebo Köprüsü | `ros_gz_bridge` |

## 📋 Gereksinimler

- Ubuntu 22.04 LTS (`linux/arm64`)
- Docker Engine
- Docker Compose
- Git
- Apple Silicon tabanlı bir Mac veya ARM64 Ubuntu ortamı

> **Not:** ROS 2 Humble, Gazebo Harmonic ve `ardupilot_gazebo` eklentisinin aynı ortamda uyumlu çalışması için konteyner imajının ilgili bağımlılıkları doğru şekilde yapılandırılmış olmalıdır.

## 🚀 Kurulum

### 1. Depoyu Klonlama ve Konteyneri Başlatma

```bash
git clone <REPO_URL>
cd uav-vtol-sim
docker compose up -d
```

Konteynerin çalıştığını kontrol edin:

```bash
docker ps
```

Konteyner adı `vtol_sim` olarak yapılandırılmışsa aşağıdaki komutla terminale bağlanabilirsiniz:

```bash
docker exec -it vtol_sim bash
```

### 2. VTOL Modelini İndirme

İlk kurulumda resmi ArduPilot SITL_Models deposundan `alti_transition_quad` modelini indirin.

Konteynere girdikten sonra:

```bash
cd /home/uav/ardupilot_gazebo/models

git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/ArduPilot/SITL_Models.git sitl_models_tmp

cd sitl_models_tmp

git sparse-checkout set Gazebo/models/alti_transition_quad

cp -r Gazebo/models/alti_transition_quad \
  /home/uav/ardupilot_gazebo/models/

cd /home/uav/ardupilot_gazebo/models

rm -rf sitl_models_tmp
```

Bu işlem, gerekli VTOL modelini `ardupilot_gazebo/models` dizinine yerleştirir.

### 3. QuadPlane Pist Dünyasını Oluşturma

Mevcut `iris_runway.sdf` dünyasını temel alarak VTOL modelini kullanacak yeni bir dünya dosyası oluşturun:

```bash
cd /home/uav/ardupilot_gazebo/worlds

sed 's|model://iris_with_gimbal|model://alti_transition_quad|g' \
  iris_runway.sdf > quadplane_runway.sdf
```

Oluşturulan dünya dosyası:

```text
/home/uav/ardupilot_gazebo/worlds/quadplane_runway.sdf
```

> **Önemli:** Bu işlem yalnızca model URI değerlerini değiştirir. Dünya dosyasının geri kalanında modelin bağlantı noktaları, sensörleri ve eklenti yapılandırmalarıyla ilgili ek değişiklikler gerekebilir.

## 🛫 Simülasyonu Çalıştırma

Simülasyonun doğru başlatılması için Gazebo ve ArduPilot SITL ayrı terminallerde çalıştırılmalıdır.

### Terminal 1 – Gazebo Harmonic

Öncelikle konteynere bağlanın:

```bash
docker exec -it vtol_sim bash
```

Gazebo model ve eklenti yollarını tanımlayın:

```bash
export GZ_SIM_RESOURCE_PATH=/home/uav/ardupilot_gazebo/models:/home/uav/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/uav/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

**Headless (GUI olmadan) çalıştırma:**

```bash
gz sim -v 4 -r -s \
  /home/uav/ardupilot_gazebo/worlds/quadplane_runway.sdf
```

**Grafik arayüzüyle çalıştırma:**

Host ekran erişimi ve grafik arayüzü yönlendirmesi yapılandırıldıktan sonra:

```bash
gz sim -v 4 -r \
  /home/uav/ardupilot_gazebo/worlds/quadplane_runway.sdf
```

### Terminal 2 – ArduPilot SITL ve MAVProxy

İkinci bir terminal açın:

```bash
docker exec -it vtol_sim bash
```

ArduPlane dizinine geçin:

```bash
cd /home/uav/ardupilot/ArduPlane
```

SITL simülasyonunu başlatın:

```bash
DISPLAY= sim_vehicle.py \
  -v ArduPlane \
  -f quadplane \
  --model JSON:127.0.0.1 \
  --no-rebuild
```

Bu komut, QuadPlane konfigürasyonunu kullanarak ArduPilot SITL'yi başlatır ve Gazebo ile JSON tabanlı bağlantı kurmayı hedefler.

Başlatma tamamlandığında MAVProxy konsolu üzerinden uçuş modları ve RC kanalları kontrol edilebilir.

## 🎮 Uçuş Testleri

Aşağıdaki komutlar MAVProxy (`MAV>`) konsolunda çalıştırılır.

### 1. Dikey Kalkış – QLOITER

Dikey kalkış ve konum koruma moduna geçin:

```text
mode QLOITER
arm throttle
rc 3 1700
```

İstenen irtifaya ulaşıldığında gaz kanalını nötr seviyeye yaklaştırın:

```text
rc 3 1500
```

### 2. Sabit Kanatlı İleri Uçuş – FBWA

İleri uçuş moduna geçin:

```text
mode FBWA
rc 3 1750
```

Bu aşamada araç, uygun uçuş koşulları sağlandığında dikey uçuş konfigürasyonundan sabit kanatlı ileri uçuşa geçmeyi hedefler.

> **Not:** Geçiş davranışı; hava hızı, uçuş kontrol parametreleri, servo ve motor eşleştirmeleri ile QuadPlane konfigürasyonuna bağlıdır. Dikey motorların kapanması ve itici motorun devreye girmesi her konfigürasyonda yalnızca bu komutlarla garanti edilmez.

### 3. Dikey İniş – QLAND

Dikey iniş moduna geçin:

```text
mode QLAND
```

Araç inişini tamamladıktan ve motorlar güvenli biçimde durduktan sonra, gerekli koşullar sağlanıyorsa:

```text
rc 3 1000
mode MANUAL
disarm
```

> **Güvenlik:** RC kanal değerleri simülasyon yapılandırmasına göre değişebilir. Mod geçişlerini ve motor davranışlarını doğrulamadan bu komutları gerçek bir araçta kullanmayın.

## 🔌 ROS 2 Entegrasyonu

Gazebo'da üretilen IMU ve odometri verilerini ROS 2 ekosistemine aktarmak için `ros_gz_bridge` kullanılabilir.

### 1. ROS 2 Ortamını Hazırlama

Yeni bir terminalde konteynere bağlanın:

```bash
docker exec -it vtol_sim bash
```

ROS 2 Humble ortamını etkinleştirin:

```bash
source /opt/ros/humble/setup.bash
```

### 2. IMU ve Odometri Köprüsünü Başlatma

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/iris_runway/model/alti_transition_quad/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /model/alti_transition_quad/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

Bu komut, belirtilen Gazebo topic'lerini ilgili ROS 2 mesaj türleriyle eşleştirmeyi amaçlar.

**Önemli:** Topic isimleri, gerçek dünyadaki model ve sensör isimleriyle birebir eşleşmelidir. Yukarıdaki topic yolları örnek yapılandırmadaki isimlere bağlıdır; farklı bir SDF dosyasında değişiklik gerekebilir.

### 3. Topic'leri Kontrol Etme

ROS 2 topic listesini görüntüleyin:

```bash
ros2 topic list
```

IMU topic'inin verilerini kontrol edin:

```bash
ros2 topic echo /world/iris_runway/model/alti_transition_quad/link/imu_link/sensor/imu_sensor/imu
```

Odometri topic'ini kontrol etmek için:

```bash
ros2 topic echo /model/alti_transition_quad/odometry
```

Topic isimlerini, kendi Gazebo ve ROS 2 yapılandırmanızla karşılaştırarak doğrulayın.

## 🧪 Temel Test Senaryoları

| Test | Amaç |
|---|---|
| Gazebo başlatma | Fizik simülasyonunun doğru çalıştığını doğrulamak |
| SITL bağlantısı | ArduPilot ile Gazebo arasındaki JSON bağlantısını kontrol etmek |
| Dikey kalkış | QLOITER modunda kontrollü kalkışı test etmek |
| İleri uçuş | FBWA moduna geçişi değerlendirmek |
| Dikey iniş | QLAND modundaki iniş davranışını test etmek |
| IMU aktarımı | Gazebo sensör verilerinin ROS 2'ye ulaştığını doğrulamak |
| Odometri aktarımı | Konum ve hareket bilgilerinin ROS 2'ye ulaştığını doğrulamak |

## 🐛 Sorun Giderme

### Gazebo modeli bulunamıyor

Model ve dünya yollarını kontrol edin:

```bash
echo "$GZ_SIM_RESOURCE_PATH"
```

Gerekirse model yolunu yeniden tanımlayın:

```bash
export GZ_SIM_RESOURCE_PATH=/home/uav/ardupilot_gazebo/models:/home/uav/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

### SITL, Gazebo'ya bağlanamıyor

- Gazebo simülasyonunun çalıştığını kontrol edin.
- `ardupilot_gazebo` sistem eklentisinin doğru yüklendiğini doğrulayın.
- JSON/UDP bağlantı adresini ve port yapılandırmasını kontrol edin.
- Gazebo ve SITL'nin aynı ağ yapılandırmasında erişilebilir olduğundan emin olun.

### ROS 2 topic'leri görünmüyor

- `source /opt/ros/humble/setup.bash` komutunun çalıştırıldığını doğrulayın.
- `ros_gz_bridge` paketinin kurulu olduğunu kontrol edin.
- Gazebo topic isimlerini doğrulayın.
- Bridge komutundaki Gazebo ve ROS 2 mesaj türlerinin eşleştiğinden emin olun.

## 📁 Proje Dizin Yapısı

Beklenen temel dizin yapısı:

```text
uav-vtol-sim/
├── docker-compose.yml
├── README.md
└── ...
```

Konteyner içindeki ilgili dizinler:

```text
/home/uav/
├── ardupilot/
│   └── ArduPlane/
└── ardupilot_gazebo/
    ├── models/
    │   └── alti_transition_quad/
    ├── worlds/
    │   ├── iris_runway.sdf
    │   └── quadplane_runway.sdf
    └── build/
```

## 📚 Referanslar

- [ArduPilot Documentation](https://ardupilot.org/)
- [ArduPilot SITL_Models](https://github.com/ArduPilot/SITL_Models)
- [ArduPilot Gazebo Plugin](https://github.com/ArduPilot/ardupilot_gazebo)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ros_gz Repository](https://github.com/gazebosim/ros_gz)

## 📄 Lisans

Bu projenin lisansı, proje deposunda belirtilen lisans koşullarına tabidir. Kullanılan üçüncü taraf yazılımlar kendi lisansları kapsamında dağıtılmaktadır.
