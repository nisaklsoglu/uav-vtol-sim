# UAV VTOL Simulation Environment

Apple Silicon (ARM64) mimarisinde yerel olarak çalışan **ArduPilot SITL (ArduPlane – QuadPlane)**, **Gazebo Harmonic (GZ Sim 8)** ve **ROS 2 Humble** tabanlı otonom VTOL (Vertical Take-Off and Landing) İHA simülasyon ortamı.

Bu proje, dikey kalkış ve iniş yapabilen, sabit kanatlı uçuş moduna geçiş gerçekleştiren bir VTOL İHA'nın simülasyon ortamında test edilmesini sağlar. Fizik simülasyonu, uçuş kontrolü, telemetri ve ROS 2 sensör entegrasyonu Docker tabanlı bir ortamda bir araya getirilmiştir.

## 🛠️ Mimari ve Teknolojiler

| Bileşen | Teknoloji |
|---|---|
| İşletim Sistemi | Ubuntu 22.04 LTS (`linux/arm64`) |
| Konteynerleştirme | Docker & Docker Compose |
| Robotik Platform | ROS 2 Humble Hawksbill |
| Fizik Motoru ve Simülatör | Gazebo Harmonic (GZ Sim 8.x), DART Physics |
| Uçuş Kontrol Yazılımı | ArduPilot SITL – ArduPlane Stable |
| İHA Konfigürasyonu | QuadPlane VTOL |
| Gazebo Haberleşmesi | `ardupilot_gazebo` JSON/Lockstep UDP köprüsü |
| Telemetri ve GCS | MAVProxy |
| ROS 2 Entegrasyonu | `ros_gz_bridge` |

## ✨ Temel Özellikler

- **ARM64 uyumluluğu:** Apple Silicon işlemcili Mac bilgisayarlarda Docker üzerinden çalıştırılmak üzere tasarlanmıştır.
- **VTOL simülasyonu:** Dikey kalkış, sabit kanatlı uçuşa geçiş ve dikey iniş senaryoları.
- **ArduPilot entegrasyonu:** ArduPlane SITL üzerinden uçuş kontrolü.
- **Gazebo fizik simülasyonu:** İHA dinamiğinin ve sensörlerin simülasyonu.
- **Telemetri kontrolü:** MAVProxy konsolu üzerinden uçuş modu, motor ve RC komutlarının yönetimi.
- **ROS 2 entegrasyonu:** IMU ve odometri gibi simülasyon verilerinin ROS 2 topic'lerine aktarılması.
- **Docker tabanlı kurulum:** Gerekli yazılım bileşenlerinin izole bir ortamda çalıştırılması.

## 📋 Gereksinimler

Simülasyonu çalıştırmadan önce aşağıdaki araçların kurulu olması gerekir:

- Apple Silicon işlemcili macOS bilgisayar
- Docker Desktop (ARM64 desteği etkin)
- Git
- Projenin Docker yapılandırma dosyaları ve gerekli kaynak kodları

## 🚀 Kurulum

### 1. Depoyu Klonlama

Terminali açın ve projeyi klonlayın:

```bash
git clone <REPO_URL>
cd uav-vtol-sim
```

`<REPO_URL>` yerine Git deposunun adresini yazın.

### 2. Docker Konteynerini Derleme ve Başlatma

Docker imajını oluşturun ve servisleri arka planda başlatın:

```bash
docker compose up -d --build
```

Çalışan konteynerleri kontrol edin:

```bash
docker ps
```

Bu kılavuzdaki sonraki komutlar, Compose yapılandırmasının `vtol_sim` adında bir konteyner oluşturduğunu varsayar.

## 🛫 Simülasyonu Çalıştırma

Simülasyonun temel çalışma akışı iki ana bileşenden oluşur:

1. Gazebo Harmonic fizik simülasyonu
2. ArduPilot SITL ve MAVProxy uçuş kontrol sistemi

Her iki bileşeni ayrı terminal oturumlarında başlatın.

### 1. Gazebo Harmonic Dünyasını Başlatma

**Terminal 1:** Konteynere bağlanın:

```bash
docker exec -it vtol_sim bash
```

Gazebo model, dünya ve sistem eklentisi yollarını tanımlayın:

```bash
export GZ_SIM_RESOURCE_PATH=/home/uav/ardupilot_gazebo/models:/home/uav/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH

export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/uav/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
```

Gazebo'yu QuadPlane pist dünyasıyla, sunucu modunda başlatın:

```bash
gz sim -v 4 -r -s /home/uav/ardupilot_gazebo/worlds/quadplane_runway.sdf
```

**Komut parametreleri:**

- `-v 4`: Ayrıntılı log çıktısı sağlar.
- `-r`: Simülasyonu başlatır.
- `-s`: Gazebo'yu sunucu modunda çalıştırır; grafik arayüzü açmaz.

Terminali açık bırakın. Gazebo sunucusu çalışmaya devam etmelidir.

### 2. ArduPilot SITL ve MAVProxy'yi Başlatma

**Terminal 2:** macOS terminalinde yeni bir sekme açın ve konteynere bağlanın:

```bash
docker exec -it vtol_sim bash
```

ArduPlane dizinine geçin:

```bash
cd /home/uav/ardupilot/ArduPlane
```

SITL ve MAVProxy'yi başlatın:

```bash
DISPLAY= sim_vehicle.py -v ArduPlane -f quadplane --model JSON:127.0.0.1 --no-rebuild
```

**Komut parametreleri:**

- `-v ArduPlane`: ArduPlane uçuş kontrol yazılımını seçer.
- `-f quadplane`: QuadPlane VTOL hava aracı konfigürasyonunu kullanır.
- `--model JSON:127.0.0.1`: Gazebo ile JSON tabanlı model arayüzü üzerinden iletişim kurar.
- `--no-rebuild`: SITL yazılımını yeniden derlemeden çalıştırmayı dener.
- `DISPLAY=`: Grafik ekran değişkenini boş olarak ayarlar.

Başlatma işlemi tamamlandığında MAVProxy konsolu açılır. GPS ve EKF başlatma mesajlarını takip edin. Uçuş komutlarına başlamadan önce simülasyonun, sensörlerin ve tahmini konum bilgisinin hazır olduğunu doğrulayın.

## 🎮 Temel Uçuş Testi

Aşağıdaki komutlar MAVProxy (`MAV>`) konsolunda çalıştırılır.

> **Güvenlik notu:** Bu komutlar simülasyon testleri içindir. Önce Gazebo–SITL bağlantısının ve motor/kanal eşlemesinin doğru olduğunu doğrulayın. İrtifa, hız ve mod geçişi eşikleri hava aracı parametrelerine, model dinamiklerine ve simülasyon koşullarına bağlıdır.

### 1. Dikey Kalkış (QLOITER)

Uçuş modunu dikey konum kontrol moduna alın:

```text
mode QLOITER
```

Motorları etkinleştirin:

```text
arm throttle
```

Gaz kanalına komut gönderin:

```text
rc 3 1700
```

İHA'nın tırmanışını ve kararlılığını izleyin. Yaklaşık 30 metre irtifaya ulaşıldığında gaz kanalını nötr değere getirmek için:

```text
rc 3 1500
```

İrtifa ve motor davranışını telemetri üzerinden takip edin. Verilen RC değerlerinin hedef irtifayı garanti etmediğini unutmayın.

### 2. Sabit Kanat Uçuşuna Geçiş (FBWA)

Sabit kanatlı uçuş moduna geçin:

```text
mode FBWA
```

Gaz kanalına komut gönderin:

```text
rc 3 1750
```

İHA'nın ileri hızını, tutumunu ve geçiş davranışını izleyin. Geçişin başarıyla tamamlandığı yalnızca gaz komutuyla belirlenmez; hava hızı, QuadPlane geçiş ayarları, motor konfigürasyonu ve uçuş kontrol parametreleri de etkilidir.

Yaklaşık 15 m/s stall eşiği evrensel bir değer değildir. Gerçek eşik, simülasyon modelinin aerodinamik özelliklerine ve yapılandırmasına göre belirlenmelidir.

### 3. Dikey İniş (QLAND)

Dikey iniş modunu etkinleştirin:

```text
mode QLAND
```

İniş boyunca İHA'nın konumunu, irtifasını ve motor durumunu izleyin. Yere temas edildiğini ve inişin tamamlandığını doğrulamadan motorları devre dışı bırakmayın.

İniş tamamlandıktan sonra gaz kanalını minimum değere getirin:

```text
rc 3 1000
```

Ardından uygun bir moda geçin:

```text
mode MANUAL
```

Son olarak, araç yerde ve güvenli durumdayken:

```text
disarm
```

> Komutların tam sırası ArduPilot sürümüne, güvenlik kontrollerine ve araç konfigürasyonuna göre değişebilir. Mod değişikliklerini ve arm/disarm durumunu MAVProxy mesajlarından doğrulayın.

## 🔌 ROS 2 ve Gazebo Köprüsü

`ros_gz_bridge`, Gazebo sensör ve odometri verilerinin ROS 2 topic'lerine aktarılmasını sağlar.

### 1. ROS 2 Ortamını Hazırlama

Yeni bir terminal açın ve konteynere bağlanın:

```bash
docker exec -it vtol_sim bash
```

ROS 2 Humble ortamını yükleyin:

```bash
source /opt/ros/humble/setup.bash
```

### 2. IMU ve Odometri Verilerini Aktarma

Aşağıdaki komut, IMU ve odometri mesajlarını ROS 2 ile Gazebo arasında köprülemeyi dener:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/world/iris_runway/model/quadplane/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU' \
  '/model/quadplane/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
```

Burada:

- `sensor_msgs/msg/Imu`: ROS 2 IMU mesaj tipidir.
- `nav_msgs/msg/Odometry`: ROS 2 odometri mesaj tipidir.
- `gz.msgs.IMU`: Gazebo IMU mesaj tipidir.
- `gz.msgs.Odometry`: Gazebo odometri mesaj tipidir.
- `[` işareti, köprü için Gazebo'dan ROS 2'ye veri yönünü belirtir.

**Önemli:** Topic isimleri örnek dünya ve model isimlerine bağlıdır. Gazebo dünyasının `quadplane_runway.sdf` olarak belirtilmesine karşın örnek IMU topic'inde `iris_runway` kullanılmıştır. Çalıştırmadan önce gerçek topic isimlerini kontrol edin:

```bash
gz topic -l
```

ROS 2 tarafında topic'leri kontrol etmek için:

```bash
ros2 topic list
```

IMU mesajlarını görüntülemek için:

```bash
ros2 topic echo /world/iris_runway/model/quadplane/link/base_link/sensor/imu_sensor/imu
```

Odometri topic'ini doğrulamak için:

```bash
ros2 topic echo /model/quadplane/odometry
```

Bu komutlarda kullanılan topic yolları, gerçek simülasyonda listelenen adlarla eşleşmelidir.

### 3. Kamera Entegrasyonu

Kamera verilerini ROS 2'ye aktarmak için önce Gazebo'daki kamera topic'ini bulun:

```bash
gz topic -l
```

Daha sonra ilgili topic'i, doğru Gazebo ve ROS 2 mesaj tiplerini kullanarak `parameter_bridge` komutuna ekleyin. Kamera mesaj tipi ve topic adı, kullanılan sensör eklentisine göre değişebilir.

## 🔍 Sorun Giderme

### Docker konteyneri çalışmıyor

Konteyner durumunu ve loglarını kontrol edin:

```bash
docker ps -a
docker compose logs
```

Gerekirse servisleri yeniden oluşturun:

```bash
docker compose up -d --build
```

### Gazebo model veya dünya dosyaları bulunamıyor

Ortam değişkenlerini kontrol edin:

```bash
echo "$GZ_SIM_RESOURCE_PATH"
echo "$GZ_SIM_SYSTEM_PLUGIN_PATH"
```

Dosyaların gerçekten mevcut olduğunu doğrulayın:

```bash
ls /home/uav/ardupilot_gazebo/models
ls /home/uav/ardupilot_gazebo/worlds
ls /home/uav/ardupilot_gazebo/build
```

### SITL, Gazebo'ya bağlanamıyor

- Gazebo sunucusunun çalıştığını doğrulayın.
- `ardupilot_gazebo` eklentisinin dünya dosyasında yapılandırıldığını kontrol edin.
- JSON/UDP bağlantı adresi ve portlarının iki tarafta da uyumlu olduğunu doğrulayın.
- Gazebo ve SITL loglarını inceleyin.

### ROS 2 topic'leri görünmüyor

- ROS 2 ortamının kaynaklandığını doğrulayın.
- `ros_gz_bridge` paketinin kurulu olduğunu kontrol edin.
- Topic isimlerini `gz topic -l` ve `ros2 topic list` komutlarıyla karşılaştırın.
- Gazebo mesaj tipleri ile ROS 2 mesaj tiplerinin desteklenen eşleşmeler olduğundan emin olun.

## 🧹 Simülasyonu Durdurma

SITL, Gazebo ve ROS 2 köprüsünü çalıştıran terminallerde `Ctrl+C` kullanarak süreçleri durdurun.

Docker Compose servislerini durdurmak için:

```bash
docker compose down
```

Bu komut konteynerleri durdurur ve kaldırır; Docker imajlarını varsayılan olarak silmez.

## 📁 Proje Bileşenleri

Projenin temel bileşenleri şunlardır:

- **ArduPilot SITL:** Uçuş kontrol algoritmalarını ve araç davranışını simüle eder.
- **Gazebo Harmonic:** Fizik simülasyonunu, dünyayı ve sensörleri çalıştırır.
- **ardupilot_gazebo:** Gazebo ile ArduPilot arasında simülasyon haberleşmesini sağlar.
- **MAVProxy:** Uçuş modu, telemetri ve kontrol komutları için konsol arayüzü sunar.
- **ROS 2 Humble:** Robotik uygulamalar ve sensör verisi işleme altyapısını sağlar.
- **ros_gz_bridge:** Gazebo mesajlarını ROS 2 mesajlarına aktarır.

## 📌 Geliştirme Notları

- Bu proje, Apple Silicon üzerinde ARM64 uyumluluğunu hedefler. Kullanılan tüm Docker imajlarının ve derlenen bağımlılıkların ARM64 desteği doğrulanmalıdır.
- Gazebo Harmonic ve ROS 2 Humble paketlerinin aynı ortamda kullanılması, kurulum kaynağına ve bağımlılık sürümlerine bağlıdır.
- Gazebo dünya dosyasındaki model adları, sensör topic'leri ve eklenti yolları gerçek proje dosyalarıyla eşleşmelidir.
- Uçuş modları ve geçiş davranışları, kullanılan ArduPilot sürümü ve QuadPlane parametreleriyle doğrulanmalıdır.

## 📄 Lisans

Bu proje için geçerli lisans bilgilerini depo içindeki `LICENSE` dosyasına göre ekleyin.

## 👤 Katkıda Bulunma

Hata bildirimleri, iyileştirmeler ve yeni simülasyon senaryoları için GitHub deposunda issue açabilir veya pull request gönderebilirsiniz.
