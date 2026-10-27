# Çoklu Ajanlı VTOL Simülasyon Altyapısı
### Gazebo Harmonic · ArduPilot SITL · ROS 2 Humble · MAVROS

Bu depo, **Gazebo Harmonic (GZ Sim 8)** üzerinde iki bağımsız VTOL (QuadPlane) modelinin fizik simülasyonunu, ArduPilot SITL otopilot süreçlerini ve ROS 2 Humble üzerindeki bağımsız MAVROS telemetri ve kontrol köprülerini tek bir komutla başlatan simülasyon altyapısını içerir.

Sistem, iki İHA'nın birbirinden bağımsız şekilde çalışmasını sağlayarak çoklu ajan kontrol algoritmalarının, otonom görev planlayıcılarının ve sürü yazılımlarının geliştirilmesi ve test edilmesi için bir ortam sunar.

---

## 1. Mimari ve Ağ Yapısı

Her iki İHA da bağımsız ArduPilot SITL instance'ları olarak çalıştırılır. MAVROS köprüleri, TCP bağlantıları üzerinden ilgili otopilotlarla iletişim kurarak telemetri ve kontrol arayüzlerini ROS 2 ekosistemine aktarır.

### İHA yapılandırması

| Parametre | VTOL 1 | VTOL 2 |
|---|---|---|
| Gazebo model adı | `vtol_1` | `vtol_2` |
| SITL instance | `-I0` | `-I1` |
| Gazebo JSON portu | `9002` | `9012` |
| MAVROS bağlantı URL'i | `tcp://127.0.0.1:5762` | `tcp://127.0.0.1:5773` |
| Target System ID | `1` | `1` |
| ROS 2 namespace | `/vtol_1` | `/vtol_2` |

### Başlatma sıralaması

ArduPilot SITL'in TCP sunucusunu başlatması ve EKF sensör hizalamasını tamamlaması yaklaşık **10–12 saniye** sürebilir.

`Connection refused` ve `link down` gibi bağlantı hatalarını önlemek amacıyla MAVROS düğümleri, launch dosyasında `TimerAction` kullanılarak 12 saniye gecikmeli başlatılır.

> **Not:** Başlatma gecikmesi, SITL'in her koşulda hazır olduğunu garanti etmez. Daha sağlam bir yapı için bağlantı ve araç hazır olma durumlarının kontrol edilmesi önerilir.

---

## 2. Ön Koşullar ve Bağımlılıklar

Simülasyonu çalıştırmadan önce Docker konteynerinde aşağıdaki bileşenlerin kurulu ve yapılandırılmış olduğundan emin olun:

- Gazebo Harmonic (GZ Sim 8)
- ArduPilot SITL
- ROS 2 Humble
- MAVROS
- MAVROS mesaj paketleri
- GeographicLib veri setleri
- `colcon` derleme araçları
- `vtol_bringup` ROS 2 paketi

### MAVROS kurulumu

Konteyner içerisinde aşağıdaki komutları çalıştırın:

```bash
sudo apt-get update

sudo apt-get install -y \
    ros-humble-mavros \
    ros-humble-mavros-msgs

sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

---

## 3. Kurulum ve Çalıştırma

Simülasyon ortamı, `vtol_sim` isimli Docker konteyneri içerisinde çalıştırılır.

### Adım 1: Docker konteynerine bağlanma

Host makinede terminal açın ve konteynere bağlanın:

```bash
docker exec -it vtol_sim bash
```

### Adım 2: ROS 2 çalışma alanını derleme

Konteyner içerisinde aşağıdaki komutları çalıştırın:

```bash
cd /home/uav/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select vtol_bringup

source install/setup.bash
```

> **Not:** Bu derleme işleminin ilk kurulumda veya `vtol_bringup` paketinde değişiklik yapıldığında gerçekleştirilmesi gerekir.

### Adım 3: Simülasyonu başlatma

Simülasyonu başlatmak için aynı konteyner terminalinde şu komutları çalıştırın:

```bash
source /opt/ros/humble/setup.bash

source /home/uav/ros2_ws/install/setup.bash

export GZ_IP=127.0.0.1

ros2 launch vtol_bringup sim.launch.py
```

Bu launch dosyası yapılandırmasına bağlı olarak aşağıdaki bileşenler başlatılır:

- Gazebo simülasyon ortamı
- `vtol_1` için ArduPlane SITL instance'ı
- `vtol_2` için ArduPlane SITL instance'ı
- Her İHA için bağımsız MAVROS bağlantısı
- ROS 2 üzerinden telemetri ve kontrol arayüzleri

---

## 4. Telemetri ve Veri Akışını Doğrulama

Simülasyon çalışırken yeni bir host terminali açın ve ikinci terminali konteynere bağlayın:

```bash
docker exec -it vtol_sim bash
```

ROS 2 ortamını yükleyin:

```bash
source /opt/ros/humble/setup.bash

source /home/uav/ros2_ws/install/setup.bash
```

### VTOL 1 bağlantı kontrolü

```bash
ros2 topic echo /vtol_1/mavros/state --once
```

Beklenen temel alanlar:

```yaml
connected: true
mode: FBWA
```

`connected: true` değeri, MAVROS'un otopilot bağlantısını algıladığını gösterir. `FBWA` ise beklenen başlangıç uçuş modu olarak belirtilmiştir; gerçek değer yapılandırmaya ve mevcut uçuş durumuna bağlıdır.

Yerel konum verisini kontrol edin:

```bash
ros2 topic echo /vtol_1/mavros/global_position/local --once
```

### VTOL 2 bağlantı kontrolü

```bash
ros2 topic echo /vtol_2/mavros/state --once
```

Beklenen temel alanlar:

```yaml
connected: true
mode: FBWA
```

Yerel konum verisini kontrol edin:

```bash
ros2 topic echo /vtol_2/mavros/global_position/local --once
```

> **Önemli:** Topic isimleri ve mesaj türleri kullanılan MAVROS yapılandırmasıyla eşleşmelidir. Bağlantı veya veri akışı sorunu yaşanırsa `ros2 topic list` ve `ros2 topic info` komutlarıyla mevcut topic'leri ve mesaj türlerini doğrulayın.

---

## 5. Otonom Kontrol ve Sürü Yazılımı Arayüzleri

Sürü algoritmaları ve otonom görev planlayıcıları, her İHA'ya kendi ROS 2 namespace'i üzerinden erişebilir.

### 5.1. Temel topic'ler

| İşlev | VTOL 1 | VTOL 2 |
|---|---|---|
| Yerel konum / odometri | `/vtol_1/mavros/global_position/local` | `/vtol_2/mavros/global_position/local` |
| Küresel GPS konumu | `/vtol_1/mavros/global_position/global` | `/vtol_2/mavros/global_position/global` |
| Yerel hedef konumu | `/vtol_1/mavros/setpoint_position/local` | `/vtol_2/mavros/setpoint_position/local` |

| Topic | Mesaj türü |
|---|---|
| `mavros/global_position/local` | `nav_msgs/msg/Odometry` |
| `mavros/global_position/global` | `sensor_msgs/msg/NavSatFix` |
| `mavros/setpoint_position/local` | `geometry_msgs/msg/PoseStamped` |

Topic isimleri, namespace'in önüne eklenmesiyle her İHA için bağımsız hale gelir.

### 5.2. Temel servisler

| İşlev | VTOL 1 | VTOL 2 |
|---|---|---|
| Motorları başlatma / durdurma (Arm/Disarm) | `/vtol_1/mavros/cmd/arming` | `/vtol_2/mavros/cmd/arming` |
| Uçuş modu değiştirme | `/vtol_1/mavros/set_mode` | `/vtol_2/mavros/set_mode` |

| Servis | Mesaj türü |
|---|---|
| `mavros/cmd/arming` | `mavros_msgs/srv/CommandBool` |
| `mavros/set_mode` | `mavros_msgs/srv/SetMode` |

Uçuş modu değiştirme servisi üzerinden `GUIDED`, `QLOITER` veya araç tarafından desteklenen diğer uygun modlar talep edilebilir. Modun başarıyla değişmesi, ArduPilot'un uçuş koşullarına ve mod gereksinimlerine bağlıdır.

### 5.3. Kullanım örnekleri

VTOL 1'in durumunu sorgulama:

```bash
ros2 topic echo /vtol_1/mavros/state
```

VTOL 2'nin GPS verisini izleme:

```bash
ros2 topic echo /vtol_2/mavros/global_position/global
```

VTOL 1 için mevcut arming servis arayüzünü inceleme:

```bash
ros2 service info /vtol_1/mavros/cmd/arming
```

Mevcut topic ve servisleri listeleme:

```bash
ros2 topic list
ros2 service list
```

> **Kontrol notu:** `setpoint_position/local` üzerinden hedef gönderirken uygun uçuş modunun etkin olması, setpoint mesajlarının gerekli hızda yayımlanması ve koordinat çerçevelerinin doğru kullanılması gerekir. Yalnızca topic'e mesaj göndermek, İHA'nın hedefe uçacağını garanti etmez.

---

## 6. Simülasyonu Durdurma ve Süreç Temizliği

Simülasyon terminalinde `Ctrl + C` kullanarak launch sürecini durdurun.

Sonraki çalıştırmalarda `Address already in use` veya `Connection refused` gibi hatalarla karşılaşılması durumunda, konteyner içerisinde kalan süreçleri kontrol edin.

### Süreçleri temizleme

Aşağıdaki komut, belirtilen süreçleri zorla sonlandırır:

```bash
killall -9 gz sim_vehicle.py arduplane mavros_node parameter_bridge 2>/dev/null || true
```

> **Uyarı:** Bu komut, aynı isimli süreçlerin tümünü sonlandırabilir. Aynı konteynerde başka bir simülasyon çalışıyorsa bu süreçleri de etkileyebilir. Gerektiğinde önce `ps` veya `pgrep` ile çalışan süreçleri kontrol edin.

### Hızlı yeniden başlatma

Temizliğin ardından simülasyonu yeniden başlatmak için:

```bash
killall -9 gz sim_vehicle.py arduplane mavros_node parameter_bridge 2>/dev/null || true

source /opt/ros/humble/setup.bash

source /home/uav/ros2_ws/install/setup.bash

export GZ_IP=127.0.0.1

ros2 launch vtol_bringup sim.launch.py
```

---

## 7. Sorun Giderme

| Hata / Belirti | Kontrol edilmesi gerekenler |
|---|---|
| `Connection refused` | SITL sürecinin çalıştığını, TCP portlarının dinlendiğini ve MAVROS bağlantı URL'lerinin doğru olduğunu kontrol edin. |
| `link down` | SITL'in başlatılma süresini, TCP bağlantısını ve MAVROS loglarını kontrol edin. |
| `connected: false` | İlgili MAVROS instance'ının doğru SITL portuna bağlandığını doğrulayın. |
| Yerel konum verisi gelmiyor | İlgili topic'in varlığını, mesaj akışını ve EKF/GPS durumunu kontrol edin. |
| `Address already in use` | İlgili portu kullanan süreçleri belirleyin ve yalnızca gerekli süreçleri sonlandırın. |
| Uçuş modu değişmiyor | İstenen modun araç tarafından desteklendiğini ve geçiş koşullarının sağlandığını kontrol edin. |
| Paket bulunamıyor | Çalışma alanını yeniden derleyin ve `install/setup.bash` dosyasını kaynak olarak yükleyin. |

---

## 8. Projenin Amacı

Bu altyapı, çoklu VTOL sistemlerinin simülasyon ortamında geliştirilmesini ve test edilmesini kolaylaştırmak amacıyla tasarlanmıştır.

Başlıca kullanım alanları:

- Çoklu İHA telemetrisi ve durum takibi
- Bağımsız araç kontrolü
- Otonom görev planlama
- Sürü koordinasyonu ve çoklu ajan algoritmaları
- ROS 2 tabanlı kontrol yazılımlarının geliştirilmesi
- Simülasyon ortamında uçuş senaryolarının test edilmesi

**Temel hedef:** İki bağımsız VTOL aracının Gazebo, ArduPilot SITL ve ROS 2/MAVROS entegrasyonu üzerinden tek bir simülasyon ortamında çalıştırılması ve merkezi bir yazılım tarafından koordine edilebilmesidir.

