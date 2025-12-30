# Temizlik Robotu - ROS Noetic + TurtleBot3

Bu proje, bir ev ortamında odalara gidip temizlik yapan **süpürge robotu** simülasyonudur. Robot, Gazebo ortamında evin haritasını kullanarak konumunu belirler, odalara gider, QR kodları okur ve mini temizlik rotalarını tamamlar.

---

## Proje Mantığı

1. **Haritalama ve Navigasyon**
   - Robot evin haritasını kullanır ve AMCL ile kendi konumunu belirler.
   - Waypoint’lere giderek odalara ulaşır.

2. **Oda Girişi ve QR Doğrulama**
   - Her odanın girişinde bir QR kod bulunur.
   - Robot QR kodu okur ve doğru odada olduğunu doğrular.

Oda Girişi → [QR Kodu] → Doğru Oda? → Mini Temizlik

3. **Mini Temizlik Rotaları**
   - Doğru odadaysa robot, 3–5 waypoint üzerinden odanın mini turunu tamamlar.
   - Temizlik sırasında odanın durumu raporlanır

---

## Dosya Yapısı
temizlik_robotu/
├─ config/
│ └─ mission.yaml # Odaların ve temizlik görevlerinin tanımı
├─ launch/
│ ├─ mission.launch # Görev ve simülasyonu başlatır
│ ├─ move_base.launch
│ └─ turtlebot3_navigation.launch
├─ maps/
│ ├─ house_map.yaml
│ └─ house_map.pgm
├─ scripts/
│ └─ mission_controller.py
├─ src/
│ └─ waypoint_marker.py
└─ qr/
└─ *.png # QR kod görselleri

##Calıstırma Komutları

-Catkin workspace’i hazırla
cd ~/catkin_ws
catkin_make
source devel/setup.bash

-Gazebo’yu açmak
export TURTLEBOT3_MODEL=waffle
roslaunch temizlik_robotu turtlebot3_navigation.launch

-RViz’i açmak
roslaunch temizlik_robotu mission.launch




