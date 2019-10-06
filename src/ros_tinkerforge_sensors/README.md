### Tinkerforge Sensors

Mit diesem Paket können Tinkerforge Sensoren im ROS genutzt werden. Auch Sensoren des gleichen Typs werden erkannt. Durch Nutzung des "robusten Ansatzes" ist die Angabe von UIDs nicht notwendig. Die Sensorwerte werden in entsprechende ROS-Messages über automatisch generierte Topics verbreitet. Unter Verwendung eines Launch-Files können Topic per UID manuell festgelegt werden.

This package allows you to use TinkerForge sensors with ROS. Sensors of same type will be supported. Package used the "Rugged Approach", so it's not necessary to configure the sensor UIDs. Sensor values will be published over automatic generated topics with typical ROS message types. With a Launchfile a topic can be bind on an UID.

##### Hinweise / Hints

Es wird nicht empfohlen zwei Instanzen des Programmes zu starten (z.B. zwei Master Bricks), da sich die Sensor Topics sonst überlagern.

It's not recommended to run more than one instance of the program, because different sensors of same type will be published into one topic.

### Unterstütze Geräte / Supported Devices

Bricklets:

* Ambient Light => sensor_msgs/Illuminance
* Ambient Light 2.0 => sensor_msgs/Illuminance
* Distance IR => sensor_msgs/Range
* Distance US => sensor_msgs/Range
* GPS (Test) => sensor_msgs/NavSatFix
* Motion Detector (Test)
* Temperature => sensor_msgs/Temperature

Bricks:

* IMU => sensor_msgs/Imu
* IMU 2.0 => sensor_msgs/Imu

### Installation

Herunterladen des Git-Repository in das catkin workspace Verzeichnis.

clone the git repository to catkin workspace.

`git clone https://github.com/gus484/ros-tinkerforge_sensors`

Workspace kompilieren / build workspace

`catkin_make`

### Ausführen / run node

`rosrun tinkerforge_sensors tinkerforge_sensors_node`

### Launchfile

Argumente
* port (int) *Tinkerforge Port*
* ip (string) *Tinkerforge IP*

`roslaunch tinkerforge_sensors tinkerforge_sensors.launch`

### ToDo

* mehr Sensoren unterstützen / suport more sensors
* Kalibrierung für Distance US Sensor / calibration for Distance US sensor
* mehr Konfigurationsmöglichkeiten / more config options
* Verwaltung mehrerer Master / handle more than one master
