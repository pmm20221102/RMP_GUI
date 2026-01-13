# RMP_GUI
Diese GUI dient der Implementierung der Bewegung von Robotern und Roboterarmen im RMP-Projekt sowie der Anzeige von Echtzeit-Kamerabildern.
## version
ROS2 jazzy + ubuntu 24.04
## Verzeichnisstruktur / Directory Structure
- **launch/**: Enthält die Launch-Dateien / Contains launch files.
- **ui/**: Benutzeroberflächen-Dateien / UI design files (.ui).
- **rmp_gui/**: Python-Quellcode / Core Python logic.
- **config/**: Parameterkonfiguration / Node parameter configurations.

## starten
Nachdem Sie das Gazebo-Projekt für RMP gestartet haben, führen Sie es aus...
```bash
ros2 launch rmp_gui rmp_gui.launch.py
```
