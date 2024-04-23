# spot-perception

### Ausführen des Docker-Containers auf Jetson:

1. Terminal in Jetson starten:
```
strg + alt + t
```

2. Docker-Image mit spot ros2 driver ziehen und starten
```
docker run -it eshansavla0512/ros2-spot-arm64
```

Es sollte eine Instanz des Containers starten und kann über eine solch-ähnliche Ausgabe im Terminal erkannt werden:

```
$ docker run -it eshansavla0512/ros2-spot-arm64

#
robot@67f249cc0997:~$
```

3. Den folgenden Befehl ausführen, um den ROS-Distro zu sourcen
```
source /opt/ros/humble/setup.bash
```

4. Den folgenden Befehl ausführen, um zum ROS-Workspace Ordner zu wechseln
```
cd spot_ros2_ws
```

5. Folgenden Befehl ausführen, um den ROS-Package zu bauen
```
colcon build --symlink-install --packages-ignore proto2ros_tests
```

Ausgaben bspw. mit Screenshots protokollieren zur späteren Auswertung. Ich möchte sehen, ob die Sachen richtig bauen, und falls nicht, welche Fehler entstehen.