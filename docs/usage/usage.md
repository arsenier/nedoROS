# Запуск программ

## Ручной запуск робота
Запуск контейнера для ROS и вход в него
```
cd ~/nedoROS/rpi_ws/
make zmrobo_run 
make zmrobo_into 
```
Внутри контейнера:
```
cd workspace/
source install/setup.bash
make lidar
make pi_driv
```

## Запуск модулей на компьютере
Запуск распознавания положения charuco-доски и aruco-маркера 239, а также определение объектов на стартовых позициях
```
cd charuco_ws
make
```
