# Запуск программ

## Ручной запуск робота
Запуск контейнера для ROS и вход в него
```bash
cd ~/nedoROS/rpi_ws/
make zmrobo_run 
```

В двух разных терминалах:

```bash
make zmrobo_into 
cd workspace/
source install/setup.bash
make all
```

```bash
make zmrobo_into 
cd workspace/
source install/setup.bash
make master
```

## Запуск модулей на компьютере
Запуск распознавания положения charuco-доски и aruco-маркера 239, а также определение объектов на стартовых позициях
```bash
cd charuco_ws
make
```
