# nedoROS

## Файловая структура

```
.
├── arduino_firmware/ - прошивка контоллера Arduino Uno
├── charuco_ws/ - рабочее пространство ROS2 сервера GPS
├── yolo_ws/ - рабочее пространство нейронной сети для классификации уточек
└── rpi_ws/ - рабочее пространство ROS2 бортового компьютера Raspberry Pi
```

## Общая схема системы

![](docs/scheme.excalidraw.png)

### raspberry:
 OS is raspberry 64 bit for rpi 4B
 hostname: nedoROS, username: pi, password: raspberry
 https://habr.com/ru/articles/916550/ -  гайд

## Требования к роботу

1. Задать скорость моторам (левому и правому)
2. Схватить/расхватить захват
3. Прочитать локальную одометрию
4. Прочитать значение концевиков
5. Обнулить одометрию (перезагрузить ардуино) [не реализовано]

Частота обмена Arduino-Raspberry 10Гц

Протокол обмена:

```
Rpi-Arduino:
Обычное управление:
|0x01|left_motor_speed:float|right_motor_speed:float|gripper:byte|checksum:byte|
11 bytes

Arduino-Rpi:
Ответ на обычное управление:
|0x01|x:float|y:float|theta:float|usik_left:byte|usik_right:byte|checksum:byte|
16 bytes
```

## ROS

Запуск распознавания положения charuco-доски и aruco-маркера 239
```
make
```
Позиции пишутся в топики `/image_raw_charuco_pose` - доска и `/image_raw_apriltag_pose` - маркер (позно понял что это не apriltag)

Пример программы вычисления относительных координат маркера на доске (плюс угол поворота), выводит в консоль вычесленные координаты
```
make run_pose_on_board
```

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
make pi_driver
```

## Модель машинного обучения
Постановка задачи: По изображение с верхней камеры понять начальное расположение типов объектов.

Данные: Crop картинки с верхней камеры, в разрешении 120x100 начальных  позиций с объектами. Данные были размечены в Label Studio.

Модель: Дообученная на наших данных YOLOv8n-cls.

Хранилище: Веса и данные хранятся в git lfs репозитория с помощью DVC(Data Version Control).


Надо установить:
```
pip install ultralytics==8.2.103 -q
pip install dvc
```

Чтобы получить веса модели и данные надо сделать:
```
dvc pull
```
