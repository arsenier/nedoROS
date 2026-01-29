# nedoROS

### raspberry:
 OS is raspberry 64 bit for rpi 4B
 hostname: nedoROS, username: pi, password: raspberry
 https://habr.com/ru/articles/916550/ -  гайд

## Требования к роботу

1. Задать скорость моторам (левому и правому)
2. Схватить/расхватить захват
3. Прочитать локальную одометрию
4. Прочитать значение концевиков
5. Обнулить одометрию (перезагрузить ардуино)

Частота обмена Arduino-Raspberry 10-100Гц

Протокол обмена:

```
Rpi-Arduino:
Обычное управление:
|0x01|left_motor_speed:float|right_motor_speed:float|gripper:bool|
10 bytes

Сброс:
|0xFF|0xFF|0xFF|0xFF|
4 bytes

Arduino-Rpi:
Ответ на обычное управление:
|x:float|y:float|theta:float|usiki:2x bool|
13 bytes
```


