# Классификация объектов

## Постановка задачи
По изображение с верхней камеры понять начальное расположение типов объектов.
## Данные
Crop картинки с верхней камеры, в разрешении 120x100 начальных  позиций с объектами. Данные были размечены в Label Studio.
## Модель
Дообученная на наших данных YOLOv8m-cls.

Для работы надо установить:
```
pip install ultralytics==8.2.103 -q
```

Чтобы получить веса модели и данные находятся на huggingface:

[Ссылка на модель](https://huggingface.co/BW/ROS2026_Hackathon_Yolov8_classification)

[Ссылка на датасет](https://huggingface.co/datasets/BW/ROS_Hackathon_2026_Toys_Dataset)

<!-- ### Классификация объектов -->

<!-- https://superuser.com/a/556031 -->
<!-- ffmpeg -i dataset.mp4     -vf "fps=15,scale=360:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse"     -loop 0 dataset15360.gif -->

#### Сборка датасета

![](../img/dataset15360.gif)
