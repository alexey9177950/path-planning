# path-planning
Проектная работа, 2 курс.

## Формат входных данных.

Программа принимает на вход xml-файл с описанием запроса (примеры входных файлов в папке examples). Корень xml-файла содержит три тега: map, algorithm и options.

### Тег map

Содержит описание карты.

+ width и height -- ширина и высота карты в клетках.

+ cellsize -- необязательный тег, означающий какому расстоянию соответствует клетка на карте.

+ startx, starty, finishx, finishy -- координаты начала и конца пути. Нумеруются слева направо и снизу вверх, начиная с нуля.

+ grid -- содержит таблицу из строк, заключённых в тег row. Строка содержит нули и единицы. Единица соответствует свободной клетке, а ноль -- клетке, занятой препятствием.

### Тег algorithm

Содержит описание алгоритма.

+ searchtype -- алгоритм, который будет использоваться для поиска пути. Возможные значения тега: bfs, dijkstra, astar, jp_search, theta.

+ metrictype -- метрика, которая будет использоваться алгоритмом. Возможные значения тега: diagonal, manhattan, euclidean, chebyshev (см. описания метрик далее).

+ breakingties -- ?

+ hweight -- ?

+ allowdiagonal -- определяет, возможно ли перемещаться по диагонали. Содержит true или false.

+ cutcorners -- определяет, возможно перемещаться между двумя клетками по диагонали, когда одна из клеток смежных с обоими занята, то есть путь проходит по диагонали через угол препятствия. Содержит true или false.

+ allowsqueeze -- определяет, возможно перемещаться между двумя клетками по диагонали, когда обе клетки, смежные с обоими заняты, то есть можно ли "просачиваться" между двумя клетками препятсвия, расположенными по диагонали друг относительно друга. Содержит true или false.

### Тег options

+ loglevel -- определяет насколько подробным должен быть ответ на запрос. Возможны значения от 0 до ? (чем больше число, тем более подробный ответ).

+ logpath -- определяет путь до файла, который будет являться ответом на запрос. По умолчанию -- ?

+ logfilename -- определяет имя файла, который будет являться ответом на запрос. По умолчанию -- \[имя входного файла\]\_log.xml

## Доступные метрики

Выбор метрики определяет как будет рассчитываться расстояние между клетками с координатами ![координаты 1](http://www.sciweavers.org/upload/Tex2Img_1511374621/render.png) и ![координаты 2](http://www.sciweavers.org/upload/Tex2Img_1511374644/render.png)

#### Эвклидова метрика

![Формула 1](http://www.sciweavers.org/upload/Tex2Img_1511374147/render.png)

#### Манхеттенская метрика

![Формула 2](http://www.sciweavers.org/upload/Tex2Img_1511374202/render.png)

#### Диагональная метрика

![Формула 3](http://www.sciweavers.org/upload/Tex2Img_1511374842/render.png)

#### Метрика Чебышёва

![Формула 4](http://www.sciweavers.org/upload/Tex2Img_1511374260/render.png)

