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

![срезание углов](readme_img/cc.png)

+ allowsqueeze -- определяет, возможно перемещаться между двумя клетками по диагонали, когда обе клетки, смежные с обоими заняты, то есть можно ли "просачиваться" между двумя клетками препятсвия, расположенными по диагонали друг относительно друга. Содержит true или false.

![просачивание](readme_img/sc.png)

### Тег options

+ loglevel -- определяет насколько подробным должен быть ответ на запрос. Возможны значения от 0 до ? (чем больше число, тем более подробный ответ).

+ logpath -- определяет путь до файла, который будет являться ответом на запрос. По умолчанию -- ?

+ logfilename -- определяет имя файла, который будет являться ответом на запрос. По умолчанию -- \[имя входного файла\]\_log.xml

## Доступные метрики

Выбор метрики определяет как будет рассчитываться расстояние между клетками с координатами ![координаты 1](readme_img/c1.png) и ![координаты 2](readme_img/c2.png)

#### Эвклидова метрика

![Формула 1](readme_img/f1.png)

#### Манхеттенская метрика

![Формула 2](readme_img/f2.png)

#### Диагональная метрика

![Формула 3](readme_img/f3.png)

#### Метрика Чебышёва

![Формула 4](readme_img/f4.png)

