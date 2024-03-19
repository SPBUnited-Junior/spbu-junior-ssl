import math
import bridge.processors.const as const

class Graph:
    def __init__(self, num_vertices):
        self.num_vertices = num_vertices
        self.graph = [[0] * num_vertices for _ in range(num_vertices)]

    def add_edge(self, from_vertex, to_vertex, weight):
        self.graph[from_vertex][to_vertex] = weight
        self.graph[to_vertex][from_vertex] = weight

    def dijkstra(self, start_vertex):
        distances = [float('inf')] * self.num_vertices
        distances[start_vertex] = 0
        visited = [False] * self.num_vertices

        for _ in range(self.num_vertices):
            min_distance = float('inf')
            min_vertex = -1

            for v in range(self.num_vertices):
                if not visited[v] and distances[v] < min_distance:
                    min_distance = distances[v]
                    min_vertex = v

            visited[min_vertex] = True

            for v in range(self.num_vertices):
                if (
                    not visited[v] 
                    and self.graph[min_vertex][v] 
                    and distances[min_vertex] != float('inf')
                    and distances[min_vertex] + self.graph[min_vertex][v] < distances[v]
                ):
                    distances[v] = distances[min_vertex] + self.graph[min_vertex][v]

        return distances

class bobLine:
    """
    Борина линия
    Надо ли?
    """
    def __init__(self, A, B, C):
        self.A = A
        self.B = B
        self.C = C

class Point:
    """
    Класс, описывающий точку (вектор)
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, p):
        return Point(self.x + p.x, self.y + p.y)

    def __neg__(self):
        return Point(-self.x, -self.y)

    def __sub__(self, p):
        return self + -p

    def __mul__(self, a: float):
        return Point(self.x * a, self.y * a)

    def __truediv__(self, a: float):
        return self * (1/a)

    def __pow__(self, a: float):
        return Point(self.x ** a, self.y ** a)

    def __eq__(self, p):
        return self.x == p.x and self.y == p.y

    def __str__(self):
        return f'x = {self.x:.2f}, y = {self.y:.2f}'

    def mag(self):
        return math.hypot(self.x, self.y)

    def arg(self):
        return math.atan2(self.y, self.x)

    def unity(self):
        if self.mag() == 0:
            # raise ValueError("БАГА, .unity от нулевого вектора")
            return self
        return self/self.mag()

i = Point(1, 0)
j = Point(0, 1)

def dist2line(p1, p2, p):
    """
    Рассчитать расстояние от точки p до прямой, образованной точками p1 и p2
    """
    return abs(vect_mult((p2 - p1).unity(), p - p1))

def line_poly_intersect(p1, p2, points):
    """
    Определить, пересекает ли линия p1-p2 полигон points
    """
    vec = p2-p1
    old_sign = sign(vect_mult(vec, points[0]-p1))
    for p in points:
        if old_sign != sign(vect_mult(vec, p - p1)):
            return True
    return False

def segment_poly_intersect(p1, p2, points):
    """
    Определить, пересекает ли отрезок p1-p2 полигон points
    Если да - возвращает одну из двух точек пересечения
    Если нет - возвращает None
    """
    for i in range(-1, len(points)-1):
        p = get_line_intersection(p1, p2, points[i], points[i+1], 'SS')
        if p is not None:
            return p
    return None

def is_point_inside_poly(p, points):
    """
    Определить, лежит ли точка внутри полигона
    """
    old_sign = sign(vect_mult(p - points[-1], points[0]-points[-1]))
    for i in range(len(points)-1):
        if old_sign != sign(vect_mult(p - points[i], points[i+1]-points[i])):
            return False
    return True

def dist(a, b):
    """
    Определить расстояние между двумя точками
    """
    return math.hypot(a.x - b.x, a.y - b.y)

def get_line_intersection(line1_start, line1_end, line2_start, line2_end, is_inf = 'SS'):
    """
    Получить точку пересечения отрезков или прямых

    is_inf задает ограничения на точку пересечения. Имеет вид 'AB', параметр A
    задает параметры первой прямой, B - второй.

    S(egment) - задан отрезок
    R(ay) - задан луч (начало - _start, направление - _end), точка пересечения валидна только
    если находится на луче _start-_end
    L(ine) - задана прямая
    """
    # Calculate the differences
    delta_x1 = line1_end.x - line1_start.x
    delta_y1 = line1_end.y - line1_start.y
    delta_x2 = line2_end.x - line2_start.x
    delta_y2 = line2_end.y - line2_start.y

    # Calculate the determinants
    determinant = delta_y1 * delta_x2 - delta_y2 * delta_x1

    if determinant == 0:
        # The lines are parallel or coincident
        return None

    # Calculate the differences between the start points
    delta_x_start = line1_start.x - line2_start.x
    delta_y_start = line1_start.y - line2_start.y

    # Calculate the t parameters
    t1 = (delta_x_start * delta_y2 - delta_x2 * delta_y_start) / determinant
    t2 = (delta_x_start * delta_y1 - delta_x1 * delta_y_start) / determinant

    intersection_x = line1_start.x + t1 * delta_x1
    intersection_y = line1_start.y + t1 * delta_y1
    p = Point(intersection_x, intersection_y)

    first_valid = False
    second_valid = False
    if is_inf[0] == 'S' and 0 <= t1 <= 1 or \
        is_inf[0] == 'R' and t1 >= 0 or \
        is_inf[0] == 'L':
        first_valid = True
    if is_inf[1] == 'S' and 0 <= t2 <= 1 or \
        is_inf[1] == 'R' and t2 >= 0 or \
        is_inf[1] == 'L':
        second_valid = True

    if first_valid and second_valid:
        return p

    return None

def vect_mult(v, u):
    """
    Посчитать модуль векторного произведения векторов v и u
    """
    return v.x * u.y - v.y * u.x

def scal_mult(v, u):
    """
    Посчитать скалярное произведение векторов v и u
    """
    return v.x * u.x + v.y * u.y

def rotate(p: Point, angle: float):
    """
    Повернуть вектор p на угол angle
    """
    return Point(p.x * math.cos(angle) - p.y * math.sin(angle),
                 p.y * math.cos(angle) + p.x * math.sin(angle))

def find_nearest_point(p, points, exclude = []):
    pc = None
    minDist = 10e10
    for i in range(0, len(points)):
        if points[i] in exclude:
            continue
        if dist(p, points[i]) < minDist:
            minDist = dist(p, points[i])
            pc = points[i]
    return pc

def find_nearest_robot(robot, team, avoid = []):
    """
    Найти ближайший робот из массива team к точке robot, игнорируя точки avoid
    """
    id = -1
    minDist = 10e10
    for i in range(0, len(team)):
        if i in avoid or not team[i].isUsed:
            continue
        if dist(robot, team[i].getPos()) < minDist:
            minDist = dist(robot, team[i].getPos())
            id = i
    return team[id]

def wind_down_angle(angle: float):
    """
    Привести угол к диапазону [-pi, pi]
    """
    angle = angle % (2*math.pi)
    if angle > math.pi:
        angle -= 2*math.pi
    return angle

def closest_point_on_line(point1, point2, point):
    """
    Получить ближайшую к точке point току на отрезке point1-point2
    """ 
    line_vector = (point2.x - point1.x, point2.y - point1.y)
    line_length = dist(point1, point2)
    
    if line_length == 0:
        return point1
    
    line_direction = (line_vector[0] / line_length, line_vector[1] / line_length)
    
    point_vector = (point.x - point1.x, point.y - point1.y)
    dot_product = point_vector[0] * line_direction[0] + point_vector[1] * line_direction[1]
    
    if dot_product <= 0:
        return point1
    elif dot_product >= line_length:
        return point2
    
    closest_point = Point(
        point1.x + line_direction[0] * dot_product,
        point1.y + line_direction[1] * dot_product
    )
    
    return closest_point

def closest_point_to_line(point1, point2, point):
    """
    Получить ближайшую к точке point току на прямой point1-point2
    """ 
    line_vector = (point2.x - point1.x, point2.y - point1.y)
    line_length = dist(point1, point2)
    
    if line_length == 0:
        return point1
    
    line_direction = (line_vector[0] / line_length, line_vector[1] / line_length)
    
    point_vector = (point.x - point1.x, point.y - point1.y)
    dot_product = point_vector[0] * line_direction[0] + point_vector[1] * line_direction[1]
    
    '''if dot_product <= 0:
        return point1
    elif dot_product >= line_length:
        return point2'''
    
    closest_point = Point(
        point1.x + line_direction[0] * dot_product,
        point1.y + line_direction[1] * dot_product
    )
    
    return closest_point

def point_on_line(robot, point, distance):
    """
    Получить точку на линии robot-point,
    отстоящую от точки robot на расстояние distance
    """
    angle_to_point = math.atan2(point.y - robot.y, point.x - robot.x)

    # Calculate the new point on the line at the specified distance from the robot
    new_x = robot.x + distance * math.cos(angle_to_point)
    new_y = robot.y + distance * math.sin(angle_to_point)
    return Point(new_x, new_y)
    
def LERP(p1, p2, t):
    """
    Получить линейно интерполированное значение
    """
    return p1*(1-t) + p2*t

def minmax(x, a, b):
    """
    Получить ближайшее к x число из диапазона [a, b]
    """
    return min(max(x, a), b)

def angle_to_point(point1, point2):
    """
    Получить угол вектора p = point2 - point1
    """
    return (point2-point1).arg()

def sign(num):
    """
    Получить знак числа num (0 если num == 0)
    """
    if num == 0:
        return 0
    return num / abs(num)

def det (a,b,c,d):
    """
    Получить определитель матрицы:
    |a b|
    |c d|
    """
    return a * d - b * c

def line_intersect (m, bots):
	result = []
	for n in bots:
		zn = det (m.A, m.B, n.A, n.B)
		res = Point(0, 0)
		if (abs (zn) < 1e-9):
			return None	
		res.x = - det (m.C, m.B, n.C, n.B) / zn
		res.y = - det (m.A, m.C, n.A, n.C) / zn
		result.append(res)
	return result

def probability(inter, bots, st):
    res = 1
    # print(len(inter), end = ' ')
    for i in range(len(inter)):
        koef = 1
        # print([inter[i].x, inter[i].y, bots[i].getPos().x, bots[i].getPos().y])
        tmpResX = dist(inter[i], bots[i].getPos())
        tmpResY = math.sqrt(dist(st, bots[i].getPos()) ** 2 - tmpResX ** 2)
        ang = math.atan2(tmpResY, tmpResX)
        # abs(ang) < math.pi / 4
        if abs(ang) > math.pi / 2:
            continue
        # print(tmpRes)
        '''if tmpResX < 0: 
            koef = 0
        elif tmpResX > const.ROBOT_R * 100 * 15:
            koef = 1 
        else:
            koef = tmpResX / (const.ROBOT_R * 100 * 15)'''
        # res *= (2 * abs(ang) / math.pi) * (dist(st, bots[i].getPos()) / 54e6) 
        res *= 1/(2 * abs(ang) / math.pi) 
    return res

def botPosition(st, vecx, vecy):
    modul = (vecx**2 + vecy**2)**(0.5)
    vecx = (vecx / modul) * const.ROBOT_R * 1000 * 2
    vecy = (vecy / modul) * const.ROBOT_R * 1000 * 2
    return Point(st.x - vecx, st.y - vecy)

def shotDecision(st, end, tobj):
    obj = tobj.copy()
    tmpCounter = 0
    for iter in range(len(obj)):
        if not obj[iter - tmpCounter].is_used():
            obj.pop(iter - tmpCounter)
            tmpCounter += 1
    # mx_shot_prob = 0
    shot_point = st
    mx = 0
    sum = Point(0, 0)
    n = 0
    # print(st)
    # for bot in obj:
    #     # print([bot.getPos().x, bot.getPos().y], end = " ")
    #     plt.plot(bot.getPos().x, bot.getPos().y, 'bo')
    # t = np.arange(-4500*1.0, 1000*1.0, 10)
    for point in end:    #checkai    
        A = -(point.y - st.y)
        B = (point.x - st.x)
        C = st.x * (point.y - st.y) - st.y * (point.x - st.x)
        tmpLine = bobLine(A, B, C)
        # plt.plot(t, (tmpLine.A*t + tmpLine.C)/tmpLine.B, 'g--')
        Lines = []
        for bot in obj:
            tmpC = -(B*bot.getPos().x - A*bot.getPos().y)
            L2 = bobLine(B, -A, tmpC)
            Lines.append(L2)
        inter = line_intersect(tmpLine, Lines)
        # plt.plot(inter[0].x, inter[0].y, 'bx')
        # plt.plot(inter[1].x, inter[1].y, 'gx')
        tmp_prob = probability(inter, obj, st)
        # print(tmp_prob, end = " ")
        if tmp_prob > mx:
            mx = tmp_prob
            shot_point = botPosition(st, point.x - st.x, point.y - st.y)
            point_res = point
        '''if tmp_prob > mx:
            mx = tmp_prob
            shot_point = botPosition(st, point.x - st.x, point.y - st.y)
            point_res = point
            n = 1
            sum = point
        elif tmp_prob == mx:
            sum += point
            n += 1
        else:
            point_res = sum / n
            shot_point = botPosition(st, point_res.x - st.x, point_res.y - st.y)
            sum = Point(0, 0)
            n = 0'''
    # plt.plot(t, -(point_res.A*t + point_res.C)/point_res.B, 'r-')
    # plt.plot(shot_point.x, shot_point.y, 'r^')
    # plt.axis('equal')
    # plt.grid(True)
    # plt.show()
    return shot_point, mx, point_res
    
def in_place(st, end, epsilon):
    """
    Проверить, находится ли точка st в радиусе epsilon около end
    """
    if ((st - end).mag() < epsilon):
        return True
    else:
        return False

def is_in_list(arr, x):
    """
    Узнать, есть ли элемент x в списке arr
    """
    for i in arr:
        if i == x:
            return True
    return False
