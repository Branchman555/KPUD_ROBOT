import cv2
import random
import pandas as pd
import numpy as np
from numpy.random import randn
from scipy.stats import bernoulli
import threading
import math
import requests, json
import GeoConverter as geo
from pyfiglet import Figlet

from time import sleep

# calculate relative degree
def calreldeg(src_deg, tar_deg):
    rel_deg = tar_deg - src_deg
    if rel_deg > 180:
        rel_deg -= 360
    elif rel_deg < -180:
        rel_deg += 360
    return rel_deg

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

BACKGROUND_PATH = './bg_0.jpg'
BG_WIDTH = 2048
BG_HEIGHT = 2048

referenceOrigin = (126.730667, 37.342222)



class SimulWindow:
    _default_width = 512
    _default_height = 512
    _default_x = 400
    _default_y = 400
    _default_version = 0.1
    _default_background = cv2.resize(cv2.imread(BACKGROUND_PATH), (BG_WIDTH, BG_HEIGHT), interpolation=cv2.INTER_CUBIC)
    _default_win = _default_background[
                    _default_y:_default_y + _default_height,
                    _default_x:_default_x + _default_width].copy()

    def __init__(self, name='default'):
        self._width = SimulWindow._default_width
        self._height = SimulWindow._default_height
        self._x = self._default_x
        self._y = self._default_y
        self._winname = name
        self._img = SimulWindow._default_background.copy()
        self._shimg = SimulWindow._default_win
        self._winstopflag = True
        self._endstayflag = False

        #cv2.resizeWindow(self.__winname, (200, 200))
    def __del__(self):
        print("+++++++++++++++++++++SimulWindow Disabled+++++++++++++++++++++")

    def writewindow(self):
        SimulWindow._default_background = self._img.copy()

    # def getwindow(self):
    #     return self.__img

    def movewindow(self, x, y):
        x = int(x)
        y = int(y)
        if x <= 0:
            self._x = 0
        if 0 < x < BG_WIDTH - self._width:
            self._x = x
        if x >= BG_WIDTH - self._width:
            self._x = BG_WIDTH - self._width

        if y <= 0:
            self._y = 0
        if 0 < y < BG_HEIGHT - self._height:
            self._y = y
        if y >= BG_HEIGHT - self._height:
            self._y = BG_HEIGHT - self._height

        self._shimg = self._img[self._y:self._y + self._height, self._x:self._x + self._width]

    def getwindow(self):
        self._img = SimulWindow._default_background.copy()
        return self._img

    def showwindow(self):
        while self._winstopflag is False:
        #while 1:
            cv2.imshow(self._winname, self._shimg)
            cv2.waitKey(100)
        print("outto!!!")

    def resetwindow(self):
        self._width = SimulWindow._default_width
        self._height = SimulWindow._default_height
        self._img = SimulWindow._default_win

    def startwindow(self):
        cv2.namedWindow(self._winname)
        cv2.imshow(self._winname, self._shimg)
        print("called st_win")
        if self._winstopflag:
            self._winstopflag = False
            print("window start")
            thread = threading.Thread(target=self.showwindow)
            # thread = threading.Thread(target=stt)
            thread.daemon = True
            thread.start()

    def endwindow(self):
        self._winstopflag = True
        if self._endstayflag is False:
            self.closewindow()

    def closewindow(self):
        cv2.destroyWindow(self.getwindowname())

    def setwindowname(self, winname):
        self._winname = winname

    def getwindowname(self):
        return self._winname

    def setsize(self, width, height):
        self._width = width
        self._height = height

    def getsize(self):
        return self._width, self._height


class Robot:
    _robot_num = 0
    _focusRobot = -1
    _guiLock = False
    _sm = SimulWindow()

    def __init__(self, gui=True):
        self._guiMode = gui
        self._robot_num = Robot._robot_num
        self._x = 500
        self._y = 500
        self._angle = 0
        self._shape = "circle"
        self._speed = 1
        self._focusFlag = 0

        self._color = BLACK
        self._thickness = 1
        self._drawflag = False
        self._lineflag = False

        self._version = 1.0
        if not Robot._guiLock:
            if gui:
                Robot._sm.startwindow()
            else:
                Robot._guiLock = True
                del Robot._sm
        else:
            print("==== GUI MODE IS OFFED ====")
            self._guiMode = False

        Robot._robot_num += 1

    def endsimul(self):
        if self._guiMode:
            Robot._sm.endwindow()


    def showrobotnum(self):
        print(Robot._robot_num)

    def setshape(self, str):
        self._shape = str

    def setFocus(self):
        if Robot._focusRobot == -1:
            Robot._focusRobot = self._robot_num
            self._focusFlag = True
        else:
            print("Release Focus First!!!")

    def releaseFocus(self):
        if Robot._focusRobot == self._robot_num:
            Robot._focusRobot = -1
            self._focusFlag = False
        else:
            print("Focus First!!!")

    def drawon(self):
        if self._guiMode:
            self._drawflag = True
            self._lineflag = True
            self.drawRobot()
        else:
            print("==== GUI MODE IS OFFED ====")

    def drawoff(self):
        self._drawflag = False
        self._lineflag = False

    def dot(self, dot_size, color):
        if self._guiMode:
            access_img = Robot._sm.getwindow()
            cv2.circle(access_img, (int(self._x), int(self._y)), dot_size, color, -1)
            Robot._sm.writewindow()
        else:
            print("==== GUI MODE IS OFFED ====")

    def circle(self, circle_size):
        if self._guiMode:
            access_img = Robot._sm.getwindow()
            cv2.circle(access_img, (int(self._x), int(self._y)), circle_size, self._color, self._thickness)
            Robot._sm.writewindow()
        else:
            print("==== GUI MODE IS OFFED ====")

    def drawRobot(self):
        if self._guiMode:
            access_img = Robot._sm.getwindow()
            cv2.circle(access_img, (int(self._x), int(self._y)), int(self._thickness / 2), self._color, -1)
            if self._drawflag:
                Robot._sm.writewindow()
        else:
            print("==== GUI MODE IS OFFED ====")

    def lineon(self):
        self._lineflag = True

    def lineoff(self):
        self._lineflag = False

    def setspeed(self, speed):
        self._speed = speed

    def goto(self, tar_x, tar_y):
        bef_x = self._x
        bef_y = self._y
        self._x = round(tar_x, 3)
        self._y = round(tar_y, 3)
        if self._guiMode:
            self.drawRobot()
            if self._drawflag:
                if self._lineflag:
                    access_img = Robot._sm.getwindow()
                    cv2.line(access_img, (int(bef_x), int(bef_y)), (int(self._x), int(self._y)), self._color, self._thickness)
                    Robot._sm.writewindow()
            if self._speed:
                cv2.waitKey(50 * self._speed)
            else:
                cv2.waitKey(1)

            if Robot._focusRobot == -1:
                print("Please Focus Robot At Least One!!!")

            if self._focusFlag:
                self._sm.movewindow(self._x - 256, self._y - 256)

    def forward(self, distance):
        radian = math.radians(self._angle)
        self.goto(self._x + distance * math.cos(radian), self._y + distance * math.sin(radian))

    def position(self):
        return self._x, self._y

    def distance(self, node):
        return math.sqrt((self._x - node[0]) ** 2 + (self._y - node[1]) ** 2)

    def setsize(self, thickness):
        self._thickness = thickness

    def setcolor(self, BGR_Code):
        self._color = BGR_Code

    def getcolor(self):
        return self._color

    def setangle(self, tar_angle):
        self._angle = tar_angle
        #angle is Degree

    def getangle(self):
        return self._angle

    def calangle(self, tar_point_x=0.0, tar_point_y=0.0):
        relative_loc = (tar_point_x - self._x, tar_point_y - self._y)
        resdeg = 0
        if relative_loc[0] == 0:
            if relative_loc[1] == 0:
                resdeg = 0
            elif relative_loc[1] > 0:
                resdeg = 90
            elif relative_loc[1] < 0:
                resdeg = 270
        elif relative_loc[1] == 0:
            if relative_loc[0] > 0:
                resdeg = 0
            elif relative_loc[0] < 0:
                resdeg = 180
        else:
            resdeg += math.degrees(math.atan(relative_loc[1] / relative_loc[0]))

        if relative_loc[0] > 0 and relative_loc[1] > 0:
            pass
        elif relative_loc[0] < 0 and relative_loc[1] > 0:
            resdeg += 180
        elif relative_loc[0] < 0 and relative_loc[1] < 0:
            resdeg += 180
        elif relative_loc[0] > 0 and relative_loc[1] < 0:
            resdeg += 360
        return resdeg

    def diffangle(self, tar_angle):
        diff = tar_angle - self._angle
        if diff > 180:
            diff -= 360
        elif diff < -180:
            diff += 360
        return diff

    def addangle(self, angle):
        res = self.getangle() + angle
        if res >= 360:
            res -= 360
        elif res < 0:
            res += 360
        self.setangle(res)

    def show_version(self):
        print("===============================================================")
        f = Figlet(font='colossal')
        print(f.renderText('KPU Delivery Robot ver.{}'.format(self._version)))
        print("===============================================================\n\n")


def cal_line_equ(nodes):
    equation = [0]
    for i in range(len(nodes) - 1):
        if nodes[i][1] - nodes[i + 1][1] == 0:
            val_a = 0
            val_b = 1
            val_c = -1 * nodes[i][1]
        elif nodes[i][0] - nodes[i + 1][0] == 0:
            val_a = 1
            val_b = 0
            val_c = -1 * nodes[i][0]
        else:
            val_a = nodes[i][1] - nodes[i + 1][1]
            val_b = nodes[i + 1][0] - nodes[i][0]
            val_c = nodes[i][0] * nodes[i + 1][1] - nodes[i + 1][0] * nodes[i][1]
            if val_a < 0:
                val_a *= -1
                val_b *= -1
                val_c *= -1
        equation.append((val_a, val_b, val_c))
    return equation

def cal_point_line(point, line):
    distance = abs(line[0] * point[0] + line[1] * point[1] + line[2])/math.sqrt(line[0]**2 + line[1]**2)
    return distance


if __name__ == "__main__":

    URL = 'http://1.255.54.9:3000/getPath'
    # data = {"start":55,"end":20,"name":'"Johnny"'}
    data2 = 'start=24&end=43&name=%22TEST_DRIVE%22'
    # res = requests.get(URL, data=data)
    res = requests.post(URL, data=data2, headers={"content-type": "application/x-www-form-urlencoded"})
    res_dict = json.loads(res.text)

    node_raw = list(res_dict.values())
    nodes = []
    for cont in node_raw:
        templist = list(map(float, cont.split(',')))
        pt1 = geo.GeoPoint(templist[1], templist[0])
        output = geo.convert(geo.GEO, geo.TM, pt1)
        nodes.append([output.getX(), output.getY()])
    for cont in nodes:
        cont[0] = int((cont[0] - 176066.3519693286) * 4)
        cont[1] = int((cont[1] - 426729.25143377006) * -4)

    print(nodes)

    t_GPS = Robot()
    t = Robot()
    t.setFocus()
    t_GPS.setcolor(RED)
    t_GPS.drawoff()
    t.setshape("triangle")

    t.setspeed(1)
    t_GPS.setspeed(1)
    mark_radius = 20

    GPS_ERR_VAL = 20
    ANG_ERR_VAL = 0.5
    VEL_ERR_VAL = 0.2
    NODE_CORR = 0.8

    GPS_WEIGHT = 0.15
    ROUTE_WIDTH = 10

    #nodes = [(500, 500), (600, 600), (600, 800), (700, 800), (800, 600)]

    lines = cal_line_equ(nodes)
    print(lines)

    t_GPS.setsize(ROUTE_WIDTH)

    t_GPS.setcolor(GREEN)
    for index, node in enumerate(nodes):
        t.goto(node[0], node[1])
        t_GPS.goto(node[0], node[1])
        t_GPS.drawon()
        t.drawon()
        #t.write(str(index))
        t.circle(mark_radius)

    t.drawoff()
    t_GPS.drawoff()

    t.setspeed(1)
    t_GPS.setspeed(1)

    t.setsize(4)
    t.setcolor(BLACK)
    t_GPS.setcolor(RED)
    t_GPS.setsize(1)
    t_GPS.goto(t.position()[0], t.position()[1])
    t.drawon()
    t_GPS.drawoff()

    for index, node in enumerate(nodes):
        if index == 0:
            t.drawoff()
            t_GPS.drawoff()
            t.goto(node[0], node[1])
            t_GPS.goto(node[0], node[1])
            t.drawon()
            continue

        delta = 0
        err_count = 0
        arrival_count = 0
        vision_count = 0
        vision_angle = 0

        # 초기 방향 잡아주기 및 카운트 초기화
        target_tilt = t_GPS.calangle(node[0], node[1])
        t.setangle(target_tilt + randn() * ANG_ERR_VAL)
        t_GPS.setangle(target_tilt)
        corr_angle_vis = 0
        while 1:
            # 오류와 사기가 판치는 공간의 시작
            # head = bernoulli.rvs(0.5)
            # if head is 1:
            #     rnd = random.randint(-30, 30)
            #     t.right(rnd)
            #     t_GPS.right(rnd + randn() * ANG_ERR_VAL)

            # 카운트에 맞춰서 점찍기, 랜덤 오류발생
            if err_count % 4 is 0:
                err_GPS = (t.position()[0] + randn() * GPS_ERR_VAL, t.position()[1] + randn() * GPS_ERR_VAL)
                cor_GPS = (GPS_WEIGHT * err_GPS[0] + (1 - GPS_WEIGHT) * t_GPS.position()[0],
                           GPS_WEIGHT * err_GPS[1] + (1 - GPS_WEIGHT) * t_GPS.position()[1])

                t_GPS.goto(err_GPS[0], err_GPS[1])
                t_GPS.dot(2, RED)

                t_GPS.goto(cor_GPS[0], cor_GPS[1])
                t_GPS.dot(4, BLUE)

            t_GPS.setangle(t.getangle())
            if t_GPS.distance(node) < mark_radius:
                print("TESSSSTTTTT")
                if arrival_count > 3:
                    t.forward(2)
                    t_GPS.forward(2 + randn() * VEL_ERR_VAL)
                    arrival_count = 0
                    t_GPS.goto((1 - NODE_CORR) * err_GPS[0] + NODE_CORR * node[0],
                               (1 - NODE_CORR) * err_GPS[1] + NODE_CORR * node[1])
                    break
                arrival_count += 1
            # 오류와 사기가 판치는 공간의 끝

            # 각도 변경파트: GPS 기반
            target_tilt = t.calangle(node[0], node[1])
            print("tar_tilt : " + str(target_tilt))
            tar_angle = t.diffangle(target_tilt)
            print("tar_angle : " + str(tar_angle))
            # 랜덤 자율주행
            if vision_count > 4:
                vision_count = 0
                vision_angle = tar_angle + randn() * 20

                head = bernoulli.rvs(0.5)
                if head is 1:
                    rnd = random.randint(-30, 30)
                    vision_angle += rnd

                head = bernoulli.rvs(0.5)
                if head is 1:
                    delta = 1
                else:
                    delta = -1

            # 자율주행 각도 보정
            #corr_angle_vis = 0
            if vision_angle > 0:
                if vision_angle > 30:
                    corr_angle_vis = 15
                else:
                    corr_angle_vis = 5
            elif vision_angle < 0:
                if vision_angle < -30:
                    corr_angle_vis = -15
                else:
                    corr_angle_vis = -5
            corr_angle_vis += delta * 2

            # GPS 각도 보정
            corr_angle_gps = 0
            if tar_angle > 0:
                if tar_angle > 45:
                    corr_angle_gps = 45
                elif tar_angle > 15:
                    corr_angle_gps = 30
                else:
                    corr_angle_gps = 25
            elif tar_angle < 0:
                if tar_angle < -45:
                    corr_angle_gps = -45
                elif tar_angle < -15:
                    corr_angle_gps = -30
                else:
                    corr_angle_gps = -25

            dist_route = cal_point_line(t_GPS.position(), lines[index])
            if dist_route > ROUTE_WIDTH / 2 + 10:
                print("++++++++++++++EMERGENCY CALL!!!++++++++++++++")
                target_tilt = t.calangle(node[0], node[1])
                tar_angle = t.diffangle(target_tilt)
                if tar_angle > 0:
                    t.addangle(tar_angle + 90)
                    t_GPS.addangle(tar_angle + 90)
                elif tar_angle < 0:
                    t.addangle(tar_angle - 90)
                    t_GPS.addangle(tar_angle - 90)

                for i in range(7):
                    t.forward(2 + randn() * VEL_ERR_VAL)
                    t_GPS.forward(2)
                continue
            elif dist_route < ROUTE_WIDTH / 2:
                t.addangle(corr_angle_vis + randn() * ANG_ERR_VAL)
                t_GPS.addangle(corr_angle_vis)
                print("t.angle : " + str(t.getangle()))
                print("mode : vision")
            else:
                t.addangle(corr_angle_gps + randn() * ANG_ERR_VAL)
                t_GPS.addangle(corr_angle_gps)
                print("t.angle : " + str(t.getangle()))
                print("mode : GPS filter")

            # 랜덤 속도 감속
            head = bernoulli.rvs(0.1)
            if head is 1:
                t.forward(2 + randn() * VEL_ERR_VAL)
                t_GPS.forward(2)
            else:
                t.forward(4 + randn() * VEL_ERR_VAL)
                t_GPS.forward(4)

            if err_count % 20 is 0:
                # t.write(str(round(t.distance(t_GPS.position()), 2)))
                print("ED : ", str(t.distance(t_GPS.position())))
            err_count += 1
            vision_count += 1



    # myRobot1 = Robot()
    # myRobot1.setcolor(150, 0, 40)
    # #myRobot1.drawon()
    # myRobot1.goto(50, 50)
    #
    #
    # myRobot1.showrobotnum()
    # myRobot2 = Robot()
    # myRobot1.showrobotnum()
    # myRobot2.showrobotnum()
    #
    # # sm = SimulWindow()
    # # sm.startwindow()
    # myRobot3 = Robot()
    # myRobot3.showrobotnum()
    # while 1:
    #     key = cv2.waitKey(20)
        # if key == ord('a'):
        #     print('a')
        #     myRobot1.setangle(180)
        #     myRobot1.forward(50)
        # if key == ord('d'):
        #     print('d')
        #     myRobot1.setangle(0)
        #     myRobot1.forward(50)
        # if key == ord('w'):
        #     print('w')
        #     myRobot1.setangle(90)
        #     myRobot1.forward(50)
        # if key == ord('s'):
        #     print('s')
        #     myRobot1.setangle(270)
        #     myRobot1.forward(50)
        # if key == ord('x'):
        #     break

        # if key == ord('y'):
        #     myRobot1.drawon()
        #
        # if key == ord('z'):
        #     myRobot1.lineon()
        #
        # if key == ord('p'):
        #     myRobot1.lineoff()
        #
        # if key == ord('o'):
        #     myRobot1.drawoff()
    print("\n\nENDTEST\n\n\n")
    cv2.waitKey(1)
    print("\n\nENDTEST\n\n\n")
    t_GPS.endsimul()
    t.endsimul()
    print("\n\nENDTEST\n\n\n")
    #myRobot4 = Robot()
    #myRobot4.showrobotnum()




    # myRobot = Robot()
    #
    # print(myRobot.position())
    # myRobot.goto(0, 0)
    # print(myRobot.position())
    #
    # myRobot.show_version()
    #
    # print(myRobot.calangle(1, 1))
    # myRobot.setangle(myRobot.calangle(1, 1))
    #
    # print(calreldeg(myRobot.getangle(), 200))
    # myRobot.setangle(calreldeg(myRobot.getangle(), 200) + myRobot.getangle())
    #
    # print(calreldeg(myRobot.getangle(), 200))
