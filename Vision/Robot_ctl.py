import cv2
import random
import pandas as pd
import numpy as np
from numpy.random import randn
from scipy.stats import bernoulli
import math
import requests, json
import GeoConverter as geo
import argparse
import tensorflow as tf
from tensorflow.keras.models import model_from_json
import serial
import helper
from MyRobot import *
from multiprocessing import Process, Queue
import threading
import Robot_ser

from time import sleep

# calculate relative degree

RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

mark_radius_1 = 20
mark_radius_2 = 14

GPS_ERR_VAL = 20
ANG_ERR_VAL = 0.5
VEL_ERR_VAL = 0.2
NODE_CORR = 0.8

GPS_WEIGHT = 0.15
ROUTE_WIDTH = 10

VIS = 0
GPS = 1

lock = threading.Lock()
referenceOrigin = (126.730667, 37.342222)


def callNodes(URL, start, end):
    data = 'start={}&end={}&name=%22TEST_DRIVE%22'.format(start, end)
    # res = requests.get(URL, data=data)
    res = requests.post(URL, data=data, headers={"content-type": "application/x-www-form-urlencoded"})
    res_dict = json.loads(res.text)

    node_raw = list(res_dict.values())
    nodes = []
    for cont in node_raw:
        templist = list(map(float, cont.split(',')))
        pt1 = geo.GeoPoint(templist[1], templist[0])
        output = geo.convert(geo.GEO, geo.TM, pt1)
        nodes.append([round((output.getX() - 176066.3519693286) * 4, 3),
                      round((output.getY() - 426729.25143377006) * -4, 3)])
    return nodes


class Rover(Robot):
    def __init__(self):
        super().__init__(gui=True)
        self.mode = 0
        self._forwardFlag = 0
        self._gpsX = 0
        self._gpsY = 0
        self._gpsTime = 0
        self.endComuFlag = 0
        self.comuProcs = []
        self.comuQ_1 = Queue()
        self.comuQ_2 = Queue()

    def getGPS(self):
        return self._gpsX, self._gpsY

    # SENSOR BOARD
    def RUart_1(self):
        temp = 0
        while not self.endComuFlag:
            uartRet = Robot_ser.serProtocol(self.ser_1)
            while not self.comuQ_1.empty():
                temp = self.comuQ_1.get_nowait()
            self.comuQ_1.put(uartRet)

            # output = geo.convert(geo.GEO, geo.TM, geo.GeoPoint(uartRet[2], uartRet[1]))
            # self._gpsTime = uartRet[1]
            # self._gpsX = output.getX()
            # self._gpsY = output.getY()

    # MOTOR BOARD
    def RUart_2(self):
        while not self.endComuFlag:
            uartRet = Robot_ser.serProtocol(self.ser_2)
            while not self.comuQ_2.empty():
                temp = self.comuQ_2.get_nowait()
            self.comuQ_2.put(uartRet)

    def syncValue(self):
        while not self.endComuFlag:
            if not self.comuQ_1.empty():
                uartRet = self.comuQ_1.get_nowait()
                if uartRet[0] == "BNO":
                    self.setangle(uartRet[1])
                    if self._forwardFlag:
                        self.forward(0.2)
                elif uartRet[0] == "GPS":
                    output = geo.convert(geo.GEO, geo.TM, geo.GeoPoint(uartRet[2], uartRet[1]))
                    self._gpsTime = uartRet[1]
                    self._gpsX = output.getX()
                    self._gpsY = output.getY()
                    self.goto(GPS_WEIGHT * self._gpsX + (1 - GPS_WEIGHT) * self._x,
                              GPS_WEIGHT * self._gpsY + (1 - GPS_WEIGHT) * self._y)
            cv2.waitKey(10)

    def startRUart(self, UartNum):
        if UartNum is 0:
            print("SYNC")
            self.syncThread = threading.Thread(target=self.syncValue)
            self.syncThread.start()
        elif UartNum is 1:
            self.ser_1 = Robot_ser.serInit('/dev/ttyUSB0', 115200)
            proc = Process(target=self.RUart_1, args=(self.comuQ_1,))
            proc.start()
            self.comuProcs.append(proc)
        elif UartNum is 2:
            self.ser_2 = Robot_ser.serInit('/dev/ttyUSB1', 115200)
            proc = Process(target=self.RUart_2, args=(self.comuQ_2,))
            proc.start()
            self.comuProcs.append(proc)


    def endProcess(self):
        self.endComuFlag = 1
        for proc in self.comuProcs:
            proc.join()

    def getMotorSpeed(self):
        pass

    def roverMoveStart(self):
        self._forwardFlag = True

    def roverMoveStop(self):
        self._forwardFlag = False

    def roverMoveForward(self, distance):
        node = self.position()
        while self.distance(node) > distance:
            # =======================Serial Write===========================
            dataFormat = "<255,500,1>".format(INT_steering)
            self.ser_2.write(dataFormat.encode())
            cv2.waitKey(20)

    def roverMovePointTurn_rel(self, relAngle):
        res = self.getangle() + relAngle
        if res >= 360:
            res -= 360
        elif res < 0:
            res += 360
        absAngle = res
        diff = self.diffangle(absAngle)
        while abs(absAngle - self.getangle()) > 2:
            if diff > 0:
                INT_steering = -100
            else:
                INT_steering = +100
            # =======================Serial Write===========================
            dataFormat = "<{},255,1>".format(INT_steering)
            self.ser_2.write(dataFormat.encode())
            cv2.waitKey(20)

    def roverMovePointTurn_abs(self, absAngle):
        diff = self.diffangle(absAngle)
        while abs(absAngle - self.getangle()) > 2:
            if diff > 0:
                INT_steering = -150
            else:
                INT_steering = +150
            # =======================Serial Write===========================
            dataFormat = "<{},255,1>".format(INT_steering)
            self.ser_2.write(dataFormat.encode())
            cv2.waitKey(20)


if __name__ == "__main__":

    Optimize_number = 0.14

    # 카메라 이니셜라이징
    print("Camera Initializing...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Unable to read camera feed")
    key = 0

    # 시리얼 포트 연결

    # 딥러닝 용 코드, 딥러닝 모델 로딩과정 - 약간의 시간소요
    parser = argparse.ArgumentParser(description='Remote Driving')
    parser.add_argument('model', type=str,
                        help='Path to model definition json. Model weights should be on the same path.')
    args = parser.parse_args()
    with open(args.model, 'r') as jfile:
        model = model_from_json(json.load(jfile))

    model.compile("adam", "mse")
    weights_file = args.model.replace('json', 'h5')
    model.load_weights(weights_file)

    print("Ready... Go?")
    input()
    print("Program Started...!")

    nodes = callNodes('http://1.255.54.9:3000/getPath', 24, 43)
    print(nodes)

    # 로봇 초기 셋팅
    exit = False

    thisRobot = Rover()
    thisRobot.show_version()
    # 로봇 통신 컨피규레이션
    thisRobot.startRUart(0)
    thisRobot.startRUart(1)
    thisRobot.startRUart(2)

    sleep(2)

    countCurrentNode = 0
    nodes = [thisRobot.getGPS()] + nodes
    lines = cal_line_equ(nodes)
    numEntireNodes = len(nodes)

    currentNode = nodes[countCurrentNode]
    thisRobot.goto(currentNode[0], currentNode[1])
    countCurrentNode += 1
    currentNode = nodes[countCurrentNode]

    thisRobot.roverMoveStop()
    thisRobot.setFocus()
    thisRobot.setcolor(RED)
    thisRobot.drawon()

    while not exit:
        cv2.waitKey(1000)
        tar_angle = thisRobot.calangle(currentNode[0], currentNode[1])
        thisRobot.roverMovePointTurn_abs(tar_angle)
        # thisRobot.roverMoveStart()
        cv2.waitKey(1000)
        while not exit:
            # 카메라 수신
            if thisRobot.distance(currentNode) < mark_radius_1:
                thisRobot.roverMoveForward(0.2)
                if thisRobot.distance(currentNode) < mark_radius_2:
                    thisRobot.roverMoveStop()
                    countCurrentNode += 1
                    currentNode = nodes[countCurrentNode]
                    break
                continue

            dist_route = cal_point_line(thisRobot.position(), lines[countCurrentNode])

            if dist_route > (ROUTE_WIDTH / 2 + 10):
                thisRobot.roverMoveStop()
                print("++++++++++++++EMERGENCY CALL!!!++++++++++++++")
                target_tilt = thisRobot.calangle(currentNode[0], currentNode[1])
                diff = thisRobot.diffangle(target_tilt)
                if diff > 0:
                    thisRobot.roverMovePointTurn_rel(80)
                elif diff < 0:
                    thisRobot.roverMovePointTurn_rel(-80)
                #thisRobot.roverMoveForward(10)
                #thisRobot.roverMoveStart()
                continue
            elif dist_route < ROUTE_WIDTH / 2:
                thisRobot.mode = VIS
            else:
                thisRobot.mode = GPS

            if thisRobot.mode is VIS:
                ret, frame = cap.read()
                image_array = np.asarray(frame)
                image_array = helper.crop(image_array, 0.5, 0.2)

                image_array = helper.resize(image_array, new_dim=(64, 64))
                HSV_image_array = cv2.cvtColor(image_array, cv2.COLOR_BGR2HSV)
                transformed_image_array = HSV_image_array[None, :, :, :]
                transformed_image_array = tf.cast(transformed_image_array, tf.float32)
                steering_angle = float(model.predict(transformed_image_array, batch_size=1))

                INT_steering = int(steering_angle * 320)
                if INT_steering > 250:
                    INT_steering = 250
                elif INT_steering < -250:
                    INT_steering = -250
                INT_steering += 250
                # =======================Serial Write===========================
                dataFormat = "<{},510,1>".format(INT_steering)
                thisRobot.ser_2.write(dataFormat.encode())
            elif thisRobot.mode is GPS:
                tar_angle = thisRobot.calangle(currentNode[0], currentNode[1])
                diff = thisRobot.diffangle(tar_angle)
                INT_steering = 0
                if diff > 0:
                    INT_steering = -250
                elif diff < 0:
                    INT_steering = 250
                INT_steering += 250
                dataFormat = "<{},510,1>".format(INT_steering)
                thisRobot.ser_2.write(dataFormat.encode())

            key = cv2.waitKey(1000)

        # 키보드 메시지
        if key == ord('a'):
            print('a')
        if key == ord('x'):
            thisRobot.endComuFlag = False
            dataFormat = "<250,255,3>"
            # =======================Serial Write===========================
            thisRobot.ser_2.write(dataFormat.encode())
            print(dataFormat)
            thisRobot.endProcess()
            break

    thisRobot.endsimul()