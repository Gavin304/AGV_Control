import os
import sys
import struct

class Mode():
    def __init__(self):
        self.head = 0x23
        self.id = 0x01
        self.func = 0x20
        self.len = 0x01
        self.mode = 0x06
        self.cks = 0

    def getMode(self, mode):
        if mode == 0x06:
            self.mode = 0x06
        elif mode == 0x01: #all directions
            self.mode = 0x01
        elif mode == 0x04: #move in y direction
            self.mode = 0x04
        elif mode == 0x03:
            self.mode = 0x03
        elif mode == 0x07:
            self.mode = 0x07
        elif mode == 0x00:
            self.mode = 0x00
        else:
            print("Invalid mode")
            return 0x06
        
        control_code = self.sendMode()
        return control_code
    
    def sendMode(self):
        control_code = bytes([self.head, self.id, self.func, self.len, self.mode])
        cks = calculateCRC16(control_code)
        return control_code + cks
    

class Runbytes:
    def __init__(self, vx, vy, w, acc, distance_x, distance_y, heartbeat, radius, arc_distance):
        self.vx = vx
        self.vy = vy
        self.w = w
        self.heartbeat = heartbeat
        self.acc = acc
        
        self.distance_x = distance_x
        self.distance_y = distance_y
        self.radius = radius
        self.arc_distance = arc_distance
        

        self.data0 = 0x23
        self.data1 = 0x01
        self.data2 = 0x2E
        self.data3 = 0x0B
        # self.data10 = 0x02
        # self.data11 = 0x00
        self.data12 = 0x00
        self.data13 = 0x0A
        # self.radius = 100
        self.cks = 0

    def to_byte(self, mode):
        if mode == 0x06:
            header = bytes([self.data0, self.data1, self.data2, self.data3,
                            *struct.pack('>hhhh', self.vx, self.vy, self.w, self.acc), self.data12, self.data13, self.heartbeat])
        elif mode == 0x01:
            header = bytes([self.data0, self.data1, 0x23, 0x06, *struct.pack('>hhh', self.distance_x, 0, 80)])
            self.heartbeat = 0

        elif mode == 0x04:
            # header = bytes([self.data0, self.data1, 0x23, 0x04, *struct.pack('>h', self.distance_y)])
            # self.heartbeat = 0
            print('vy: ', self.vy, 'acc: ', self.acc, 'heartbeat: ', self.heartbeat)
            print(*struct.pack('>hh', self.vy, self.acc))
            header = bytes([self.data0, self.data1, 0x2B, 0x05, *struct.pack('>hh', self.vy, self.acc), self.heartbeat])
            # print("0x04")
        elif mode == 0x03:
            header = bytes([self.data0, self.data1, 0x29, 0x04, *struct.pack('>hh', self.w, 25)])
            # print("0x03")
            self.heartbeat = 0
        elif mode == 0x07:
            if self.radius > 0:
                self.radius = self.radius *-1
            # print('radius =', self.radius, '  distance = ', self.arc_distance)
            header = bytes([self.data0, self.data1, 0x32, 0x06, *struct.pack('>hhh', self.arc_distance, self.radius, 50)])
            # header = bytes([self.data0, self.data1, 0x32, 0x06, *struct.pack('>hhh', 500, -721, 50)])
            
            # header = bytes([self.data0, self.data1, 0x32, 0x04, *struct.pack('>hh', -500, 900)])
            # header = bytes([self.data0, self.data1, 0x31, 0x09,
            #                 *struct.pack('>hhhh', self.vx, self.w*100 , self.acc, self.radius), self.heartbeat])
            # print(header)

            # print('vx: ', self.vx, 'w: ', self.w, 'acc: ', self.acc, 'heartbeat: ', self.heartbeat, 'radius: ', self.radius)
            self.heartbeat = 0
        
        elif mode == 0x00:
            header = bytes([self.data0, self.data1, 0x50, 0x00])

        else:
            print("Invalid mode")
            return
        cks = calculateCRC16(header)
        return header + cks

def calculateCRC16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, byteorder='little')