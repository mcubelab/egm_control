#!/usr/bin/env python
import time
import socket
import helpers.egm_pb2 as egm_pb2

if __name__=='__main__':
    UDP_PORT = 6510
    sequenceNumber = 0
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", UDP_PORT))

    sock.sendto("test", ("172.16.9.128", 6510))

    while True:
        data, addr = sock.recvfrom(1024)
        egm_sensor = egm_pb2.EgmSensor()
        egm_sensor.ParseFromString(data)
        print(egm_sensor.planned.cartesian)
