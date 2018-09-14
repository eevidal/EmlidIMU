# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 15:47:57 2018

@author: erica
"""

import socket

UDP_IP = "192.168.88.14"
UDP_PORT = 9999
 
sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

while True:
  data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
  print "received message:", data