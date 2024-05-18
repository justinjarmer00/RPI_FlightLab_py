# -*- coding: utf-8 -*-
"""
Created on Tue Jun 13 13:11:21 2023

@author: justinjarmer
"""
import os

def readXbee(xbee):
    try:
        if xbee.in_waiting > 0:
            message = xbee.readline().decode('utf-8').strip()
        else:
            message = 'none'
    except UnicodeDecodeError:
        message = 'error could_not_decode'
    return message

def sendXbee(xbee, reply):
    newline = b'|||'  # Byte representation of newline
    reply = reply + newline # reply is already a byte array
    xbee.write(reply)
    
def mkdir(path, name):
    directory_name = path + name
    os.makedirs(directory_name, exist_ok=True)
    
def calculateCG(FW, BLW, BRW):
    """ KNOWN VALUES """
    distF = [0,0] #front wheel
    distF[0] = 6.375 #distance from the nose
    distF[1] = 0 #distance from the centerline

    distLR = [0,0] #left rear wheel
    distLR[0] = 30.375
    distLR[1] = -12.01

    distRR = [0,0] #right rear wheel
    distRR[0] = 30.375
    distRR[1] = 12.01

    """CALCULATIONS"""
    reply = ''
    try:
        CG = [0,0]
        for i in range(2):
            CG[i] = (FW*distF[i] + BLW*distLR[i] + BRW*distRR[i])/(FW + BLW + BRW)
        rep1 = "CG from nose (cm):\t" + str(CG[0])
        rep2 = "\nCG from centerline (cm):\t" + str(CG[1])
        
        mac = 16.4417
        np = 30.17
        rep3 = "\nStatic Margin:\t" + str((np - CG[0])/mac)
        reply = rep1 + rep2 + rep3
    except:
        reply = 'error with calc_CG function\tmake sure inputs are correct'
    
    return reply