#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
* @par Copyright (C): 2010-2020, hunan CLB Tech
* @file         Basic_movement
* @version      V2.0
* @details
* @par History

@author: zhulin
"""
from LOBOROBOT import LOBOROBOT  # 载入机器人库
import  RPi.GPIO as GPIO
import os



if __name__ == "__main__":
    clbrobot = LOBOROBOT() # 实例化机器人对象
    try:
        while True:
            #clbrobot.forward_Right(50,5)  #右前方
            #heng19zong32
            #clbrobot.moveRight(70,5)
            #119
            #clbrobot.moveRight(50,5)
            #68
        
            #77
            #clbrobot.moveLeft(70,5)      # 机器人前进
            #121
            clbrobot.t_stop(100)       # 机器人停止            
    except KeyboardInterrupt:
        clbrobot.t_stop(0) # 机器人停止
        GPIO.cleanup()

