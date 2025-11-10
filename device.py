# -*- coding: utf-8 -*-
"""
A Python Libarary for MSI AMR

Created on Mon Nov 19 13:58:55 2018

@author: leonli
"""


import requests
from requests.adapters import HTTPAdapter
import json
import sys
import re

import zlib
import numpy as np
from base64 import b64encode

import time
import math
import threading

import paramiko

import os
import socket
import struct
import array


DEBUG_READ_FILE = 0


MOVABLE_REQUEST_CONNECT_TIMEOUT = 5.0 #(unit: sec)
REQUEST_CONNECT_TIMEOUT = 2.0 #(unit: sec)
REQUEST_READ_TIMEOUT = 2.0 #(unit: sec)
HTTP_MAX_RETRY_TIMES = 1
EXCEPT_MAX_RETRY_TIMES = 1

PM_WALK = 6
PM_AUTO = 3

MAX_NAME_LENGTH = 1024
MAX_NAME_LENGTH -= 10

class Pinger(object):
    '''
    A class that can send PING (to another device)
    '''
    def __init__(self,timeout=3):
        """
        Constructor

        @param timeout: timeout of PING return
        """
        self.timeout = timeout
        self.__id = abs(os.getpid()%65535)
        self.__data = struct.pack('h',1) # h代表2个字节与头部8个字节组成偶数可进行最短校验

    @property
    def __icmpSocket(self): # 返回一个可以利用的icmp原对象,当做属性使用
        icmp = socket.getprotobyname("icmp") # 指定服务
        sock = socket.socket(socket.AF_INET,socket.SOCK_RAW,icmp) # socket.SOCK_RAW原生包
        return sock


    def __doCksum(self,packet): # 校验和运算
        words = array.array('h',packet) # 将包分割成2个字节为一组的网络序列
        sum = 0
        for word in words:
            sum += (word & 0xffff) # 每2个字节相加
        sum = (sum >> 16) + (sum & 0xffff)# 因为sum有可能溢出16位所以将最高位和低位sum相加重复二遍
        sum += (sum >> 16) # 为什么这里的sum不需要再 & 0xffff 因为这里的sum已经是16位的不会溢出,可以手动测试超过65535的十进制数字就溢出了
        return (~sum) & 0xffff # 最后取反返回完成校验

    @property
    def __icmpPacket(self):#icmp包的构造
        header = struct.pack('bbHHh',8,0,0,self.__id,0)
        packet = header + self.__data
        cksum = self.__doCksum(packet)
        header = struct.pack('bbHHh',8,0,cksum,self.__id,0) # 将校验带入原有包,这里才组成头部,数据部分只是用来做校验所以返回的时候需要返回头部和数据相加
        return header + self.__data 


    def sendPing(self,target_host):
        """
        Send PING to the target device

        @param target_host: IP address of target device (String)
        @return: True -> Target response
                 False -> Target no response
        """
        try:
            socket.gethostbyname(target_host)

            sock = self.__icmpSocket
            sock.settimeout(self.timeout)

            packet = self.__icmpPacket

            sock.sendto(packet,(target_host,1)) # send icmp packet

            ac_ip = sock.recvfrom(1024)[1][0]
            #print('[+] %s active'%(ac_ip))
            #print("ac_ip=", ac_ip)

            if ac_ip != target_host:
                return False


            return True;
        except Exception:
            sock.close()
            return False



###################################################################################################
class robot():
    '''
    A class that can control MSI AMR
    '''
    def __init__(self, ip="172.16.113.75", port=6660, myMac="e8:99:c4:c0:97:00"):
        """
        Constructor

        @param ip: IP Address of AMR (String)
        @param port: Communication port of AMR, default is 6600 (Int)
        @param myMac: User's MAC address, do nothing but still need this (String)
        """
        self.__ip=ip;
        self.__port=port;
        self.__isAlive=False;
        
        self.__isRemoteRepeat=False;
        self.__remoteRepeatStartTime=0;
        self.__remoteRepeatTime=0;
        self.__remoteV=0;
        self.__remoteW=0;
        self.__remoteThread=threading.Thread(target=self.__remote_repeat,  args=(0, 0, 1) );

        self.given_ai_name = ""
        self.is_enable_uuid = False

        self.__myMac=myMac;
        self.__url="http://"+self.__ip+":"+str(self.__port);
        self.__sshName='pyuser';
        self.__sshPwd='pyuser945';
        
        self.__pinger=Pinger(REQUEST_CONNECT_TIMEOUT);

        self.__session=requests.session();
        self.__session.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.__session_get_misc=requests.session();
        self.__session_get_misc.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session_get_misc.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.__session_get_life=requests.session();
        self.__session_get_life.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session_get_life.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.__session_get_pp=requests.session();
        self.__session_get_pp.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session_get_pp.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.__session_get_pr=requests.session();
        self.__session_get_pr.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session_get_pr.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.__session_map=requests.session();
        self.__session_map.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session_map.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.__session_map_info=requests.session();
        self.__session_map_info.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session_map_info.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.__session_file=requests.session();
        self.__session_file.mount('http://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))
        self.__session_file.mount('https://', HTTPAdapter(max_retries=HTTP_MAX_RETRY_TIMES))

        self.STANDBY=                          3  #Standby
        self.AI_PAUSE=                            12  #暫停
        self.AI_RESUME=                           13  #恢復暫停
        
        self.MOVE_FORWARD=                    14  #向前行走
        self.MOVE_FORWARD_SLOW=               15  #低速向前行走
        self.MOVE_BACKWARD=                   16  #向後行走
        self.TURN_LEFT=                       17  #原地向左轉
        self.TURN_RIGHT=                      18  #原地向右轉
        self.STOP=                            19
        self.STOP_NOW=                        20  #當機器有錯誤發生，而這個錯誤不屬於 ai_client 
        self.FIND_DOCK=                       25  #返回充電座
        self.PATH_PLAN=                       38  #沿規定路徑行走
        self.PATH_FOLLOW=                     39  #沿規定路徑行走
        self.REMOTE=                          40  #遙控，px=linear, py=turn
        self.TRACK_FOLLOW=                    41  #沿規定路徑行走
        
        self.MOVE2TARGET=                     100  #移動到目標點
        
        self.SLAM_MAP_REFRESH=                500
        self.SLAM_MAP_RESET=                  501  #Reset 目前的地圖
        self.SLAM_MAP_RELOCATE=               502  #重新定位 SLAM 地圖位置
        self.SLAM_MAP_AI_AUTO=                504  #由AI判斷是否鎖地圖
        self.SLAM_MAP_LOCK=                   505  #未使用，lock slam map
        self.SLAM_MAP_UN_LOCK=                506  #未使用，un-lock slam map
        self.SLAM_MAP_LOAD=                   507  #Load SLAM map
        self.SLAM_MAP_SAVE=                   508  #Save SLAM map
        self.SLAM_SET_ORIGIN=                 511  #Set Origin
        self.SLAM_ERROR_RESET=                523  #Reset SLAM map
        self.SLAM_CLOSING_START=              520  #Closing start
        self.SLAM_CLOSING_END=                521  #Closing end
        self.SLAM_CLOSING_CANCEL=             522  #Closing cancel
        self.SLAM_BUILD_ON=                   526  #Build On
        self.SLAM_BUILD_OFF=                  527  #Build Off


        #info define                
        self.AI_SUCCESS                    =     900  #各種AI行為，成功
        self.AI_FAILURE                    =     901  #各種AI行為，失敗
        self.AI_BUSY                       =     902  #各種AI行為，計算忙碌中
        self.AI_WAITING                    =     903  #各種AI行為，等待中
        self.AI_STALL                      =     904  #機器無法前進 (地圖錯誤、緊急停止按紐...等)
        
        self.AI_PP_TARGET_ERROR            =     930  #PP 目標點錯誤 (空間不夠、在障礙物上...等)

        self.AI_PP_NOPATH                  =     933  #PP 無法規劃路徑
        self.AI_PF_NOPATH                  =     935  #PF 無法抵達目標

        self.AI_PF_DERAIL                  =     941  #PF 脫軌，偏離軌道
        self.AI_DOCK_ABORT_WF              =     945  #Docking 無法前往充電站
        self.AI_DOCK_ABORT_DOCKING         =     946  #Docking 充電站停靠失敗

        self.AI_FIND_DOCK_ABORT_WF         =     907  #無法前往充電站
        self.AI_FIND_DOCK_ABORT_DOCKING    =     908  #充電站停靠失敗

        self.AI_RESET_ERROR                =     300  #清除ai error並reset
        
        self.DO_UV_EN                      =     1

        self.try_print_exception_str = '''try:\n    print('url=', url)\nexcept:\n    pass\ntry:\n    print('payload=', payload)\nexcept:\n    pass\ntry:\n    print('r.text=', r.text)\nexcept:\n    pass\ntry:\n    print('data=', data)\nexcept:\n    pass\n'''
        self.try_print_exception_itself_str='''try:\n    print(e)\nexcept:\n    pass'''
        self.check_no_ok='''try:\n    if len(r.content)==5:\n        if r.text=="NO_OK":\n            print("NO_OK,",sys._getframe().f_code.co_name,",",time.time())\nexcept:\n    pass'''

    def __enter__(self):
        print("__enter__")
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        print("__exit__")
        self.__isAlive=False;
        self.__isRemoteRepeat=False;
        
    def __del__(self):
        print("__del__")
        self.__isAlive=False;
        self.__isRemoteRepeat=False;

    def SetEnableUUID(self, sw):
        self.is_enable_uuid = sw

    def AI_name_UUID_postfix(self, name):
        new_name = name
        if self.is_enable_uuid == True:
            new_name += f"   <{int(time.time())}>"

        return new_name

    def pos_to_grid(self, x, y, mapScale=0.02, mapOrgX=-81.92, mapOrgY=-81.92):
        """
        Transform world coordinate (used by moving command) to map coordinate (a 8192*8192 2d array)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y, unit: m (Float)
        @param mapScale: Resolution of map coordinate, default is 0.02 m/grid (Float)
        @param mapOrgX: Origin X of map coordinate,
                        X distance from world coordinate origin, unit: m,
                        default is -81.92 (Float)
        @param mapOrgY: Origin Y of map coordinate,
                        Y distance from world coordinate origin, unit: m,
                        default is -81.92 (Float)
        @return:
                 Map coordinate X -> Column of the 2d array (Int)
                 Map coordinate Y -> Row of the 2d array (Int)
        """
        return int((x-mapOrgX)/mapScale + 0.5), int((y-mapOrgY)/mapScale + 0.5);

    def grid_to_pos(self, gridX, gridY, mapScale=0.02, mapOrgX=-81.92, mapOrgY=-81.92):
        """
        Transform map coordinate (a 8192*8192 2d array) to world coordinate (usually used by moving command)

        @param gridX: Map coordinate X, unit: grid or pixel (Int)
        @param gridY: Map coordinate Y, unit: grid or pixel (Int)
        @param mapScale: Resolution of map coordinate, default is 0.02 m/grid (Float)
        @param mapOrgX: Origin X of map coordinate,
                        X distance from world coordinate origin, unit: m,
                        default is -81.92 (Float)
        @param mapOrgY: Origin Y of map coordinate,
                        Y distance from world coordinate origin, unit: m,
                        default is -81.92 (Float)
        @return:
                 World coordinate X (Floor)
                 World coordinate Y (Floor)
        """
        return (gridX*mapScale+mapOrgX), (gridY*mapScale+mapOrgY);

    def get_ip(self):
        """
        Get AMR's IP of this class

        @return:
                 IP Address (String)

        """
        return self.__ip

    def get_port(self):
        """
        Get AMR's communication port of this class

        @return:
                 Port (Int)

        """
        return self.__port



    def get_map_information(self, to=REQUEST_CONNECT_TIMEOUT):
        #mapInfo=data['map_info']
        #map_width=mapInfo['width']    #map width unit:<grid>
        #map_height=mapInfo['height']  #map height unit:<grid>
        #map_scale=mapInfo['scale']    #map scale unit:<m/grid>
        #map_orgx=mapInfo['orgx']      #map orgx unit:<m>
        #map_orgy=mapInfo['orgy']      #map orgy unit:<m>

        """
        Get information in map coordinate of current map

        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                None      -> If timeout or disconnect (None)
                "NO_OK"   -> Fail request (String)
                JSON data ->
                             {
                               'width': 8192,
                               'height': 8192,
                               'orgx': -81.92,
                               'orgy': -81.92,
                               'scale': 0.019999999552965164
                             }

                             width -> width (column count) of map coordinate (Int)
                             height -> height (row count) of map coordinate (Int)
                             orgx -> Origin X of map coordinate,
                                     X distance from world coordinate origin, unit: m (Float)
                             orgy -> Origin Y of map coordinate,
                                     Y distance from world coordinate origin, unit: m (Float)
                             scale -> Resolution of map coordinate (Float)

        """
        for trycount in range(1):
            try:
                url = self.__url+"/get_map_info"
                payload = {"mac": self.__myMac}
                r = self.__session_map_info.post(url, data=json.dumps(payload), timeout=to)
                #r = requests.post(url, data=json.dumps(payload), timeout=to)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                data=r.json()
                return data['map_info']
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def set_button(self, btCount=-999999):
        """
        Set times of button being pressed and released
        If times of a button is greater than previous, it means the button is being pressed and released

        @param btCount: An array of Int, array[<button_idx>] = <botton pressed times>
                          [ 0,        1,        20 ]
                            ^         ^         ^
                            |         |         |
                        button 0  button 1  button 2

        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)

        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url + "/set_misc"

                # print(do, doVal)
                if (type(btCount) != list):
                    pass

                else:
                    payload = {"set_misc": {
                        "btn": btCount
                    },
                        "mac": self.__myMac}

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code != 200:
                    print("====== {", sys._getframe().f_code.co_name, "} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def set_misc(self, doSel=[0,0,0,0,0,0,0,0], doValue=[0,0,0,0,0,0,0,0], btCount=-999999):
        """
        Set port of external power output and times of button being pressed

        @param doSel: An array of Int.
                      Mask to enable/disable external power output change
                         [0, 1, 0, 0, 0, 0, 0, 0]
                          ^                    ^
                          |                    |
                       Port 0                Port 7

                      0 -> Disable change
                      1 -> Enable change

        @param doValue:
                        An array of Int.
                        High/Low voltage of external power output
                            [0, 1, 0, 0, 0, 0, 0, 0]
                             ^                    ^
                             |                    |
                          Port 0                Port 7

                        0 -> Low Voltage (0V)
                        1 -> High Voltage (12V or 24V depend on port)

        @param btCount: An array of Int, array[<button_idx>] = <botton pressed times>
                          [ 0,        1,        20 ]
                            ^         ^         ^
                            |         |         |
                         button 0  button 1  button 2

        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/set_misc"
                

                if (type(btCount)!=list):
                    payload = { "set_misc" : {
                                    "ext_io_sel" : doSel,
                                    "ext_io"  : doValue
                                },
                             "mac": self.__myMac }                
                else:
                    payload = { "set_misc" : {
                                    "ext_io_sel" : doSel,
                                    "ext_io"  : doValue,
                                    "btn"  : btCount
                                },
                             "mac": self.__myMac}
                        
                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
                
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None
        
    #position=get_misc()["position"]
    def set_internal_misc(self, doSel=[0, 0, 0, 0, 0, 0, 0, 0], doValue=[0, 0, 0, 0, 0, 0, 0, 0]):
        """
        Set port of internal power (not GPIO) output

        @param doSel: An array of Int.
                      Mask to enable/disable internal power (not GPIO) output change
                         [0, 1, 0, 0, 0, 0, 0, 0]
                          ^                    ^
                          |                    |
                       Port 0                Port 7

                      0 -> Disable change
                      1 -> Enable change

        @param doValue:
                        An array of Int.
                        High/Low voltage of internal power (not GPIO) output
                            [0, 1, 0, 0, 0, 0, 0, 0]
                             ^                    ^
                             |                    |
                          Port 0                Port 7

                        0 -> Low Voltage
                        1 -> High Voltage

        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url + "/set_misc"

                payload = {"set_misc": {
                    "internal_io_sel": doSel,
                    "internal_io": doValue
                },
                    "mac": self.__myMac}


                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code != 200:
                    print("====== {", sys._getframe().f_code.co_name, "} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def get_misc(self, try_count_max=EXCEPT_MAX_RETRY_TIMES, to=REQUEST_CONNECT_TIMEOUT):
        """
        Get a bunch of basic information from AMR

        @param try_count_max: Max times to retry request (Int)
        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                 None      -> If timeout or disconnect (None)
                 "NO_OK"   -> Fail request (String)
                 JSON data ->
                              {
                                 'battery': {'power': 100.0, 'volts': 29.18, 'runtime': 0.0, 'status': 0, 'charging': 1},
                                 'battery1': {'power': 50.0, 'volts': 24.0, 'runtime': 14400.0, 'status': 0, 'charging': 1},
                                 'status': {
                                              'name': 'SLAM Build On',
                                              'data': {'@cxlCamera#': False,
                                                       '@cxlLaserNorm#': False,
                                                       '@cxlMap#': False,
                                                       '@dg_FB:': 0.075,
                                                       '@dg_LR:': 0.035,
                                                       '@pf_FB:': 1.5,
                                                       '@pf_LR:': 0.2,
                                                       '@rs_CF:': 0.2,
                                                       '@rs_CL:': 0.28,
                                                       '@rs_RH:': 1.399,
                                                       '@rs_RL:': 0.579,
                                                       '@rs_RW:': 0.56,
                                                       '@rs_SH:': 0.15,
                                                       '@status_DockID#': '[--- --- ---]',
                                                       '@status_DockLast#': '[12.23 3.47 -1.6142]',
                                                       '@status_PP#': 601,
                                                       '@status_RTi#': 0,
                                                       '@status_RTn#': 3,
                                                       '@status_Reverse#': False,
                                                       '@status_Slope#': False,
                                                       '@status_TOi#': 0,
                                                       '@status_TOn#': 24},
                                               'status': 19,
                                               'info': 900,
                                               'power': 9,
                                               'error': 0,
                                               'warning': 0
                                           },

                                 'position': {  'x': 12.213,
                                                'y': 3.095,
                                                'a': -1.614,
                                                'lost': -0.159,
                                                'status': 100
                                              },

                                 'velocity': {'x': 0.0, 'y': 0.0, 'a': 0.0},
                                 'btn': [0, 0, 0],
                                 'ext_io': [0, 0, 0, 0, 0, 0, 0, 0],
                                 'utc': 1661493377.171
                               }
        """
        r=None
        for trycount in range(try_count_max):
            try:
                #取得空間位置 電池電力以及 AI 狀態
                url = self.__url+"/get_misc"
                payload = {"mac": self.__myMac}
                r = self.__session_get_misc.post(url, data=json.dumps(payload), timeout=to )
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                data=r.json()
                return data
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None
            
    def get_plan_path(self, try_count_max=EXCEPT_MAX_RETRY_TIMES, to=REQUEST_CONNECT_TIMEOUT):
        """
        Get AMR's current planed (or moving) path

        @param try_count_max: Max times to retry request (Int)
        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                 None       -> If timeout or disconnect (None)
                 "NO_OK"    -> Fail request (String)
                 NUMPY data ->
                               A Numpy 2D array contained a bunch of continuous world coordinate

                               array([ [12.2128525,  3.0947647],
                                       [12.210091 ,  3.082634 ],
                                       [12.199852 ,  3.0654526],
                                       ...,
                                       [23.580353 , -4.6148834],
                                       [23.599571 , -4.6153336],
                                       [23.615723 , -4.615715 ]  ], dtype=float32)

                               array([ [x0,  y0],
                                       [x1 , y1],
                                       [x2 , y2],
                                       ...,
                                       [x3000 , y3000],
                                       [x3001 , y3001],
                                       [x3002 , y3002]  ], dtype=float32)

                               x0 ~ x3002 ->  World coordinate X
                               y0 ~ y3002 ->  World coordinate Y

        """

        for trycount in range(try_count_max):
            try:
                #取得規劃的軌跡
                url = self.__url+"/get_plan_path"
                payload = { "mac": self.__myMac }
                r = self.__session_get_pp.post(url, data=json.dumps(payload),timeout=to)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                if ( len(r.content)>0 ):
                    planPath = zlib.decompress(r.content)
                    planPath_ = np.fromstring(planPath, dtype = np.float32 )
                    planPathResharp = planPath_.reshape( int(len(planPath_)/2), 2)
                    return planPathResharp
                    """
                    plt.plot(planPathResharp[:,0], planPathResharp[:,1] )
                    plt.xlabel("X unit:m")
                    plt.ylabel("Y unit:m")
                    """
                else:
                    #print("no plan path")
                    return np.zeros((0,2))
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None



    def get_plan_ring(self, try_count_max=EXCEPT_MAX_RETRY_TIMES, to=REQUEST_CONNECT_TIMEOUT):
        """
        Get intergrated sensor result of AMR's obstacle avoiding

        @param try_count_max: Max times to retry request (Int)
        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                 None       -> If timeout or disconnect (None)
                 "NO_OK"    -> Fail request (String)
                 NUMPY data ->
                               A Numpy 2D array contained a bunch of world coordinate
                               Minimum row of array is 28
                               Maximum row of array is 400

                               array([[ 12.483914  ,   2.8828118 ],
                                      [ 11.924441  ,   2.907096  ],
                                      [ 11.949593  ,   3.4865503 ],
                                      [ 12.509066  ,   3.4622662 ],
                                      ...
                                      [ 12.212851  ,   3.0947654 ]], dtype=float32)

                               array([ [x0,  y0],
                                       [x1 , y1],
                                       [x2 , y2],
                                       [x3 , y4],
                                       ...,
                                       [x399 , y399] ], dtype=float32)

                               (x0, y0) ~ (x3, y3)       -> World coordinate of AMR size
                                                            (4 elements represent 4 vertices of rectangle)

                               (x4, y4) ~ (x7, y7)       -> World coordinate of AMR dangrous area
                                                            (4 elements represent 4 vertices of rectangle)

                               (x8, y8) ~ (x11, y11)     -> World coordinate of AMR warning area
                                                            (4 elements represent 4 vertices of rectangle)

                               (x12, y12) ~ (x15, y15)   -> World coordinate of AMR safe area
                                                            (4 elements represent 4 vertices of rectangle)

                               (x16, y16) ~ (x18, y18)   -> Front seneor intergration
                                                            (3 elements for 3 positions of seneor intergration)

                               (x19, y19) ~ (x21, y21)   -> Right seneor intergration
                                                            (3 elements for 3 positions of seneor intergration)

                               (x22, y22) ~ (x24, y24)   -> Backward seneor intergration
                                                            (3 elements for 3 positions of seneor intergration)

                               (x25, y25) ~ (x27, y27)   -> Left seneor intergration
                                                            (3 elements for 3 positions of seneor intergration)

                               (x28, y28) ~ (x387, y387) -> Specific seneor intergration
                                                            28~387 appear only when it is triggered by function - "set_sensor_display_config(name, mode, s_type, t)"
                                                            (360 elements for 360 positions of seneor intergration)

        """
        for trycount in range(try_count_max):
            try:
                #取得防撞圈的資訊
                url = self.__url+"/get_plan_ring"
                payload = { "mac": self.__myMac }
                r = self.__session_get_pr.post(url, data=json.dumps(payload),timeout=to)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                if ( len(r.content)>0 ):
                    planRing = zlib.decompress(r.content)
                    planRing_ = np.fromstring(planRing, dtype = np.float32 )
                    #print("len(planRing_) =", len(planRing_))
                    if len(planRing_)%3==0:
                        planRingResharp = planRing_.reshape( int(len(planRing_)/3), 3)
                    else:
                        planRingResharp = planRing_.reshape( int(len(planRing_)/2), 2)
                    return planRingResharp
                    """
                    plt.plot(planRingResharp[:,0], planRingResharp[:,1] )
                    plt.xlabel("X unit:m")
                    plt.ylabel("Y unit:m")
                    """
                else:
                    #print("no plan path")
                    return np.zeros((0,2))
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def reset_ai_error(self, name="Reset AI Error", t=-1):
        """
        Reset AI error (AI has error when ai_error shown not 0 in get_misc() json data)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.AI_RESET_ERROR},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    # ask ai_client to do slam map refresh
    def slam_map_refresh(self, name="Fresh SLAM Map", t=-1):
        """
        Refresh SLAM map (remember to do this after loading another map)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_MAP_REFRESH},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_map_reset(self, name="Reset SLAM Map", t=-1):
        """
        Reset SLAM map (AMR will get a whole empty map, and use its reality location as origin(0,0) of the map)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #重置地圖
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_MAP_RESET},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_map_save(self, name="Save SLAM Map", t=-1, vwmapindex=None, ppmapindex=None):
        """
        Save SLAM map (You can always do this after scanning the working area)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                map_arr = []
                if type(vwmapindex) is int:
                    map_arr.append(vwmapindex)

                if type(ppmapindex) is int:
                    map_arr.append(ppmapindex)


                if len(map_arr) > 0:
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : self.SLAM_MAP_SAVE,
                                    "map"   : map_arr
                                            },
                             "mac": self.__myMac}
                else:
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : self.SLAM_MAP_SAVE
                                            },
                             "mac": self.__myMac}


                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_map_load(self, name="Load SLAM Map", t=-1):
        """
        Load SLAM map (use this function after running function - "set_map_group(group_name)")

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #載入地圖
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_MAP_LOAD},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def slam_map_relocate(self, x, y, a, name="Relocate By SLAM Map", data="", t=-1):
        """
        Relocate AMR (always do it after loading the map)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y, unit: m (Float)
        @param a: World coordinate Angle, unit: radian (Float)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #重新定位 根據輸入的座標點進行重新定位
                #大約需要花費 10~30s
                url = self.__url+"/set_job"

                if data!="":
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : self.SLAM_MAP_RELOCATE,
                                    "target"  : [
                                      {"x": x, "y": y, "a": a } #人工指定目前 robot 空間的位置 unit: m, m, rad
                                    ],
                                    "data" : data
                                },
                             "mac": self.__myMac}

                else:
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : self.SLAM_MAP_RELOCATE,
                                    "target"  : [
                                      {"x": x, "y": y, "a": a } #人工指定目前 robot 空間的位置 unit: m, m, rad
                                    ]
                                },
                             "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_setlocation(self, x, y, a, name="Set Location In SLAM Map", t=-1):
        """
        "Forcing" AMR to relocate by assigned world coordinate position and angle (even the posiion and angle is ridiculous)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y, unit: m (Float)
        @param a: World coordinate Angle, unit: radian (Float)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                            "name"  : name[0:MAX_NAME_LENGTH],
                            "cmd"   : self.SLAM_MAP_RELOCATE,
                            "target"  : [
                                    {"x": x, "y": y, "a": a } #人工指定目前 robot 空間的位置 unit: m, m, rad
                                    ] ,
                            "data" : "SetLocation"
                            },
                     "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_set_origin(self, x, y, a, name="Set SLAM Map New Origin", t=-1):
        """
        Set a new world coordinate position and angle as new origin(0,0).
        (It means map moving and rotating can be done through this function)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y, unit: m (Float)
        @param a: World coordinate Angle, unit: radian (Float)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                            "name"  : name[0:MAX_NAME_LENGTH],
                            "cmd"   : self.SLAM_SET_ORIGIN,
                            "target"  : [
                                    {"x": x, "y": y, "a": a } #人工指定目前 robot 空間的位置 unit: m, m, rad
                                    ] ,
                            "data" : ""
                            },
                     "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_build_on(self, name="SLAM Build On", t=-1):
        """
        Increase the map scanning range to the MAXIMUM (recommand only use it when doing map scan)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_BUILD_ON},
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_build_off(self, name="SLAM Build Off", t=-1):
        """
        Decrease the map scanning range to the MINIMUM (recommand use it when AMR is doing moving job)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_BUILD_OFF},
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_map_lock(self, name="Lock SLAM Map", t=-1):
        """
        Lock SLAM map (new obstacle will not write to map)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #鎖定地圖不在允許更新
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_MAP_LOCK},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_map_un_lock(self, name="Unlock SLAM Map", t=-1):
        """
        Unlock SLAM map (new obstacle always write to map, must do it before loading map)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #解鎖地圖
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_MAP_UN_LOCK},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_map_auto_lock(self, name="Autolock SLAM Map", t=-1):
        """
        AMR determine to unlock/lock SLAM map by itself

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #解鎖地圖
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_MAP_AI_AUTO},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_auto_closing_start(self, name="SLAM Auto Closing Start", t=-1):
        """
        Turn on "Auto Closing Loop" mode.
        (Recommand to do it before scanning map, turning on this mode always get better result,
         especially at a very large working area (almost 160m x 160m) )

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 515},
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_auto_closing_end(self, name="SLAM Auto Closing End", t=-1):
        """
        Turn off "Auto Closing Loop" mode.
        ("Auto Closing Loop" mode will consume a lot of CPU resource, close it after ending map scanning)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 516},
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_closing_start(self, name="SLAM Closing Start", t=-1):
        """
        ABANDONED. DO NOT USE IT
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_CLOSING_START},
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_closing_end(self, name="SLAM Closing End", t=-1):
        """
        ABANDONED. DO NOT USE IT
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_CLOSING_END},
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_closing_cancel(self, name="SLAM Closing Cancel", t=-1):
        """
        ABANDONED. DO NOT USE IT
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_CLOSING_CANCEL},
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_reset_error(self, name="Reset SLAM Map", t=-1):
        """
        Reset SLAM error (SLAM error is also included in AI error, so just do this with reset_ai_error())

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #解鎖地圖
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.SLAM_ERROR_RESET},
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def leave_dock(self, vwmapindex=0, zonemapindex=0, name="Leave Dock", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to leave the last docking

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param number: Number of sign's id (Int)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        #head_data = "@dc_ID:C{nstr}#;".format(nstr=number_str)
        head_data = ""
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 24,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None


    def move2dock_and_docking(self, x, y, a, vwmapindex=0, zonemapindex=0, name="Move2Dock And Docking", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to go back to charge station by given position and charge station toward angle
        (AMR will search and check whether valid charge station is found)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y, unit: m (Float)
        @param a: World coordinate Angle, unit: radian (Float)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                #移動到指定目標點
                #10 是參考的 VW indes range 10~29 
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                #"power": 3,
                                "cmd"   : self.FIND_DOCK,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  #{"x": x+math.cos(a)*0.8, "y": y+math.sin(a)*0.8, "a": a } #要移動到的空間位置 unit: m, m, rad
                                  {"x": x, "y": y, "a": a }
                                ],
                                "data": data+"@cxlLaserSide#;@cxlSonar#@pp_RT:30#;"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                #threading.Thread(target=self.__move2dock_and_docking,args=(name,)).start()
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def move2dock_and_docking_blind(self, x, y, a, vwmapindex=0, zonemapindex=0, name="Move2Dock And Docking Blind", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to go back to charge station by given position and charge station toward angle
        (AMR will "NOT" search and check whether valid charge station is found, AMR believes the given position and angle is correct)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y, unit: m (Float)
        @param a: World coordinate Angle, unit: radian (Float)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                #移動到指定目標點
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  #{"x": x+math.cos(a)*0.8, "y": y+math.sin(a)*0.8, "a": a } #要移動到的空間位置 unit: m, m, rad
                                  {"x": x, "y": y, "a": a }
                                ],
                                "data": data+"@cxlLaserSide#;@cxlSonar#@cxlCamera#@pp_RT:30#@dc_SimSig#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                #threading.Thread(target=self.__move2dock_and_docking,args=(name,)).start()
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None



    # 只有機器在看的到的狀態下下此命令才會進行返回 dock 行為
    def find_dock(self, name="Find Dock", data="", t=-1):
        """

        Command AMR to go back to charge station right now
        (AMR will search nearby and check whether valid charge station is found)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令 
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "data"  : data + "@cxlLaserSide#;@cxlSonar#@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def pattern_docking_dock(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Dock", number=None, data="", t=-1):
        """

        Command AMR to docking by 'Front' sign (a group of qrcode)
        (AMR will search nearby and check the 'Front' sign)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param number: Number of sign's id (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        if number is not None:
            if type(number) is int:
                number_str = '%03d'%number

        head_data = "@dc_ID:D{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@dc_RD:0#;@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def pattern_docking_front(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Front", number=None, data="", t=-1):
        """

        Command AMR to docking by 'Front' sign (a group of qrcode)
        (AMR will search nearby and check the 'Front' sign)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param number: Number of sign's id (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        if number is not None:
            if type(number) is int:
                number_str = '%03d'%number

        head_data = "@dc_ID:F{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)


                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@dc_RD:0#;@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None


    def pattern_docking_back(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Back", number=None, data="", t=-1):
        """

        Command AMR to docking by 'Back' sign (a group of qrcode)
        (AMR will search nearby and check the 'Back' sign)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param number: Number of sign's id (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        if number is not None:
            if type(number) is int:
                number_str = '%03d'%number

        head_data = "@dc_ID:B{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : "@dc_RD:0#;@pp_RT:30#" + data
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def pattern_docking_left(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Left", number=None, data="", t=-1):
        """

        Command AMR to docking by 'Left' sign (a group of qrcode)
        (AMR will search nearby and check the 'Left' sign)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param number: Number of sign's id (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        if number is not None:
            if type(number) is int:
                number_str = '%03d'%number

        head_data = "@dc_ID:L{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)


                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@dc_RD:0#;@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def pattern_docking_right(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Right", number=None, data="", t=-1):
        """

        Command AMR to docking by 'Right' sign (a group of qrcode)
        (AMR will search nearby and check the 'Right' sign)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param number: Number of sign's id (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        if number is not None:
            if type(number) is int:
                number_str = '%03d'%number

        head_data = "@dc_ID:R{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)


                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@dc_RD:0#;@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def pattern_docking_side(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Side", number=None, data="", t=-1):
        """

        Command AMR to docking by 'Side' sign (a group of qrcode)
        (AMR will search nearby and check the 'Side' sign)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param number: Number of sign's id (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        if number is not None:
            if type(number) is int:
                number_str = '%03d'%number

        head_data = "@dc_ID:S{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)


                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@dc_RD:0#;@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def pattern_docking_center(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Center", number=None, data="", t=-1):
        """

        Command AMR to docking to the center through 'Front', 'Back', 'Left' and 'Right' sign (a group of qrcode)
        (AMR will search nearby and check the sign, then AMR will docking to the center of all the sign)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param number: Number of sign's id (Int)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        if number is not None:
            if type(number) is int:
                number_str = '%03d'%number

        head_data = "@dc_ID:C{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.FIND_DOCK,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@dc_RD:0#;@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def pattern_docking_center_leave(self, vwmapindex=0, zonemapindex=0, name="Pattern Docking Center Leave", data="", t=-1):
        """

        Command AMR to leave the center of the last docking

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param number: Number of sign's id (Int)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        head_data = ""
        number_str = "---"
        head_data = "@dc_ID:C{nstr}#;".format(nstr=number_str)
        data = head_data + data

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)


                #進行返家 機器必須與充電坐在同一個圖層下才能下達返家命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 24,
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "data"  : data + "@pp_RT:30#"
                                },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    #move2target(10.,0.,0.,10,30)
    def move2target(self, x, y, a, vwmapindex=0, zonemapindex=0, name="Move2Target", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to go to the given position and toward the given angle in shortest path (like a car driver hurry to somewhere)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y, unit: m (Float)
        @param a: World coordinate Angle, unit: radian (Float)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                #移動到指定目標點
                #10 是參考的 VW indes range 10~29 
                #30 是參考的 速限地圖 indes range 30~49
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                print("map_arr=", map_arr)

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.MOVE2TARGET,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  {"x": x, "y": y, "a": a } #要移動到的空間位置 unit: m, m, rad
                                ],
                                "data": data+"@pp_RT:10#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def path_plan_AB(self, Ax, Ay, Aa, Bx, By, Ba, vwmapindex=0, zonemapindex=0, name="Path Plan AB", side="none", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to calculate a path from START (position A) to END (position B)
        (Given Virtual Wall Map would affect the result, virtual walls are regarded as obstacles)

        @param Ax: START, world coordinate X, unit: m (Float)
        @param Ay: START, world coordinate Y, unit: m (Float)
        @param Aa: START, world coordinate Angle, unit: radian (Float)
        @param Bx: END, world coordinate X, unit: m (Float)
        @param By: END, world coordinate Y, unit: m (Float)
        @param Ba: END, world coordinate Angle, unit: radian (Float)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                #移動到指定目標點
                #10 是參考的 VW indes range 10~29 
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                #side = "left"   規劃靠左
                #     = "right"  規劃靠右
                #     = "center" 規劃靠中

                data_str = "@pp_Sim#"
                if side=="left":
                    data_str += " @pp_Left#"
                elif side == "right":
                    data_str += " @pp_Right#"
                elif side == "center":
                    data_str += " @pp_Center#"
                elif side=="none":
                    data_str += ""
                else:
                    data_str += ""

                data_str +="@pp_RT:0#@pp_WT:0#"
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.PATH_PLAN,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  {"x": Ax, "y": Ay, "a": Aa }, #要移動到的空間位置 unit: m, m, rad
                                  {"x": Bx, "y": By, "a": Ba }
                                ] ,
                                "data" : data+data_str
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def path_plan_B(self, Bx, By, Ba, vwmapindex=0, zonemapindex=0, name="Path Plan B", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to calculate a path from START (current position) to END (position B)
        (Given Virtual Wall Map would affect the result, virtual walls are regarded as obstacles)

        @param Bx: END, world coordinate X, unit: m (Float)
        @param By: END, world coordinate Y, unit: m (Float)
        @param Ba: END, world coordinate Angle, unit: radian (Float)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                #移動到指定目標點
                #10 是參考的 VW indes range 10~29 
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.PATH_PLAN,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  {"x": Bx, "y": By, "a": Ba } #要移動到的空間位置 unit: m, m, rad
                                ] ,
                                "data" : data+"@pp_RT:0#@pp_WT:0#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None
            
    #Device.path_follow(np.random.rand(5,3)*10.0-5.0,10,30)
    def path_follow(self, path, vwmapindex=0, zonemapindex=0, name="Path Follow", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to follow a given path (or track), AMR will avoid the obstacle and go back to follow the path (or track)

        @param path: list of positions, [[x0,y0],[x1,y1],...[xn,yn]], the max count of list is 20000 (List)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                #沿著指定路徑行走
                if len(path[0])==2:
                    pathlist=[{"x": xy[0], "y": xy[1], "a": 10.0 } for xy in path]
                else:
                    pathlist=[{"x": xya[0], "y": xya[1], "a": xya[2] } for xya in path]

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.PATH_FOLLOW,  #沿規定路徑行走
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                #"target"  : [
                                #  {"x": 0.0, "y": 0.1, "a": 0.0 }, #要走的路徑 unit: m, m, rad
                                #  {"x": 1.0, "y": 1.1, "a": 0.1 },
                                #  {"x": 2.0, "y": 2.1, "a": 0.2 },
                                #  {"x": 3.0, "y": 3.1, "a": 0.3 },
                                #  {"x": 4.0, "y": 4.1, "a": 0.4 }
                                #]
                                "target"  : pathlist,
                                "data": data+"@pf_RT:3#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def rail_follow(self, path, vwmapindex=0, zonemapindex=0, name="Rail Follow", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to follow a given path (or track), AMR will "NOT" avoid the obstacle, AMR will stop and wait the obstacle gone

        @param path: list of positions, [[x0,y0],[x1,y1],...[xn,yn]], the max count of list is 20000 (List)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                #沿著指定路徑行走
                if len(path[0])==2:
                    pathlist=[{"x": xy[0], "y": xy[1], "a": 10.0 } for xy in path]
                else:
                    pathlist=[{"x": xya[0], "y": xya[1], "a": xya[2] } for xya in path]

                print(f"rail_follow - pathlist = {pathlist}")

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.TRACK_FOLLOW,  #沿規定路徑行走
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                #"target"  : [
                                #  {"x": 0.0, "y": 0.1, "a": 0.0 }, #要走的路徑 unit: m, m, rad
                                #  {"x": 1.0, "y": 1.1, "a": 0.1 },
                                #  {"x": 2.0, "y": 2.1, "a": 0.2 },
                                #  {"x": 3.0, "y": 3.1, "a": 0.3 },
                                #  {"x": 4.0, "y": 4.1, "a": 0.4 }
                                #]
                                "target"  : pathlist,
                                "data": data+"@pp_RT:10#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def rail_follow_backward(self, path, vwmapindex=0, zonemapindex=0, name="Rail Follow Backward", data="", ppmapindex=None, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to follow a given path (or track), AMR will "NOT" avoid the obstacle, AMR will stop and wait the obstacle gone
        (AMR's back is toward front, the moving direction)

        @param path: list of positions, [[x0,y0],[x1,y1],...[xn,yn]], the max count of list is 20000 (List)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param data: Extra command for out-of-spec enviroment and appearance modification of AMR (String)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                #沿著指定路徑行走
                if len(path[0])==2:
                    pathlist=[{"x": xy[0], "y": xy[1], "a": 10.0 } for xy in path]
                else:
                    pathlist=[{"x": xya[0], "y": xya[1], "a": xya[2] } for xya in path]

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.TRACK_FOLLOW,  #沿規定路徑行走
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                #"target"  : [
                                #  {"x": 0.0, "y": 0.1, "a": 0.0 }, #要走的路徑 unit: m, m, rad
                                #  {"x": 1.0, "y": 1.1, "a": 0.1 },
                                #  {"x": 2.0, "y": 2.1, "a": 0.2 },
                                #  {"x": 3.0, "y": 3.1, "a": 0.3 },
                                #  {"x": 4.0, "y": 4.1, "a": 0.4 }
                                #]
                                "target"  : pathlist,
                                "data": data+"@behReverse#@pp_RT:10#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None


    def get_mission_script_list(self):
        """

        Get all mission script

        @return:
            None          -> If timeout or disconnect (None)

            ["missionA.py",
             "missionB.py",
             "missionC.py",...] -> List of map folders' name (List)
        """
        fileNameList=[];
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            #print("device - get_map_group_list")
            try:
                ssh = paramiko.SSHClient();
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
                ssh.connect(hostname=self.__ip , port=22, username=self.__sshName, password=self.__sshPwd);
                stdin, stdout, stderr = ssh.exec_command('ls /home/pyuser/FMS/mscript/*'); #find all folder only
                result = stdout.read();
                ssh.close();

                file=result.decode();
                files=file.split('\n');
                for i in files:
                    if ( len(i)!=0 ):
                        r=i.split('/');
                        fname = r[5].replace(".py", "")
                        fileNameList.append(fname);
                return fileNameList;
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None


    def set_mission_script(self, ms_name, argv=None, mtype=None, start_from=None, ai_name_prefix=None):
        """

        Run the mission script uploaded to robot

        @param ms_name: Mission script name (Str)
        @return:
                 None      -> If timeout or disconnect (None)
                 JSON data ->
                             {
                               'result': 'sucess'
                             }

                             'result' -> Whether "set_mission_script" command is transmitted success or fail (String)
        """
        for trycount in range(1):
            # try:

            url = self.__url + "/set_mission_script"

            payload = {
                "mscript": ms_name
            }

            if argv is not None:
                print("argv =", argv)
                print("type(argv) =", str(type(argv)) )
                if type(argv) is list:
                    payload['argv'] = argv

            if mtype is not None:
                if type(mtype) is str:
                    payload['mtype'] = mtype

            if start_from is not None:
                if type(type) is int:
                    payload['start_from'] = start_from

            if ai_name_prefix is not None:
                if type(ai_name_prefix) is str:
                    payload['ai_name_prefix'] = ai_name_prefix



            # t1 = time.time();
            # print("Device r pre:",t1)
            r = self.__session_file.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
            if r.status_code != 200:
                print("====== {", sys._getframe().f_code.co_name, "} r.status_code!=200 ======")
                continue

            # exec(self.check_no_ok)

            if len(r.content) == 0:
                return None
            if len(r.content) == 5:
                if r.text == "NO_OK":
                    return None

            return r.content


            # except Exception as e:
            #     print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            #     exec(self.try_print_exception_itself_str)
            #     exec(self.try_print_exception_str)

        return None


    def pause(self, name="Pause", time=0.0, vwmapindex=10, zonemapindex=30, ppmapindex=None, t=-1):
        """

        Command AMR to pause current moving job (move to target, fllow path, follow rail, back to charge station)
        (paused job can be resumed)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param time: AMR will pause after given seconds, unit: seconds (Float)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "cmd"   : self.AI_PAUSE,
                                "data"  : str(time)
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
                
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def resume(self,name="Resume", vwmapindex=10, zonemapindex=30, ppmapindex=None, t=-1):
        """

        Command AMR to resume the paused moving job (move to target, fllow path, follow rail, back to charge station)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param vwmapindex: Index of used Virtual Wall map (valid index: 10~29)
        @param zonemapindex: Index of used Speed Limit map (valid index: 30~49)
        @param ppmapindex: Index of used Constant Obstacle map for path planning (valid index: 50)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                map_arr = []
                if vwmapindex not in map_arr:
                    map_arr.append(vwmapindex)

                if zonemapindex not in map_arr:
                    map_arr.append(zonemapindex)

                if ppmapindex is not None:
                    if ppmapindex not in map_arr:
                        map_arr.append(ppmapindex)

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "map": map_arr,  # 移動的過程需要參考的圖層 也可不參考
                                "cmd"   : self.AI_RESUME
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=MOVABLE_REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
                
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def __remote_repeat(self, v, w, repeatTime, name="Remote Repeat Stop", t=-1):
        self.__remoteRepeatStartTime=time.time();
        sendTime=time.time();
        self.__isRemoteRepeat=True;
        #print("__remote_repeat", v, w, repeatTime, self.__isRemoteRepeat)
        while (self.__isRemoteRepeat==True):
            #print("__remote_repeat, {ts}, {rr}".format(ts=time.time()-sendTime, rr=self.__remoteRepeatTime))
            if ( (time.time()-sendTime)>self.__remoteRepeatTime ):
                print("Remot Repeat Stop!!!")

                sendTime=time.time();
                url = self.__url+"/set_job"

                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd": self.STOP
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                self.__session.post(url, data=json.dumps(payload));

                break
            
            # loop 每 0.1s 跑一次
            time.sleep(0.1); 
            
        self.__isRemoteRepeat=False;
    
    
    
    def stop(self, name="Stop", emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to stop (as same as standby)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.STOP
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue

                exec(self.check_no_ok)
                return r.text
                
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None
    
    
    def standby(self, name="Standby", emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to standby (as same as stop)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(emrt):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.STANDBY
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue

                exec(self.check_no_ok)
                return r.text
                
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def blockstop(self, name="Stop", t=-1):
        """

        Command AMR to stop, return the result after "stop" is success, timeout of block is 60 seconds

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t:
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.STOP
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                for i in range(600):
                    time.sleep(0.1)
                    m=self.get_misc()
                    if m==None:
                        return m
                    if m["status"]["status"]==self.STOP:
                        return m
                return r.text
                
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def blockstandby(self, name="Standby", t=-1):
        """

        Command AMR to standby, return the result after "standby" is success, timeout of block is 60 seconds

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.STANDBY
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                for i in range(600):
                    time.sleep(0.1)
                    m=self.get_misc()
                    if m==None:
                        print("blockstandby get_misc() None!")
                        return m
                    if m["status"]["status"]==self.STANDBY:
                        return m

                return r.text
                
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_on(self, name="SlamOn", t=-1):
        """

        Turn "ON" the SLAM, SLAM must turn on when AMR is doing moving job or relocation

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(1):
            try:
                url = self.__url + "/set_job"
                payload = {"set_job": {
                    "name": name[0:MAX_NAME_LENGTH],
                    "cmd": 529
                },
                    "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code != 200:
                    print("====== {", sys._getframe().f_code.co_name, "} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def slam_off(self, name="SlamOff", t=-1):
        """

        Turn "OFF" the SLAM, SLAM always draw the new obstacle in time, do it if you don't want obstacle is generated too quickly during editing
        (REMEBER TO "TURN ON SLAM" BEFORE DOING MOVING JOB AND RELOCATION)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(1):
            try:
                url = self.__url + "/set_job"
                payload = {"set_job": {
                    "name": name[0:MAX_NAME_LENGTH],
                    "cmd": 530
                },
                    "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code != 200:
                    print("====== {", sys._getframe().f_code.co_name, "} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def scan(self, name="Scan", opt="", alarm_enable=False, emrt=EXCEPT_MAX_RETRY_TIMES, t=-1):
        """

        Command AMR to scan real-time obstacle nearby

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param opt: Various option for scanning (String)
                    "LASER" -> Laser scanning result only
                    "CAMERA" -> Camera scanning result only
                    "LASER & CAMERA" -> Both laser and camera result

        @param alarm_enable: Whether cancel the speaker alarm warning
        @param emrt: Max retry times for sending HTTP request (Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        pre_data = "@cxlBluez#"
        data = "@cxlLaserNorm#;@cxlCamera#"
        split_opt = opt.split("&")
        if len(split_opt) > 1:
            for o in split_opt:
                if "LASER" in o:
                    data = data.replace("@cxlLaserNorm#", "")

                if "CAMERA" in o:
                    data = data.replace("@cxlCamera#", "")

        else:
            if "LASER" in split_opt:
                data = data.replace("@cxlLaserNorm#","")

            if "CAMERA" in split_opt:
                data = data.replace("@cxlCamera#", "")

        data = data + pre_data

        if alarm_enable == False:
            data += "@cxlAlarm#"

        for trycount in range(emrt):
            try:
                url = self.__url + "/set_job"
                payload = {"set_job": {
                    "name": name[0:MAX_NAME_LENGTH],
                    "cmd": 32,
                    "data": data
                },
                    "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code != 200:
                    print("====== {", sys._getframe().f_code.co_name, "} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None




    # repeatTime 是否要自動 repeat repeatTime unit<s>
    def remote(self, v ,w, repeatTime=0, name="Remote", showtarget=None, t=-1):
        """

        Command AMR to moving through given velocity (v) and rotational speed (w)

        @param v: Velocity, unit: m/s (Float)
        @param w: Rotational speed, unit: rad/s (Float)
        @param repeatTime: Make command stop automatically after given seconds, unit: seconds (Float)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param showtarget: Follow target by given position and angle, [x, y, angle] (List)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        #print("remote v=", v, "w=", w);

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        try:
            if ( repeatTime==0 ):
                #如果原本的 thread 還在跑先把他停掉
                if ( self.__isRemoteRepeat==True ):
                    print("stop remote thread");
                    self.__isRemoteRepeat=False;
                    self.__remoteThread.join();

                if showtarget is None:
                    #對機器下達要移動的速度 and 角速度命令
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : self.REMOTE,
                                    "target"  : [
                                      {"x": v, "y": 0.0, "a": w} #要移動的速度量 unit: m/s, m/s, rad/s
                                    ],
                                    "data" : "@cxlMap#"
                                },
                             "mac": self.__myMac}

                    if t > -1:
                        payload['set_job']['time'] = int(t)

                    r=self.__session.post(url, data=json.dumps(payload))
                    exec(self.check_no_ok)
                
                else:    
                    #對機器下達要移動的速度 and 角速度命令
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : self.REMOTE,
                                    "target"  : [
                                      {"x": v, "y": 0.0, "a": w }, #要移動的速度量 unit: m/s, m/s, rad/s
                                      {"x": showtarget[0], "y": showtarget[1], "a": showtarget[2] }  
                                    ],
                                    "data": "@cxlMap#"
                                    
                                },
                             "mac": self.__myMac}

                    if t > -1:
                        payload['set_job']['time'] = int(t)

                    r=self.__session.post(url, data=json.dumps(payload))
                    exec(self.check_no_ok)

                return r.text
            else:
                self.__remoteV=v;
                self.__remoteW=w;

                #如果原本的 thread 還在跑先把他停掉
                if ( self.__isRemoteRepeat==True ):
                    print("stop remote thread");
                    self.__isRemoteRepeat=False;
                    self.__remoteThread.join();

                #對機器下達要移動的速度 and 角速度命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.REMOTE,
                                "target"  : [
                                  {"x": v, "y": 0.0, "a": w } #要移動的速度量 unit: m/s, m/s, rad/s
                                ],
                                "data": "@cxlMap#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r=self.__session.post(url, data=json.dumps(payload))
                exec(self.check_no_ok)

                print("start remote thread");
                self.__remoteRepeatStartTime = time.time();
                self.__remoteRepeatTime=repeatTime;
                self.__remoteThread=threading.Thread(target=self.__remote_repeat,  args=(v, w, repeatTime) );
                self.__remoteThread.start();
                
                return r.text

            ##為了安全 必須 client 端每秒送一次命令 不然 robot 端會視為斷線 將機器停下 
            #for i in range(30):
            #    self.__session.post(url, data=json.dumps(payload))
            #    time.sleep(1)

        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None

    # repeatTime 是否要自動 repeat repeatTime unit<s>
    def remote_cancelPCAll(self, v, w, repeatTime=0, name="Remote", t=-1):
        """

        Command AMR to moving with "NO SENSOR AVOIDING" through given velocity (v) and rotational speed (w)

        @param v: Velocity, unit: m/s (Float)
        @param w: Rotational speed, unit: rad/s (Float)
        @param repeatTime: Make command stop automatically after given seconds, unit: seconds (Float)
        @param name: Command name, it is always useful to check whether command is executed (String)
        @param showtarget: Follow target by given position and angle, [x, y, angle] (List)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        #print("remote v=", v, "w=", w);

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        try:
            if ( repeatTime==0 ):
                #如果原本的 thread 還在跑先把他停掉
                if ( self.__isRemoteRepeat==True ):
                    print("stop remote thread");
                    self.__isRemoteRepeat=False;
                    self.__remoteThread.join();
                
                #對機器下達要移動的速度 and 角速度命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.REMOTE,
                                "target"  : [
                                  {"x": v, "y": 0.0, "a": w } #要移動的速度量 unit: m/s, m/s, rad/s
                                ],
                                "data" : "@cxlLaserNorm#,@cxlCamera#,@cxlSonar#,@cxlMap#,@cxlBluez#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r=self.__session.post(url, data=json.dumps(payload))
                exec(self.check_no_ok)
                return r.text
            else:
                self.__remoteV=v;
                self.__remoteW=w;

                #對機器下達要移動的速度 and 角速度命令
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : self.REMOTE,
                                "target"  : [
                                  {"x": v, "y": 0.0, "a": w } #要移動的速度量 unit: m/s, m/s, rad/s
                                ], 
                                "data" : "@cxlLaserNorm#,@cxlCamera#,@cxlSonar#,@cxlMap#,@cxlBluez#"
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r=self.__session.post(url, data=json.dumps(payload))
                exec(self.check_no_ok)

                #如果原本的 thread 還在跑先把他停掉
                if ( self.__isRemoteRepeat==True ):
                    print("continue remote thread");
                    self.__remoteRepeatStartTime=time.time();
                    self.__remoteRepeatTime=repeatTime;
                else:
                    print("start remote thread");
                    self.__remoteRepeatTime=repeatTime;
                    self.__remoteThread=threading.Thread(target=self.__remote_repeat,  args=(v, w, repeatTime) );
                    self.__remoteThread.start();
                
                return r.text
            ##為了安全 必須 client 端每秒送一次命令 不然 robot 端會視為斷線 將機器停下 
            #for i in range(30):
            #    self.__session.post(url, data=json.dumps(payload))
            #    time.sleep(1)

        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None


    def set_map_resolution(self, name="Set Map Resolution", sc=None, t=-1):
        """

        Change map resolution, default is 0.02 m/pixel(or grid)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param sc: Map resolution, unit: m/pixel(or grid) (Float)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if (sc is not None):
            if type(sc) is float:

                print(f"set_map_resolution - resolution={sc}")

                name = self.AI_name_UUID_postfix(name)
                self.given_ai_name = name

                for trycount in range(EXCEPT_MAX_RETRY_TIMES):
                    try:
                        url = self.__url+"/set_job"
                        payload = { "set_job" : {
                                        "name"  : name[0:MAX_NAME_LENGTH],
                                        "cmd"   : 512,
                                        "map": [0, 0, 0],
                                        "target": [
                                            {"x": -1.0, "y": -1.0, "a": sc}
                                        ],
                                        "data"  : ""
                                        },
                                        "mac": self.__myMac}

                        if t > -1:
                            payload['set_job']['time'] = int(t)

                        r = requests.post(url, data=json.dumps(payload))
                        if r.status_code!=200:
                            print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                            continue
                        exec(self.check_no_ok)
                        return r.text
                    # except requests.exceptions.RequestException as e:
                    #     exec(self.try_print_exception_itself_str)
                    #     return None
                    except Exception as e:
                        print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                        exec(self.try_print_exception_itself_str)
                        exec(self.try_print_exception_str)
                return None

            return None

        return None


    #set_map_group("HY_Folder_6")
    def set_map_group(self,group_name):
        """

        Change to map folder by given name

        @param group_name: Name of map folder (String)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        for trycount in range(1):
            try:
                #設定使用的 group name 
                #舉例 一個建築物內有 5 層樓
                #一層樓內會包括 自己的地圖 數個 VW 數個 速限地圖
                #一層樓 就是一個 group
                #可令名圖成為
                # 1樓 Floor1
                # 2樓 Floor2
                # 3樓 Floor3
                # 以此類推
                url = self.__url+"/set_map_group"
                payload = {"mac": self.__myMac,
                           "name": group_name[0:MAX_NAME_LENGTH] }
                #print("set_map_group - mac:", self.__myMac, ", name:", group_name)         
                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def get_map_group(self, to=REQUEST_CONNECT_TIMEOUT):
        """

        Get current map folder's name

        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "NO_OK" -> Fail request (String)
                 "msi3F" -> Name of map folder (String)
        """
        #print("device - get_map_group")
        r=None
        data=None
        for trycount in range(1):
            try:
                #取得目前的 robot 所group name 
                #一層樓 就是一個 group
                #可令名圖成為
                # 1樓 Floor1
                # 2樓 Floor2
                # 3樓 Floor3
                # 以此類推
                url = self.__url+"/get_map_group"
                payload = {"mac": self.__myMac}
                r = requests.post(url, data=json.dumps(payload), timeout=to)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                data = r.json()["name"]
                return data

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def get_map_group_list(self):
        """

        Get all map folder

        @return:
            None          -> If timeout or disconnect (None)

            ["msi1F",
             "msi2F",
             "msi3F",...] -> List of map folders' name (List)
        """
        folderNameList=[];
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            #print("device - get_map_group_list")
            try:
                ssh = paramiko.SSHClient();
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
                ssh.connect(hostname=self.__ip , port=22, username=self.__sshName, password=self.__sshPwd);
                stdin, stdout, stderr = ssh.exec_command('ls -d /data/map8192/*/'); #find all folder only
                result = stdout.read();
                ssh.close();

                folder=result.decode();
                folders=folder.split('\n');
                for i in folders:
                    if ( len(i)!=0 ):
                        r=i.split('/');
                        folderNameList.append(r[3]);
                return folderNameList;
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None
            

    def del_map_group(self, folderName):
        """

        Delete map folder by given name

        @param folderName: Name of map folder (String)
        @return:
                None -> No matter success or fail (None)
        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                # rmFolderName="rm -rf /data/map8192/" + folderName;
                # print("del_map_group - rm folder name cmd=", rmFolderName);
                #
                # rmFolderNameAcl="rm -rf /data/map8192_acl/" + folderName;
                # print("del_map_group - rm folder name acl cmd=", rmFolderNameAcl);
                
                ssh = paramiko.SSHClient();
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
                ssh.connect(hostname=self.__ip , port=22, username=self.__sshName, password=self.__sshPwd);
                stdin, stdout, stderr = ssh.exec_command(
                    'rm -rf /data/map8192/{folder_name}'.format(folder_name=folderName));
                result = stdout.read()
                result_str = result.decode()

                stdin, stdout, stderr = ssh.exec_command(
                    'rm -rf /data/map8192_acl/{folder_name}'.format(folder_name=folderName));
                result = stdout.read()
                result_str = result.decode()

                stdin, stdout, stderr = ssh.exec_command(
                    'rm -rf /data/AI_ShapePP/{folder_name}'.format(folder_name=folderName));
                result = stdout.read()
                result_str = result.decode()

                ssh.close();

                print("del_map_group - remove folderName=", folderName);

                return None

            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def del_func_map(self, folderName, idx):
        """

        Delete functional map by given map folder name and functional map's index

        @param folderName: Name of map folder (String)
               idx: Functional map's index (Int)
        @return:
                None -> No matter success or fail (None)
        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                # rmFolderName="rm -rf /data/map8192/" + folderName;
                # print("del_map_group - rm folder name cmd=", rmFolderName);
                #
                # rmFolderNameAcl="rm -rf /data/map8192_acl/" + folderName;
                # print("del_map_group - rm folder name acl cmd=", rmFolderNameAcl);

                ssh = paramiko.SSHClient();
                ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
                ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd);
                stdin, stdout, stderr = ssh.exec_command(
                    'rm /data/map8192/{folder_name}/map_{idx}.*'.format(folder_name=folderName, idx=idx));
                result = stdout.read()
                result_str = result.decode()

                ssh.close();

                print("del_func_map - remove folderName =", folderName);
                print("del_func_map - functional map's index =", idx);

                return None

            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def get_func_map_list(self, group_name=None):
        """

        Get function map (Virtual Wall, Speed Zone...) list of the map folder by given name

        @param group_name: Name of map folder
        @return:
                None               -> If timeout or disconnect (None)

                ["map_0.map",
                 "map_10.map",
                 "map_30.map",...] -> List of function map's name (List)
        """
        mapNameList=[];
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                assigned_group = None
                if group_name is None:
                    assigned_group = self.get_map_group()
                else:
                    if type(group_name) is str:
                        assigned_group = group_name

                if assigned_group is not None:
                    ssh = paramiko.SSHClient();
                    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
                    ssh.connect(hostname=self.__ip , port=22, username=self.__sshName, password=self.__sshPwd);

                    exec_cmd_str = 'ls -1 /data/map8192/{ag}/'.format(ag=assigned_group)
                    print("exec_cmd_str=", exec_cmd_str)

                    stdin, stdout, stderr = ssh.exec_command(exec_cmd_str); #find all folder only
                    result = stdout.read();
                    ssh.close();

                    maplist_raw=result.decode();
                    maplist=maplist_raw.split('\n');
                    for m in maplist:
                        if ( len(m)!=0 ):
                            if ".map" in m:
                                mapNameList.append(m);

                return mapNameList;

            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None


    #Device.get_map(0,4096,4096,10,10)
    def get_map(self, mapindex, xstart, ystart, width, height):
        """

        Get map content by given map index
        ( map index     0: Scanned raw map
                    10~29: Virtual Wall map
                    30~49: Speed Limitation map
                       50: Obstacle map for path planning )

        @param mapindex: Map index (Int)
        @param xstart: Request START of map coordinate X, unit: grid or pixel (Int)
        @param ystart: Request START of map coordinate Y, unit: grid or pixel (Int)
        @param width:  Request width, unit: grid or pixel (Int)
        @param height: Request height, unit: grid or pixel (Int)
        @return:
                None                -> If timeout or disconnect (None)

                [ [0,0,0...,1,1],
                  [0,1,0...,1,0],
                  [0,0,1...,0,1],
                        .
                        .
                        .
                  [0,0,0...,0,0] ]  -> A 2D array, its width and height is the given width and height (Numpy Array)
        """
        for trycount in range(1):
            try:
                #get slam map
                # mapIndex data range 0~49
                # 0 代表的是 SLAM 地圖
                # 10~29 是 VW 的圖層
                # 30~49 是 速限地圖所使用的範圍
                #print("device get_map: {xst},{yst},{w},{h}".format(xst=xstart, yst=ystart, w=width, h=height))
                mapIndex=mapindex
                x=xstart
                y=ystart
                url = self.__url+"/get_map"
                payload = { "get_map": {
                                "index" : mapIndex,
                                "x": x,
                                "y": y, 
                                "width": width,
                                "height": height
                                },
                        "mac": self.__myMac}
                        
                #t1 = time.time();
                #print("Device r pre:",t1)
                r = self.__session_map.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                #t1 = time.time();
                #print("Device r after:",t1)
                
                #print("len(r.content):", len(r.content))
                if len(r.content)==0:
                    #print(mapindex,xstart,ystart,width,height)
                    return np.zeros((0,0),dtype=np.int8)
                if len(r.content)==5:
                    if r.text=="NO_OK":
                        #print(mapindex,xstart,ystart,width,height)
                        return np.zeros((0,0),dtype=np.int8)
                #print(r.text)
                #t1 = time.time();
                #print("Device z pre:",t1)
                mapl = zlib.decompress(r.content) #由於傳回的地圖是被壓縮過的資料 收到資料後必須解壓縮後才可以得到地圖 array
                
                #t1 = time.time();
                #print("Device z aftr:",t1)
                map_ = np.fromstring(mapl, dtype = np.int8)
                map_reshape=map_.reshape((height,width))
                
                #t1 = time.time();
                #print("Device x aftr:",t1)
                return map_reshape

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def set_map(self,mapindex,mapall):
        """

        Set map by given map index and numpy 2D array with value inside

        @param mapindex: Map index (Int)
        @param mapall: Numpy 2D array (Numpy Array)
        @return:
                None    -> If timeout or disconnect (None)
                "OK"    -> Success request (String)
                "NO_OK" -> Fail request (String)
        """
        try:
            #modify slam map and set_map
            #一次只能回存一整張地圖
            mapIndex=mapindex
            x=0
            y=0
            #width =  map_width
            #height = map_height
            (height,width)=mapall.shape
            #print(height,width)
            url = self.__url+"/set_map"
            #map_ = np.fromstring(mapall, dtype = np.int8)
            #map_ = map_*(-1)
            mapBytes=bytes(mapall)
            imgCompress=zlib.compress(mapBytes) #將地圖資料壓縮
            #print("map size", len(imgCompress))
            imgCompress64 = b64encode(imgCompress) #受限 JSON 不支持 binary 傳輸資料只 能先轉成 字串形式來做傳輸
            imgCompress64Str = imgCompress64.decode(encoding='utf-8')
            payload = { "set_map": {
                            "index" : mapIndex,
                            "x": x,
                            "y": y,
                            "width": width,
                            "height": height,
                            "map": imgCompress64Str
                            },
                        "mac": self.__myMac}
            data=json.dumps(payload)
            r = requests.post(url, data)
            if r.status_code!=200:
                print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")

            exec(self.check_no_ok)
            return r.text
        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)

        return None

    ##get sensor information
    def get_sensor_info(self):
        """

        Get sensor information

        @return:

                None      -> If timeout or disconnect (None)
                "NO_OK"   -> Fail request (String)
                JSON data ->
                            { "laser":
                                [
                                    { "id": 0,
                                      "pose": [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ],
                                      "min_angle": -3.14,
                                      "max_angle": 3.14,
                                      "resolution": 0.1,
                                      "max_range": 20.0,
                                      "scanning_frequency": 40 },

                                    { "id": 1,
                                      "pose": [ 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 ],
                                      "min_angle": -1.5,
                                      "max_angle": 1.5,
                                      "resolution": 0.01,
                                      "max_range": 20.0,
                                      "scanning_frequency": 40 },
                                ]
                            }

                            id -> Laser device index (Int)
                            pose -> List of local coordinate (List)
                            pose[0] -> Local coordinate: x of AMR, unit: m (Float)
                            pose[1] -> Local coordinate: y of AMR, unit: m (Float)
                            pose[2] -> Local coordinate: z of AMR, unit: m (Float)
                            pose[3] -> Local coordinate: roll of AMR, unit: radian (Float)
                            pose[4] -> Local coordinate: pitch of AMR, unit: radian (Float)
                            pose[5] -> Local coordinate: yaw of AMR, unit: radian (Float)
                            min_angle -> Sensor measure minimum angle, unit: radian (Float)
                            max_angle -> Sensor measure maximum angle, unit: radian (Float)
                            resolution -> Sensor measure angle one step, unit: radian (Float)
                            max_range -> Distance measure maximum range, unit: m (Float)
                            scanning_frequency -> Sensor update frequency, unit: Hz (Float)

        """
        r=None
        data=None
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):        
            try:
                url = self.__url+"/get_sensor_info"
                payload = {"mac": self.__myMac}
                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                data = r.json()
                return data
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    ##get sound list
    def get_sound(self):
        """
        ABANDONED. DO NOT USE IT
        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/get_sound"
                payload = {}
                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)

                json_data = r.json()
                return json_data['value']
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    ##get sound list
    def get_sound_ssh(self):
        """

        Get sound file list

        @return:
                None    -> If timeout or disconnect (None)

                [
                 '[ai]emergency_button_pressed.mp3',
                 '[ai]pause.mp3', '[ai]resume.mp3',
                 '[ai]task_fail.mp3',
                 '[ai]task_start.mp3',
                 '[ai]task_stop.mp3',
                 ...
                ]       -> List of sound files' name (List)
        """
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd);
            cmd0 = "ls /data/wav"

            sound_list = []
            stdin, stdout, stderr = ssh.exec_command(cmd0);
            stdout.channel.recv_exit_status();  # block here
            result = stdout.read();
            ssh.close();

            sound_list_txt = result.decode('utf-8')
            sound_list = sound_list_txt.split("\n")

            filter_list = []
            for sn in sound_list:
                if ".mp3" in sn:
                    filter_list.append(sn)

            return filter_list

        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            # exec(self.try_print_exception_str)

        return None


    ##get laser data  
    def get_laser(self, laserId=0):
        """

        Get laser data by given laser device index

        @param laserId: Laser device index
        @return:
                None    -> If timeout or disconnect (None)

                Numpy 2D array ->
                                    [
                                     [2.7, 2.7, ..., 7.0],
                                     [1.0, 1.0, ..., 1.0]
                                    ]

                                   first row is measured distance, second is measured intensity
                                   distance, unit: m, (Float)
                                   intensity, unit: unknown (Float)
        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                url = self.__url+"/get_laser"
                payload = { "get_laser": 
                            {
                              "id": laserId,
                            },
                            "mac": self.__myMac
                          }
                r = self.__session.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                laser = np.fromstring(r.content , dtype = np.float32 )  
                laser=laser.reshape((2, int(len(laser)/2)));
                return laser
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None
        
    def logout(self):
        """
        ABANDONED, DO NOT USE
        """
        try:
            self.__isAlive=False
            url = self.__url+"/logout"
            payload = {"logout": {"mac": self.__myMac,
                                 "password": "foobar",
                                 "brand": "htc",
                                 "model": "HTC Butterfly",
                                 "sdk": "19"} }
            #print(json.dumps(payload))
            r = self.__session.post(url, data=json.dumps(payload))
            exec(self.check_no_ok)
            return r.text
        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None

    
    def set_name(self, name):
        """
        Set AMR's name

        @param name: Name (String)
        """
        sftp = None;
        trans = None;
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if infoSplitColon[0] == "name":
                    # print("name=", infoSplitColon[1]);
                    to_replace_str = i;

            wrote_name = "name:" + name
            info = info.replace(to_replace_str, "replace_name")
            info = info.replace("replace_name", wrote_name)

            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except:
            if sftp is not None:
                sftp.close()

            if trans is not None:
                trans.close()
    
    def get_name(self):
        """

        Get AMR's name

        @return:
                None    -> If timeout or disconnect (None)

                "msi_1" -> Name (None)
        """
        name = None;
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):                
            try:
                trans = paramiko.Transport((self.__ip, 22))
                trans.start_client();
                trans.auth_password(username=self.__sshName, password=self.__sshPwd);
                sftp = paramiko.SFTPClient.from_transport(trans);

                info_file = sftp.open("info.cfg", "r")
                info = info_file.read().decode('utf-8');
                infoSplitEnter = info.split("\n");
                for i in infoSplitEnter:
                    infoSplitColon = i.split(":");
                    if infoSplitColon[0]=="name":
                        #print("name=", infoSplitColon[1]);
                        name = infoSplitColon[1];

                sftp.close();
                trans.close()
                return name;
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                #exec(self.try_print_exception_str)
        return None


    def get_info_size(self):
        """

        Get AMR's size from config file
        (AMR will write the size from config file to AI automatically after booting)

        @return:
                None    -> If timeout or disconnect (None)

                (0.85, 0.85, 1.9, 0.3, 0.24) -> Group of size info (Tuple)

                                                (length, width, height, ignore below, center to front, charge pad offset)

                                                 length, unit: m (Float)
                                                 width, unit: m (Float)
                                                 height, unit: m (Float)
                                                 ignore below, unit: m (Float)
                                                 center to front, unit: m (Float)
                                                 charge pad offset (in relative), unit: m (Float)
        """
        length = None
        width = None
        height = None
        bottom_height = None
        center_to_front = None
        dock_plug_offset = None

        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport(trans);

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');
            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if infoSplitColon[0] == "length":
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            length = float(infoSplitColon[1])
                        except ValueError:
                            length = None

                if infoSplitColon[0] == "width":
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            width = float(infoSplitColon[1])
                        except ValueError:
                            width = None

                if infoSplitColon[0] == "height":
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            height = float(infoSplitColon[1])
                        except ValueError:
                            height = None

                if infoSplitColon[0] == "bottom_height":
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            bottom_height = float(infoSplitColon[1])
                        except ValueError:
                            bottom_height = None

                if infoSplitColon[0] == "center_to_front":
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            center_to_front = float(infoSplitColon[1])
                        except ValueError:
                            center_to_front = None

                if infoSplitColon[0] == "dock_plug_offset":
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            dock_plug_offset = float(infoSplitColon[1])
                        except ValueError:
                            dock_plug_offset = None

            sftp.close();
            trans.close()

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            # exec(self.try_print_exception_str)
            # return None

        return length, width, height, bottom_height, center_to_front, dock_plug_offset


    def set_info_size(self, length=None, width=None, height=None, bottom_height=None, center=None, \
                      dock_plug_offset=None):
        """

        Set AMR's size to config file
        (AMR will write the size from config file to AI automatically after booting)

        @param length: length, unit: m (Float)
        @param width: width, unit: m (Float)
        @param height: height, unit: m (Float)
        @param bottom_height: Ignore below, unit: m (Float)
        @param center: Center to front, unit: m (Float)
        """
        sftp = None;
        trans = None;
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            to_replace_str_exist = [0, 0, 0, 0, 0, 0]
            to_replace_str = ["","","","","",""]

            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");

                if infoSplitColon[0] == "length":
                    to_replace_str_exist[0] = 1;
                    to_replace_str[0] = i

                elif infoSplitColon[0] == "width":
                    to_replace_str_exist[1] = 1;
                    to_replace_str[1] = i

                elif infoSplitColon[0] == "height":
                    to_replace_str_exist[2] = 1;
                    to_replace_str[2] = i

                elif infoSplitColon[0] == "bottom_height":
                    to_replace_str_exist[3] = 1;
                    to_replace_str[3] = i

                elif infoSplitColon[0] == "center_to_front":
                    to_replace_str_exist[4] = 1;
                    to_replace_str[4] = i

                elif infoSplitColon[0] == "dock_plug_offset":
                    to_replace_str_exist[5] = 1;
                    to_replace_str[5] = i


            if length is not None:
                if to_replace_str_exist[0] == 1:
                    wrote_info = "length:" + "%.3f"%length
                    info = info.replace(to_replace_str[0], "replace_name")
                    info = info.replace("replace_name", wrote_info)
                else:
                    wrote_info = "length:" + "%.3f"%length
                    info += "\n" + wrote_info


            if width is not None:
                if to_replace_str_exist[1] == 1:
                    wrote_info = "width:" + "%.3f"%width
                    info = info.replace(to_replace_str[1], "replace_name")
                    info = info.replace("replace_name", wrote_info)
                else:
                    wrote_info = "width:" + "%.3f"%width
                    info += "\n" + wrote_info


            if height is not None:
                if to_replace_str_exist[2] == 1:
                    wrote_info = "height:" + "%.3f"%height
                    info = info.replace(to_replace_str[2], "replace_name")
                    info = info.replace("replace_name", wrote_info)
                else:
                    wrote_info = "height:" + "%.3f"%height
                    info += "\n" + wrote_info


            if bottom_height is not None:
                if to_replace_str_exist[3] == 1:
                    wrote_info = "bottom_height:" + "%.3f"%bottom_height
                    info = info.replace(to_replace_str[3], "replace_name")
                    info = info.replace("replace_name", wrote_info)
                else:
                    wrote_info = "bottom_height:" + "%.3f"%bottom_height
                    info += "\n" + wrote_info

            if center is not None:
                if to_replace_str_exist[4] == 1:
                    wrote_info = "center_to_front:" + "%.3f"%center
                    info = info.replace(to_replace_str[4], "replace_name")
                    info = info.replace("replace_name", wrote_info)
                else:
                    wrote_info = "center_to_front:" + "%.3f"%center
                    info += "\n" + wrote_info

            if dock_plug_offset is not None:
                if to_replace_str_exist[5] == 1:
                    wrote_info = "dock_plug_offset:" + "%.3f"%dock_plug_offset
                    info = info.replace(to_replace_str[5], "replace_name")
                    info = info.replace("replace_name", wrote_info)
                else:
                    wrote_info = "dock_plug_offset:" + "%.3f"%dock_plug_offset
                    info += "\n" + wrote_info


            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except:
            if sftp is not None:
                sftp.close()

            if trans is not None:
                trans.close()


    def set_type(self, type):
        """

        Set AMR's type to config file (Only type=0 can be managed by FMS)

        @param type: type (Int)
        """
        trans = paramiko.Transport((self.__ip, 22))
        trans.start_client();
        trans.auth_password(username=self.__sshName, password=self.__sshPwd);
        sftp = paramiko.SFTPClient.from_transport(trans);

        info_file = sftp.open("info.cfg", "r")
        info = info_file.read().decode('utf-8');

        infoSplitEnter = info.split("\n");
        for i in infoSplitEnter:
            infoSplitColon = i.split(":");
            if infoSplitColon[0] == "type":
                # print("name=", infoSplitColon[1]);
                to_replace_str = i;

        wrote_name = "type:" + str(type)
        info = info.replace(to_replace_str, "replace_name")
        info = info.replace("replace_name", wrote_name)

        sftp.open("info.cfg", "w").write(info);

        sftp.close();
        trans.close();

    def get_type(self):
        """

        Get AMR's type from config file (Only type=0 can be managed by FMS)

        @return:
                None    -> If timeout or disconnect (None)

                0       -> Type (Int)
        """
        type = None;
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                trans = paramiko.Transport((self.__ip, 22))
                trans.start_client();
                trans.auth_password(username=self.__sshName, password=self.__sshPwd);
                sftp = paramiko.SFTPClient.from_transport(trans);

                info_file = sftp.open("info.cfg", "r")
                info = info_file.read().decode('utf-8');
                infoSplitEnter = info.split("\n");
                for i in infoSplitEnter:
                    infoSplitColon = i.split(":");
                    if infoSplitColon[0] == "type":
                        # print("name=", infoSplitColon[1]);
                        type = infoSplitColon[1];

                sftp.close();
                trans.close()

                type = -1
                try:
                    type = int(type)
                except:
                    pass

                return type;

            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                # exec(self.try_print_exception_str)
        return None




    #add by leon
    #prepare for change pyuser passwd
    def passwd(self,current,now):
        """

        Change AMR's password of account: pyuser

        @param current: Current Password (String)
        @param now: New Password (String)
        """
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd);
            cmd0=r'echo -e "'+current+r'\n'+now+r'\n'+now+r'" | passwd;'
            print(cmd0)
            stdin, stdout, stderr = ssh.exec_command(cmd0);
            stdout.channel.recv_exit_status() ; #block here
            ssh.close();


        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None


    def start_update(self):
        """

        Command AMR to update uploaded firmware

        """
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="sudo systemctl start player.update";
            stdin, stdout, stderr = ssh.exec_command(cmd);
            ssh.close();
            

        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

    #add by leon
    #prepare for update F.W
    def reboot(self, isUseOldMethod=False):
        """

        Command AMR to reboot

        @param isUseOldMethod: Use whether old method for old firmware, NO MORE DIFFERENT NOW (Boolean)
        """
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd0="sudo systemctl stop taskmanager;sudo systemctl stop player*;";
            stdin, stdout, stderr = ssh.exec_command(cmd0); #stop player service
            stdout.channel.recv_exit_status() ; #block here
            if isUseOldMethod==True:
                cmd1 = "sudo systemctl reboot;"
            else:
                cmd1 = "sudo systemctl reboot;"
            
            stdin, stdout, stderr = ssh.exec_command(cmd1);
            #stdout.channel.recv_exit_status() ; #block here
            time.sleep(5);
            ssh.close();
            

        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

    def get_wifi_mac(self, device="wlan0"):
        """

        Get AMR's WIFI MAC address

        @return:
                "4c:1d:96:a5:f6:36" -> MAC address, empty string returned when AMR is disconnect (String)
        """
        wifi_mac = ""

        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip,
                        port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False,
                        look_for_keys=False);
            cmd = "cat /sys/class/net/" + device + "/address";
            stdin, stdout, stderr = ssh.exec_command(cmd);  # find all folder only
            result = stdout.read();
            ssh.close();



        except Exception as e:
            print("Exception ", Exception);

        return result.decode();


    #test OK@2019/08/16
    def get_wifi_ap_list(self, interface='wlan0'):
        """

        Get WIFI AP list scanned by AMR

        @param interface: Network interface, AMR only has 'wlan0' as default (String)
        @return:
                [
                  ['d8:0d:17:84:c2:17', 'fc:34:97:70:80:d4', ... , '34:8a:12:d2:59:b0'],
                  ['-32', '-36', ... , '-78'],
                  ['october_mesh_1', 'MSI-DOA-5G', ... , 'msi-AP'],
                  ['5240', '5745', ... , '5500']
                ]

                -> List of WIFI APs' MAC address, signal strength, SSID, frequency
                   [
                     [ap1_mac, ap2_mac, ...],
                     [ap1_strength, ap2_strength, ...],
                     [ap1_ssid, ap2_ssid, ...],
                     [ap1_freq, ap2_freq, ...]
                   ]

                   MAC address -> (String)
                   signal strength -> unit: db (String)
                   SSID -> (String)
                   frequency -> (String)

        """
        #必須先決定是用Ubuntu 14.04或Ubuntu 16.04; wpa_supplicant或nmcli，這裡回傳的東西會不一樣，會需要重parse
        apMac=[];
        signalLevel=[];
        ssid=[];
        frequency=[];

        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd);
            cmd0="sudo wifi_robot " + interface + " enable_network";
            cmd1="sudo wpa_cli -i " + interface + " scan";
            cmd2="sudo wpa_cli -i " + interface + " scan_results";

            stdin, stdout, stderr = ssh.exec_command(cmd0);
            stdout.channel.recv_exit_status() ; #block here
            if ( stdout.read().decode()!= "OK" ):
                pass;#TODO: retry again
            
            stdin, stdout, stderr = ssh.exec_command(cmd1);
            stdout.channel.recv_exit_status() ; #block here
            time.sleep(1)
            
            stdin, stdout, stderr = ssh.exec_command(cmd2); #find all folder only
            stdout.channel.recv_exit_status() ; #block here
            result = stdout.read();
            wlanScan=result.decode();       
            ssh.close();
            
            print(wlanScan)
            walnScanLine=wlanScan.splitlines();
            for line in walnScanLine[1:]:
                slplited=line.split('\t', 5);
                apMac.append(slplited[0]);
                frequency.append(slplited[1]);
                signalLevel.append(slplited[2]);
                ssid.append(slplited[4].lstrip());                 

        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

        return apMac, signalLevel, ssid, frequency;
    
    #取得目前連上的無線 AP ssid
    def get_wifi_ap_ssid(self, interface='wlan0'):
        """

        Get AMR current connected WIFI AP

        @param interface: Network interface, AMR only has 'wlan0' as default (String)
        @return:
                None                -> If timeout or disconnect (None)

                'october_mesh_1\\n'  -> Connected WIFI AP (String)
        """
        ssid=None;
        
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="iwgetid -r";
            stdin, stdout, stderr = ssh.exec_command(cmd); #find all folder only
            stdout.channel.recv_exit_status() ; #block here
            result = stdout.read();
            ssid = result.decode();

            if len(ssid) == 0:
                cmd = "sudo iwgetid -r";
                stdin, stdout, stderr = ssh.exec_command(cmd);  # find all folder only
                stdout.channel.recv_exit_status();  # block here
                result = stdout.read();
                ssid = result.decode();

            ssh.close();

        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None
        return ssid;
    
    #取得目前連上的無線 AP 的 IP    
    def get_wifi_info(self, interface='wlan0'):
        """

        Get AMR current IP and gateway

        @param interface: Network interface, AMR only has 'wlan0' as default (String)
        @return:
                ('172.16.210.53 ', '172.16.208.2') -> Tuple of IP and gateway (Tuple)
                                                      IP -> (String)
                                                      Gateway -> (String)

        """
        ip = "";
        gateway = ""

        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="ip r";
            stdin, stdout, stderr = ssh.exec_command(cmd); #find all folder only
            stdout.channel.recv_exit_status() ; #block here
            result = stdout.read().decode();
            ssh.close();
            
            lines=result.splitlines();
            
            findstr="default via ";
            start=lines[0].find(findstr);
            start=start+len(findstr);  
            
            findstr=" dev ";
            end=lines[0].find(" dev ");
            gateway=lines[0][start:end];           
            
            findstr="dev " + interface + " proto kernel scope link src "
            for line in lines:
                start=line.find(findstr);
                if ( start>0 ):
                    ip=line[start+len(findstr):];
                    break;

        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

        return ip, gateway;
        
    def get_wifi_dns(self):
        """
        UNDONE, DO NOT USE
        """
        #必須先決定是用Ubuntu 14.04或Ubuntu 16.04; wpa_supplicant或nmcli，這裡回傳的東西會不一樣，會需要重parse
        #TODO - Really get the wifi dns
        return "192.168.1.1" #完成前，先餵假的

    def get_wifi_fullinfo(self, interface='wlan0'):
        """

        Get full configuration of AMR current connected WIFI AP

        @param interface: Network interface, AMR only has 'wlan0' as default (String)
        @return:
                ('_dhcp', '', '', '', '', '', '', ['october_mesh_1', '0232345599'])

                -> Group of Protocol, IP address, Netmask, Network, Broadcast, Gateway, DNS, SSID, Password (Tuple)
                   Protocol -> '_dhcp' or '_static' (String)
                   IP address -> (String)
                   Netmask -> (String)
                   Network -> (String)
                   Broadcast -> (String)
                   Gateway -> (String)
                   DNS -> ['8.8.8.8', dns_ip_2, dns_ip_3] (List of String)
                   [SSID, Password] -> (List of String)

        """
        ssh = paramiko.SSHClient();
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
        ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False,
                    look_for_keys=False);

        file = r"/data/etc/network/interfaces.d/" + interface + ".cfg";
        cmd = r"cat " + file
        stdin, stdout, stderr = ssh.exec_command(cmd);  # find all folder only
        cat_network_result = stdout.read();

        file = r"/data/etc/wpa_supplicant/wpa_supplicant.conf";
        cmd = r"cat " + file
        stdin, stdout, stderr = ssh.exec_command(cmd);  # find all folder only
        cat_wpa_result = stdout.read();

        ssh.close();

        settled_ssid_info = ['','']
        ip_config_type = "_dhcp"
        cat_wpa_result = cat_wpa_result.decode().split('\n')
        print("cat_wpa_result = ")
        print(cat_wpa_result)
        block_count = 0
        block_start = 0
        for cwr in cat_wpa_result:
             print("cwr =", cwr)
             if "{" in cwr:
                 block_start = 1

             if block_start==1:
                 if "id_str=" in cwr:
                     #print(cwr)
                     ip_config_type = cwr.split("=")[1]
                     ip_config_type = ip_config_type.replace("\"", "")

                 if "ssid=" == cwr[0:5]:
                     used_ssid_str = cwr.split("=")[1]
                     used_ssid_str = used_ssid_str.replace("\"", "")
                     settled_ssid_info[0] = used_ssid_str

                 if "psk=" == cwr[0:4]:
                     used_psk_str = cwr.split("=")[1]
                     used_psk_str = used_psk_str.replace("\"", "")
                     settled_ssid_info[1] = used_psk_str

             if "}" in cwr:
                 block_start = 0
                 block_count += 1

             if block_count==1:
                 break

        dns_str_list = []
        address_str = ""
        netmask_str = ""
        network_str = ""
        broadcast_str = ""
        gateway_str = ""

        cat_network_result = cat_network_result.decode().split('\n')
        print("cat_network_result = ")
        print(cat_network_result)
        for cnr in cat_network_result:
            cnr = cnr.lstrip()
            print("cnr =", cnr)
            if ("dns-nameserver" in cnr) and ('#' not in cnr):
                #print(cnr)
                cnr_split = cnr.split(" ")
                for i,sstr in enumerate(cnr_split):
                    if (i > 0) and (sstr!=""):
                        dns_str_list.append(sstr)

            if ("address" in cnr) and ('#' not in cnr):
                cnr_split = cnr.split(" ")
                address_str = cnr_split[1]

            if ("netmask" in cnr) and ('#' not in cnr):
                cnr_split = cnr.split(" ")
                netmask_str = cnr_split[1]

            if ("network" in cnr) and ('#' not in cnr):
                cnr_split = cnr.split(" ")
                network_str = cnr_split[1]

            if ("broadcast" in cnr) and ('#' not in cnr):
                cnr_split = cnr.split(" ")
                broadcast_str = cnr_split[1]

            if ("gateway" in cnr) and ('#' not in cnr):
                cnr_split = cnr.split(" ")
                gateway_str = cnr_split[1]

        return ip_config_type, address_str, netmask_str, network_str, \
               broadcast_str, gateway_str, dns_str_list, settled_ssid_info


    def set_wifi_ip(self, interface='wlan0',
                    address='172.16.98.100',
                    netmask='255.255.240.0',
                    broadcast='172.16.127.255',
                    network='',
                    gateway='',
                    dnsservers='',
                    use_dhcp=True):
        """

        Set WIFI network configuration

        @param interface: Network interface, AMR only has 'wlan0' as default (String)
        @param address: IP address (String)
        @param netmask: (String)
        @param broadcast: (String)
        @param network: (String)
        @param gateway: (String)
        @param dnsservers: Can add more than one DNS, use SPACE to seperate different DNS (String)
        @param use_dhcp: Whether use DHCP or not (Boolean)
        @return:
                None                -> If timeout or disconnect (None)

                "..."               -> Result text (String)

        """
        result = None

        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            file=r"/data/etc/network/interfaces.d/"+interface+".cfg";
            temfile=r"/tmp/"+interface+".cfg";
            
            cmdList=[]
            cmdList.append(r"echo 'allow-hotplug wlan0' > " + temfile);
            cmdList.append(r"echo 'iface wlan0 inet manual' >> " + temfile);
            cmdList.append(r"echo 'wpa-driver wext' >> " + temfile);
            cmdList.append(r"echo 'wpa-roam /data/etc/wpa_supplicant/wpa_supplicant.conf' >> " + temfile);            
            cmdList.append(r"echo 'wireless-power off' >> " + temfile);
            cmdList.append(r"echo 'iface _static inet static' >> " + temfile);  
            
            if (address!=''):
                tmp="    address "+ address;
                cmdList.append(r"echo '"+ tmp + "' >> " + temfile);
            if (netmask!=''):
                tmp="    netmask "+ netmask;
                cmdList.append(r"echo '"+ tmp + "' >> " + temfile);
            if (network!=''):
                tmp="    network "+ network;
                cmdList.append(r"echo '"+ tmp + "' >> " + temfile);
            if (broadcast!=''):
                tmp="    broadcast "+ broadcast;
                cmdList.append(r"echo '"+ tmp + "' >> " + temfile);
            if (gateway!=''):
                tmp="    gateway "+ gateway;
                cmdList.append(r"echo '"+ tmp + "' >> " + temfile);
            if (dnsservers!=''):
                tmp="    dns-nameservers "+ dnsservers;
                cmdList.append(r"echo '"+ tmp + "' >> " + temfile);

            cmdList.append(r"echo 'iface _dhcp inet dhcp' >> " + temfile);              
            cmdList.append(r"cp -f "+ temfile + " " + file );  

            cmd=""
            for c in cmdList:
                cmd=cmd+c+';'

            stdin, stdout, stderr = ssh.exec_command(cmd); #find all folder only
            result = stdout.read();
            ssh.close();
        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None
        
        return result;

    def connect_wifi_reconnect(self, interface='wlan0'):
        """

        Command AMR's WIFI interface to do reconnection

        @param interface: Network interface, AMR only has 'wlan0' as default (String)

        @return:
                None                -> If timeout or disconnect (None)

                "..."               -> Result text (String)
        """

        try:

            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False,
                        look_for_keys=False);

            wifiReconfigureCmd = "sudo wifi_robot " + interface + " reconfigure";
            stdin, stdout, stderr = ssh.exec_command(wifiReconfigureCmd);
            stdout.channel.recv_exit_status();  # block here

            print("stdout=", stdout.read().decode("ascii"))

            ssh.close();


        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)

        return result;


    def connect_wifi_ap(self, interface='wlan0', 
                        ssid=None, 
                        pwd=None, 
                        priority=255, 
                        address='',
                        netmask='',
                        broadcast='',
                        network='',
                        gateway='',
                        dnsservers='',
                        use_dhcp=True,
                        config=None):
        """

        Command AMR to connect to a WIFI AP (Wrap "set_wifi_ip" function)

        @param interface: Network interface, AMR only has 'wlan0' as default (String)
        @param ssid: SSID of WIFI AP (String)
        @param pwd: Password of WIFI AP (String)
        @param priority: (Int)
        @param address: IP address (String)
        @param netmask: (String)
        @param broadcast: (String)
        @param network: (String)
        @param gateway: (String)
        @param dnsservers: Can add more than one DNS, use SPACE to seperate different DNS (String)
        @param use_dhcp: Whether use DHCP or not (Boolean)
        @param config: Ignore all provided parameters, write the whole custom config (String)
        @return:
                None                -> If timeout or disconnect (None)

                "..."               -> Result text (String)
        """
        self.set_wifi_ip(interface,address,netmask,broadcast,network,gateway,dnsservers,use_dhcp);
        file=r"/data/etc/wpa_supplicant/wpa_supplicant.conf";
        cmd=None
        result = None
        
        if ( config==None ):
            #必須先決定是用Ubuntu 14.04或Ubuntu 16.04; wpa_supplicant或nmcli，這裡回傳的東西會不一樣，會需要重parse
            cmd1=r"sed -i -e'1,/^ssid/     s/^ssid=.*/ssid=\"myssid\"/' "          + file;
            cmd2=r"sed -i -e'1,/^psk/      s/^psk=.*/psk=\"mypsk\"/' "             + file;
            cmd3=r"sed -i -e'1,/^priority/ s/^priority=.*/priority=mypriority/g' " + file;
            cmd6=r"sed -i -e'1,/^id_str/     s/^id_str=.*/id_str=\"myuse_dhcp\"/' "+ file;

            ssid = ssid.replace("/","\/")
            
            cmd1=cmd1.replace("myssid", ssid);
            cmd2=cmd2.replace("mypsk", pwd);
            cmd3=cmd3.replace("mypriority", str(priority));

            #print("connect_wifi_ap - cmd 1=", cmd1)
            #print("connect_wifi_ap - cmd 2=", cmd2)
            #print("connect_wifi_ap - cmd 3=", cmd3)

            if ( use_dhcp==True ):
                cmd6=cmd6.replace("myuse_dhcp", "_dhcp");
            else:
                cmd6=cmd6.replace("myuse_dhcp", "_static");    
            cmd=cmd1+";"+cmd2+";"+cmd3+";"+cmd6+";";
        else:
            cmd1=r"confS=$(grep -n 'network={' myfile | awk -F  ':' '{print $1; exit}')"
            cmd2=r"confE=$(grep -n '}' myfile | awk -F  ':' '{print $1; exit}')"
            cmd3=r'new=$(sed "$confS,$confE d" myfile)'
            cmd4=r"new2='myConfig'"
            cmd5=r'new3=$(echo "$new" | sed "$confS i$new2")'        
            cmd6=r'echo "$new3" > myfile'
            cmd7=r"sed -i -e'1,/^id_str/     s/^id_str=.*/id_str=\"myuse_dhcp\"/' "+ file;
            if ( use_dhcp==True ):
                cmd7=cmd7.replace("myuse_dhcp", "_dhcp");
            else:
                cmd7=cmd7.replace("myuse_dhcp", "_static");
                    
            cmd1=cmd1.replace("myfile", file)
            cmd2=cmd2.replace("myfile", file)
            cmd3=cmd3.replace("myfile", file)
            #cmd4=cmd4.replace("myConfig", config)
            cmd6=cmd6.replace("myfile", file)

            #cmd=cmd1+";"+cmd2+";"+cmd3+";"+cmd4+";"+cmd5+";"+cmd6+";"+cmd7+";";
            cmd=cmd1+";"+cmd2+";"+cmd3+";"+cmd7+";";

            if config is not None:

                print("config =")
                print(config)

                self.write_file("/data/etc/wpa_supplicant/wpa_supplicant.conf", config)
            
        
        try:
            
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);

            print("connect_wifi_ap - cmd=")
            print(cmd)

            stdin, stdout, stderr = ssh.exec_command(cmd); #config wpa_supplicant.conf
            stdout.channel.recv_exit_status() ; #block here
            time.sleep(1)


            wifiReconfigureCmd="sudo wifi_robot "+ interface + " reconfigure";
            stdin, stdout, stderr = ssh.exec_command(wifiReconfigureCmd);
            stdout.channel.recv_exit_status() ; #block here

            print("stdout=", stdout.read().decode("ascii"))


            ssh.close();
            

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)


        return result;

    #test OK
    #目前由於機器上有 reconnect 的功能
    #基本上就算設定 disconnect 後 1min 內他也會根據之前設定好的無線AP進行連線
    def disconnect_wifi_ap(self, interface='wlan0' ):
        """

        Command AMR to disconnect from WIFI AP

        @param interface: Network interface, AMR only has 'wlan0' as default (String)
        @return:
                None                -> If timeout or disconnect (None)

                "..."               -> Result text (String)
        """
        #必須先決定是用Ubuntu 14.04或Ubuntu 16.04; wpa_supplicant或nmcli，這裡回傳的東西會不一樣，會需要重parse
        result = None
        
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="sudo wifi_robot "+ interface +" disable_network";
            stdin, stdout, stderr = ssh.exec_command(cmd); #wifi disconnect
            stdout.channel.recv_exit_status() ; #block here
            result = stdout.read();
            result.decode();
            ssh.close();
            
        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

        return result;


    def get_ble_mac(self):
        """

        Get AMR's bluetooth MAC address

        @return:
                '4C:1D:96:A5:F6:3A' -> Bluetooth MAC address, will be an empty string if timeout or disconnect (String)
        """
        ble_mac = ""

        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip,
                        port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False,
                        look_for_keys=False);
            cmd = "hciconfig";
            stdin, stdout, stderr = ssh.exec_command(cmd);  # find all folder only
            result = stdout.read();
            ssh.close();

            hciconfig_txt = result.decode('ascii')

            split_line_hciconfig_txt = hciconfig_txt.split("\n")

            interfaces = {}
            pre_interface_idx = 0
            interface_idx = 0
            ble_mac = ""
            interface_now_name = ""
            for line in split_line_hciconfig_txt:
                if (":" in line) and ("Type" in line) and ("Bus" in line):
                    interface_now_name = line.split(":")[0]
                    #print("interface_now_name=", interface_now_name)

                if interface_now_name == "hci0":
                    if "BD Address" in line:
                        space_split = line.split(" ")
                        ether_str_idx = None
                        for i, s in enumerate(space_split):
                            #print("s=", s)
                            if s == "Address:":
                                ether_str_idx = i

                        #print("============")

                        if ether_str_idx != None:
                            ble_mac = space_split[ether_str_idx + 1]

        except Exception as e:
            print("Exception ", Exception);


        return ble_mac;


    def set_uv_enable(self, sw=0, mode=4):
        """

        Easy Command AMR to turn on/off UV light and cover

        @param sw: Turn on/off UV light and cover with different mode (Int)
                   Close light and cover -> 0
                   Auto (Cover closed when people nearby automatically) -> 1
                   Hide (Cover always closed) -> 2
                   Exposed (Cover always opened) -> 3

        @param mode: NO USE VARIABLE (Int)

        """
        isOpen = False
        if sw==1: # Auto
            mode = 4
            isOpen = True
        elif sw==2: # UV Hide
            mode = 0
            isOpen = True
        elif sw==3: # UV Exposed
            mode = 2
            isOpen = True

        self.set_uvc(isOpen, mode)

    def set_uvc(self, enabled, mode=0, sensitivity=1.0, range_d=4.0):
        """

        Complicate Command AMR to turn on/off UV light and cover

        @param enabled: Turn on/off UV light and cover (Int)
                        0 -> Off
                        1 -> On

        @param mode: Mode of cover open and close (Int)
                     0 -> Cover close
                     2 -> Cover open
                     4 -> Cover open and close by nearby people

        @param sensitivity: (Float)
        @param range_d: People detected range, unit: m (Float)
        @return:
                None     -> If timeout or disconnect (None)
                "OK"     -> Success request (String)
                "NO_OK"  -> Fail request (String)
        """
        try:
            url = self.__url+"/set_uvc"
            payload = {
                        "enable": enabled,
                        "mode": mode,
                        "sensitivity": sensitivity,
                        "range": range_d
                      }
            r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
            if r.status_code!=200:
                print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
            exec(self.check_no_ok)

            return r.text

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)

            return None

    def sync_ntp(self, server_ip_addr):
        """

        Command AMR to sync NTP server by given server IP

        @param server_ip_addr: IP address (String)
        @return:
                None      -> If timeout or disconnect (None)
                JSON data ->
                {
                    'result': '23 Sep 16:38:06 ntpdate[27744]: adjust time server 118.163.81.61 offset 0.056570 sec\n'
                }

                result -> result text (String)
        """
        try:
            url = self.__url+"/sync_ntp"
            payload = {
                        "ip": server_ip_addr
                      }
            r = self.__session.post(url, data=json.dumps(payload), timeout=20.0)
            if r.status_code!=200:
                print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
            exec(self.check_no_ok)
            data = r.json()
            return data

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)

            return None

    def ping(self):
        """

        Send PING to this AMR

        @return:
                 True -> Target response
                 False -> Target no response
        """
        return self.__pinger.sendPing(self.__ip);
        

    #透過 NTP 跟 NTP service 同步時間
    # NTP service 建議就設在跟派車同一台電腦上
    #windows 10 設定方法:
    #http://172.16.49.26:8080/redmine/projects/qisda/wiki/Windows_10_%E5%AE%89%E8%A3%9DNTP_Server
    #回傳的資料會如下:
    #27 Feb 10:43:28 ntpdate[1902]: adjust time server 172.16.114.220 offset 0.098016 sec
    #test OK
    def sync_time(self, serverIP="192.168.1.1" ):
        """

        Command AMR to sync NTP server by given server IP

        @param serverIP: IP address (String)
        @return:
                None      -> If timeout or disconnect (None)

                '23 Sep 16:38:06 ntpdate[27744]: adjust time server 118.163.81.61 offset 0.056570 sec'
                          -> Result text (String)

        """
        result = None
        err_result = None
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="sudo ntpdate " + serverIP;
            stdin, stdout, stderr = ssh.exec_command(cmd); #wifi disconnect
            stdout.channel.recv_exit_status() ; #block here

            result = stdout.read();
            result_txt = result.decode();
            print(f"sync_time - result =")
            print(result_txt)

            err_result = stderr.read();
            err_result_txt = err_result.decode();
            print(f"sync_time - err_result =")
            print(err_result_txt)

            ssh.close();
            
        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

        return result, err_result;

    def sync_time_nohup(self, serverIP="192.168.1.1"):
        """

        Command AMR to sync NTP server by given server IP

        @param serverIP: IP address (String)
        @return:
                None      -> If timeout or disconnect (None)

                '23 Sep 16:38:06 ntpdate[27744]: adjust time server 118.163.81.61 offset 0.056570 sec'
                          -> Result text (String)

        """
        result = None
        err_result = None
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False,
                        look_for_keys=False);
            cmd = "sudo nohup ntpdate " + serverIP + " &";
            stdin, stdout, stderr = ssh.exec_command(cmd);  # wifi disconnect
            stdout.channel.recv_exit_status();  # block here

            result = stdout.read();
            result_txt = result.decode();
            print(f"sync_time_nohup - result =")
            print(result_txt)

            err_result = stderr.read();
            err_result_txt = err_result.decode();
            print(f"sync_time_nohup - err_result =")
            print(err_result_txt)

            ssh.close();

        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            # exec(self.try_print_exception_str)
            # return None

        return result, err_result;


    #如果正常寫入會返回 "OK"
    #如果資料過大有的沒的 無法寫入會返回 "NO_OK"
    #寫入資料型態不限 但寫入 binary 讀回也會是 binary 寫入字串讀回也會是字串
    #test OK
    def set_graffiti(self, inData):
        """
        ABANDONED, DO NOT USE
        """
        try:
            url = self.__url+"/set_graffiti"
            r = self.__session.post(url, data=inData, timeout=REQUEST_CONNECT_TIMEOUT )
            exec(self.check_no_ok)
            return r.content
        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None

    #return 的資料跟當初寫入資料會完全一致
    def get_graffiti(self):
        """
        ABANDONED, DO NOT USE
        """
        try:
            url = self.__url+"/get_graffiti"
            r = self.__session.post(url, timeout=REQUEST_CONNECT_TIMEOUT)
            exec(self.check_no_ok)
            return r.content
        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None

    def set_default_map(self, map_name):
        """

        Set default map folder to configuration file
        (AMR will load the default map automatically after booting)

        @param map_name: Map folder's name (String)

        """
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if infoSplitColon[0] == "default_map":
                    to_replace_str = i;

            wrote = "default_map:" + map_name
            info = info.replace(to_replace_str, "replace_default_map")
            info = info.replace("replace_default_map", wrote)

            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None


    def get_default_map(self):
        """

        Get default map folder from configuration file

        @return:
                 None      -> If timeout or disconnect (None)

                 "msi3F"   -> Map folder's name, empty string if default map is not set (String)
        """
        map_name = None;

        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport(trans);

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');
            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if infoSplitColon[0] == "default_map":
                    if (infoSplitColon[1]!="<none>"):
                        map_name = infoSplitColon[1];

            sftp.close();
            trans.close()

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            # exec(self.try_print_exception_str)
            # return None

        return map_name;

    def set_default_pos(self, x, y, a):
        """

        Set default position to configuration file
        (AMR relocate the default position after booting)

        @param x: World coordinate X, unit: m (Float)
        @param y: World coordinate Y unit: m (Float)
        @param a: World coordinate Angle, unit: radian (Float)
        """
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if infoSplitColon[0] == "default_pos":
                    to_replace_str = i;

            wrote = "default_pos:" + "%.3f"%x + "," + "%.3f"%y + "," + "%.3f"%a
            info = info.replace(to_replace_str, "replace_default_pos")
            info = info.replace("replace_default_pos", wrote)

            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

    def get_default_pos(self):
        """

        Get default position from configuration file

        @return:
                (-4.246, 5.041, -3.133) -> Group of positions (Tuple)
                                           (x, y, angle)
                                           x -> World coordinate X, will return None if disconnect or timeout, unit: m (Float)
                                           y -> World coordinate Y, will return None if disconnect or timeout, unit: m (Float)
                                           angle -> World coordinate Angle, will return None if disconnect or timeout, unit: radian (Float)
        """
        x = None;
        y = None;
        a = None;

        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport(trans);

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');
            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if infoSplitColon[0] == "default_pos":
                    xya = infoSplitColon[1].split(",");
                    if ( len(xya)==3 ):
                        x = float(xya[0])
                        y = float(xya[1])
                        a = float(xya[2])

            sftp.close();
            trans.close()

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            # exec(self.try_print_exception_str)
            # return None

        return x, y, a;


    def get_life(self, try_count_max=EXCEPT_MAX_RETRY_TIMES, to=REQUEST_CONNECT_TIMEOUT):
        """

        Get working time of system, motor, sensor...etc

        @param try_count_max: Max times to retry request (Int)
        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                None      -> If timeout or disconnect (None)
                JSON data

        """
        for trycount in range(try_count_max):
            try:
                url = self.__url + "/get_life"
                payload = {}

                r = self.__session_get_life.post(url, data=json.dumps(payload), timeout=to)
                if r.status_code != 200:
                    print("====== {", sys._getframe().f_code.co_name, "} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)

                #print(r)

                data=r.json()
                return data

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None


    def assign_client(self, ip):
        """

        Assign FMS host IP address to AMR

        @param ip: FMS host IP address (String)
        @return:
                None      -> If timeout or disconnect (None)
                JSON data
        """
        try:
            url = self.__url+"/assign_client"
            payload = {
                "ip": ip
            }
            r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT )
            exec(self.check_no_ok)
            data = r.json()
            return data
        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None

    def ask_host(self):
        """

        Ask FMS IP which manage the AMR

        @return:
                None      -> If timeout or disconnect (None)
                JSON data
        """
        try:
            url = self.__url+"/ask_host"
            payload = {}
            r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT )
            exec(self.check_no_ok)
            data = r.json()
            return data
        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None

    def remove_host(self):
        """

        Ask FMS IP which manage the AMR (Not really move AMR out of FMS's list)

        @return:
                None      -> If timeout or disconnect (None)
                JSON data
        """
        try:
            url = self.__url+"/remove_host"
            payload = {}
            r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT )
            exec(self.check_no_ok)
            data = r.json()
            return data
        # except requests.exceptions.RequestException as e:
        #     exec(self.try_print_exception_itself_str)
        #     return None
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)
            return None


    def get_fw_version(self):
        """

        Get firmware version of AMR

        @return:
                "1.8.123" -> firmware version, will be empty string if timeout or disconnect (String)
        """
        ver = "";
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                trans = paramiko.Transport((self.__ip, 22))
                trans.start_client();
                trans.auth_password(username=self.__sshName, password=self.__sshPwd);
                sftp = paramiko.SFTPClient.from_transport(trans);

                ver_file = sftp.open("/data/etc/fw_version", "r")
                ver = ver_file.read().decode('utf-8');

                sftp.close();
                trans.close()
                return ver;
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                #exec(self.try_print_exception_str)

        return None
        

    #LeonLi test OK@2021/04/12
    def list_timezones(self):
        """

        Show list of time zones that can be configured

        @return:
                ['Africa/Abidjan', 'Africa/Accra', 'Africa/Addis_Ababa'...] -> Group of time zones (List)

                                                                               [contry/area 1, contry/area 2...]
                                                                               contry/area -> (String)
        """
        result = None
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="sudo timedatectl --no-pager  list-timezones";
            stdin, stdout, stderr = ssh.exec_command(cmd); 
            stdout.channel.recv_exit_status() ; #block here
            result = stdout.read();
            result=result.decode("utf-8");
            result=result.split('\n')
            ssh.close();
            
        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

        return result;
    
    #LeonLi test OK@2021/04/12
    def set_timezone(self, timezone):
        """

        Set AMR's time zone by given valid country/area

        @param timezone: Country or area (String)
        @return:
                None      -> If timeout or disconnect (None)
        """
        result = None
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="sudo timedatectl set-timezone " + timezone;
            stdin, stdout, stderr = ssh.exec_command(cmd); 
            stdout.channel.recv_exit_status() ; #block here
            ssh.close();
            
        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

        return result;
    
    #LeonLi test OK@2021/04/12
    def get_timezone(self):
        """
        Get AMR current time zone

        @return:
                None          -> If timeout or disconnect (None)
                'Asia/Taipei' -> Country or area (String)
        """
        result = None
        try:
            ssh = paramiko.SSHClient();
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy());
            ssh.connect(hostname=self.__ip, port=22, username=self.__sshName, password=self.__sshPwd, allow_agent=False, look_for_keys=False);
            cmd="sudo timedatectl | grep zone:";
            stdin, stdout, stderr = ssh.exec_command(cmd); 
            #stdout.channel.recv_exit_status() ; #block here
            time.sleep(1.0)
            result = stdout.read();
            result=result.decode("utf-8");
            result=result.split(' ')
            ssh.close();
            
        # except Exception as e:
        #     print("Exception ", Exception);
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            #exec(self.try_print_exception_str)
            #return None

        if result is not None:
            return result[-3]
        else:
            return None
    
    
    def get_info(self, try_count_max=EXCEPT_MAX_RETRY_TIMES, to=REQUEST_CONNECT_TIMEOUT):
        """

        Get AMR information

        @param try_count_max: Max times to retry request (Int)
        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                None          -> If timeout or disconnect (None)
                JSON data
                                 {
                                    'info': {
                                              'mode': 'none',
                                              'fw_version': 'V1.8.214',
                                              'id': '2cf05d91658f',
                                              'robot': {
                                                         'max_linear': 0.9,
                                                         'max_angular': 1.571,
                                                         'robot_length': 0.85,
                                                         'robot_width': 0.85,
                                                         'robot_height': 1.9,
                                                         'center2front': 0.245
                                                       }
                                        }
                                 }

                                 'mode'         -> UV disinfection mode - auto, hide, exposed (String)
                                 'fw_version'   -> firmware version (String)
                                 'id'           -> AMR's id (String)
                                 'max_linear'   -> Max velovity, unit: m/s (Float)
                                 'max_angular'  -> Max rotation speed, unit: radian/s (Float)
                                 'robot_length' -> AMR's length, unit: m (Float)
                                 'robot_width'  -> AMR's width, unit: m (Float)
                                 'robot_height' -> AMR's height, unit: m (Float)
                                 'center2front' -> AMR's distance between center and front, unit: m (Float)



        """
        r=None
        for trycount in range(try_count_max):
            try:
                #機器的相關性能與外觀資訊
                url = self.__url+"/get_info"
                payload = {"mac": self.__myMac}
                r = self.__session_get_misc.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT )
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                data=r.json()
                return data
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def debug_SaveImage(self, name="Save AI Debug Image", file_name=None, t=-1):
        """

        Command AMR to save the debug image (image file will be saved to folder - "/data/dump")

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param file_name: Change the header of image file's name (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #移動到指定目標點
                #10 是參考的 VW indes range 10~29
                #30 是參考的 速限地圖 indes range 30~49
                #
                map_arr = [0, 0]

                data_str = ""
                if file_name is not None:
                    if type(file_name) is str:
                        data_str = file_name

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 302,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  {"x": float("%.2f"%39.00), "y": float("%.2f"%102.00), "a": float("%.2f"%0.00) } # This is the cheatcode
                                ],
                                "data": data_str
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def debug_SensorDisplayOption(self, name="Sensor Display Option", opt=None, t=-1):
        """

        Command AMR to show sensor data (will not stop AMR current moving job)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param opt: Type of sensor data (String)
                    "LASER"  -> Show laser data
                    "CAMERA" -> Show camera detected result
                    "SONAR"  -> Show sonar data, AMR has no sonar as default
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                map_arr = [0, 0]

                x = 218.0
                y = 0.0
                a = 0.0

                if opt=="LASER":
                    x = 218.0
                    y = 9.0
                    a = -1.0
                elif opt=="CAMERA":
                    x =  218.0
                    y =  8.0
                    a = -1.0
                elif opt=="SONAR":
                    x = 218.0
                    y = 6.0
                    a = 0.0

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 302,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  {"x": float("%.2f"%x), "y": float("%.2f"%y), "a": float("%.2f"%a) }
                                ],
                                "data": ""
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def debug_SensorDisplayOption_Detail(self, name="Sensor Display Option Detail", opt=None, idx=-1.0, t=-1):
        """

        Command AMR to show sensor data (will not stop AMR current moving job)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param opt: Type of sensor data (String)
                    "LASER"  -> Show laser data
                    "CAMERA" -> Show camera detected result
                    "SONAR"  -> Show sonar data, AMR has no sonar as default
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                map_arr = [0, 0]

                x = 218.0
                y = 0.0
                a = 0.0

                if opt=="LASER":
                    x = 218.0
                    y = 9.0
                    a = idx
                elif opt=="CAMERA":
                    x =  218.0
                    y =  8.0
                    a =  idx
                elif opt=="SONAR":
                    x = 218.0
                    y = 6.0
                    a = 0.0

                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 302,
                                "map"   : map_arr, #移動的過程需要參考的圖層 也可不參考
                                "target"  : [
                                  {"x": float("%.2f"%x), "y": float("%.2f"%y), "a": float("%.2f"%a) }
                                ],
                                "data": ""
                            },
                         "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = self.__session.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text

            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None


    def set_size(self, name="Set Size", wx=None, wy=None, hz=None, bhz=None, center=None, t=-1):
        """

        Set AMR's size for avoiding obstacle

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param wx: Length of AMR (Float)
        @param wy: Width of AMR (Float)
        @param hz: Height of AMR (Float)
        @param bhz: Ignored bottom belowed of AMR (Float)
        @param center: Distance from center to front of AMR (Float)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if (wx is not None) or (wy is not None) or (hz is not None):

            for trycount in range(EXCEPT_MAX_RETRY_TIMES):

                name = self.AI_name_UUID_postfix(name)
                self.given_ai_name = name

                try:
                    size_data_str = ""
                    if (wx is not None):
                        if (type(wx) is float):
                            wx = "%.3f"%wx
                            size_data_str += "@rs_RL:{wx}#;".format(wx=wx)

                    if (wy is not None):
                        if (type(wy) is float):
                            wy = "%.3f"%wy
                            size_data_str += "@rs_RW:{wy}#;".format(wy=wy)

                    if (hz is not None):
                        if (type(hz) is float):
                            hz = "%.3f"%hz
                            size_data_str += "@rs_RH:{hz}#;".format(hz=hz)

                    if (bhz is not None):
                        if (type(bhz) is float):
                            bhz = "%.3f"%bhz
                            size_data_str += "@rs_SH:{hz}#;".format(hz=bhz)

                    if (center is not None):
                        if (type(center) is float):
                            center = "%.3f"%center
                            size_data_str += "@rs_CF:{hz}#;".format(hz=center)

                    print("size_data_str=", size_data_str)


                    #start closing map record
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : 221,
                                    "data"  : size_data_str
                                    },
                                    "mac": self.__myMac}

                    if t > -1:
                        payload['set_job']['time'] = int(t)

                    r = requests.post(url, data=json.dumps(payload))
                    if r.status_code!=200:
                        print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                        continue
                    exec(self.check_no_ok)
                    return r.text
                # except requests.exceptions.RequestException as e:
                #     exec(self.try_print_exception_itself_str)
                #     return None
                except Exception as e:
                    print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                    exec(self.try_print_exception_itself_str)
                    exec(self.try_print_exception_str)
            return None

    def set_default_size(self, name="Set Default Size", t=-1):
        """

        Set AMR's size for avoiding obstacle to AI configuration

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        for trycount in range(EXCEPT_MAX_RETRY_TIMES):

            name = self.AI_name_UUID_postfix(name)
            self.given_ai_name = name

            try:
                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 221,
                                "data"  : "@rs_FN:algorithm.cfg#"
                                },
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def output_dock_plug_offset(self, name, t=-1):
        """

        Command AMR to get dc_plug_offset in get_misc() function

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #print("size_data_str=", size_data_str)

                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 222,
                                "data"  : "@dc_PO:x#"
                                },
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def set_sensor_display_config(self, name="Set Sensor Display Config", mode=None, s_type=None, t=-1):
        """

        Command AMR to show sensor data (will stop AMR current moving job)

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param mode: Sensor Display mode (Int)
                     0 -> Diasble
                     1 -> Enable when robot is walking
                     2 -> Enable always

        @param s_type: Sensor type (Int)
                       0 -> Show rough sensor data
                       1 -> Show complete all sensor data
                       2 -> Show laser sensor data
                       3 -> Show camera sensor data
                       4 -> Show QR code sensor data
                       5 -> Show ultra sound sensor data

        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if (mode is not None) or (s_type is not None):
            for trycount in range(EXCEPT_MAX_RETRY_TIMES):

                name = self.AI_name_UUID_postfix(name)
                self.given_ai_name = name

                try:
                    size_data_str = ""
                    if (mode is not None):
                        if (type(mode) is int):
                            # Sensor Display mode
                            # 0：Disable
                            # 1：Enable when robot is walking
                            # 2：Enable always
                            size_data_str += "@rs_SM:{m}#;".format(m=mode)

                    if (s_type is not None):
                        if (type(s_type) is int):
                            # Sensor type
                            # 0：Show rough sensor data
                            # 1：Show complete all sensor data
                            # 2：Show laser sensor data
                            # 3：Show camera sensor data
                            # 4：Show QR code sensor data
                            # 5：Show ultra sound sensor data
                            size_data_str += "@rs_ST:{st}#;".format(st=s_type)


                    #print("size_data_str=", size_data_str)


                    #start closing map record
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : 221,
                                    "data"  : size_data_str
                                    },
                                    "mac": self.__myMac}

                    if t > -1:
                        payload['set_job']['time'] = int(t)

                    r = requests.post(url, data=json.dumps(payload))
                    if r.status_code!=200:
                        print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                        continue
                    exec(self.check_no_ok)
                    return r.text
                # except requests.exceptions.RequestException as e:
                #     exec(self.try_print_exception_itself_str)
                #     return None
                except Exception as e:
                    print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                    exec(self.try_print_exception_itself_str)
                    exec(self.try_print_exception_str)
            return None

    def output_laser_ignore_area(self, name, t=-1):
        """

        Command AMR to show laser ignore area in get_misc() function

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """

        name = self.AI_name_UUID_postfix(name)
        self.given_ai_name = name

        for trycount in range(EXCEPT_MAX_RETRY_TIMES):
            try:
                #print("size_data_str=", size_data_str)

                #start closing map record
                url = self.__url+"/set_job"
                payload = { "set_job" : {
                                "name"  : name[0:MAX_NAME_LENGTH],
                                "cmd"   : 222,
                                "data"  : "@laser0_ADD:#@laser1_ADD:#"
                                },
                                "mac": self.__myMac}

                if t > -1:
                    payload['set_job']['time'] = int(t)

                r = requests.post(url, data=json.dumps(payload))
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                return r.text
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None

    def set_info_laser_ignore_area(self, idx=None, min_rad=None, max_rad=None):
        """

        Set laser ignore area to config file
        (AMR will write the laser ignore area from config file to AI automatically after booting)

        @param idx: Laser device index (Int)
        @param min_rad: Minimum radian, unit: radian (Float)
        @param max_rad: Maximum radian, unit: radian (Float)
        """
        sftp = None;
        trans = None;
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            to_replace_str_exist = 0
            to_replace_str = ""
            content_str = ""
            info_idx = -1

            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");

                if("laser_" in infoSplitColon[0]) and ("_ignore_area" in infoSplitColon[0]):
                    info_idx = infoSplitColon[0].split("_")[1]
                    info_idx = int(info_idx)

                    if info_idx == idx:
                        to_replace_str_exist = 1
                        to_replace_str = i
                        content_str = infoSplitColon[1]
                        break



            if (max_rad is not None) and (min_rad is not None):
                isValidContent = False
                item_arr = []
                if (content_str != ""):
                    try:
                        lia_json_str = "{" + "\"arr\":" + content_str + "}"
                        print("lia_json_str=")
                        print(lia_json_str)

                        lia_json = json.loads(lia_json_str)

                        item_arr = lia_json['arr']
                        print("item_arr=", item_arr)

                        isValidContent = True

                    except:
                        pass


                    print("set_info_laser_ignore_area - replace valid one")
                    if to_replace_str_exist == 1:
                        isFoundSame = False

                        if isValidContent == True:
                            for item in item_arr:
                                tmp_item_0 = float("%.3f" % item[0])
                                tmp_item_1 = float("%.3f" % item[1])
                                min_rad = float("%.3f" % min_rad)
                                max_rad = float("%.3f" % max_rad)

                                if (tmp_item_0==min_rad) and (tmp_item_1==max_rad):
                                    isFoundSame = True
                                    break

                        if isFoundSame == False:
                            min_rad = float("%.3f" % min_rad)
                            max_rad = float("%.3f" % max_rad)
                            item_arr.append([min_rad, max_rad])

                            wrote_info = "laser_{i}_ignore_area:".format(i=info_idx) + str(item_arr)

                            #print("wrote_info=", wrote_info)
                            #print("to_replace_str=", to_replace_str)
                            info = info.replace(to_replace_str, "replace_name")
                            info = info.replace("replace_name", wrote_info)

                else:
                    new_item_arr = []
                    min_rad = float("%.3f" % min_rad)
                    max_rad = float("%.3f" % max_rad)
                    new_item_arr.append([min_rad, max_rad])
                    wrote_info = "laser_{i}_ignore_area:".format(i=idx) + str(new_item_arr)
                    info += "\n" + wrote_info + "\n"


            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except:
            if sftp is not None:
                sftp.close()

            if trans is not None:
                trans.close()


    def clear_info_laser_ignore_area(self, idx=None):
        """

        Clear laser ignore area from config file

        @param idx: Laser device index (Int)
        """
        sftp = None;
        trans = None;
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            to_replace_str_exist = 0
            to_replace_str = ""
            content_str = ""
            info_idx = -1

            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");

                if("laser_" in infoSplitColon[0]) and ("_ignore_area" in infoSplitColon[0]):
                    info_idx = infoSplitColon[0].split("_")[1]
                    info_idx = int(info_idx)

                    if info_idx == idx:
                        to_replace_str_exist = 1
                        to_replace_str = i
                        content_str = infoSplitColon[1]
                        break

            if content_str != "":
                wrote_info = "laser_{i}_ignore_area:".format(i=info_idx) + "<none>"

                #print("wrote_info=", wrote_info)
                #print("to_replace_str=", to_replace_str)
                info = info.replace(to_replace_str, "replace_name")
                info = info.replace("replace_name", wrote_info)




            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except:
            if sftp is not None:
                sftp.close()

            if trans is not None:
                trans.close()


    def set_info_camera_ignore_area(self, idx=None, min=None, max=None, direction="LR"):
        """

        Set camera ignore area to config file
        (AMR will write the camera ignore area from config file to AI automatically after booting)

        @param idx: Camera device index (Int)
        @param min_rad: Minimum value, unit: radian or meter (Float)
        @param max_rad: Maximum value, unit: radian or meter (Float)
        """
        sftp = None;
        trans = None;
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            to_replace_str_exist = 0
            to_replace_str = ""
            content_str = ""
            info_idx = -1

            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");

                if("camera_" in infoSplitColon[0]) and ("_ignore_area" in infoSplitColon[0]):
                    info_idx = infoSplitColon[0].split("_")[1]
                    info_direction = infoSplitColon[0].split("_")[-1]
                    info_idx = int(info_idx)

                    if (info_idx == idx) and (info_direction == direction):
                        to_replace_str_exist = 1
                        to_replace_str = i
                        content_str = infoSplitColon[1]
                        break



            if (max is not None) and (min is not None):
                isValidContent = False
                item_arr = []
                if (content_str != ""):
                    try:
                        lia_json_str = "{" + "\"arr\":" + content_str + "}"
                        print("lia_json_str=")
                        print(lia_json_str)

                        lia_json = json.loads(lia_json_str)

                        item_arr = lia_json['arr']
                        print("item_arr=", item_arr)

                        isValidContent = True

                    except:
                        pass


                    print("set_camera_laser_ignore_area - replace valid one")
                    if to_replace_str_exist == 1:
                        isFoundSame = False

                        if isValidContent == True:
                            for item in item_arr:
                                tmp_item_0 = float("%.3f" % item[0])
                                tmp_item_1 = float("%.3f" % item[1])
                                min = float("%.3f" % min)
                                max = float("%.3f" % max)

                                if (tmp_item_0==min) and (tmp_item_1==max):
                                    isFoundSame = True
                                    break

                        if isFoundSame == False:
                            min_rad = float("%.3f" % min)
                            max_rad = float("%.3f" % max)
                            item_arr.append([min, max])

                            wrote_info = "camera_{i}_ignore_area_{dir}:".format(i=info_idx, dir=direction) + str(item_arr)

                            #print("wrote_info=", wrote_info)
                            #print("to_replace_str=", to_replace_str)
                            info = info.replace(to_replace_str, "replace_name")
                            info = info.replace("replace_name", wrote_info)

                else:
                    new_item_arr = []
                    min = float("%.3f" % min)
                    max = float("%.3f" % max)
                    new_item_arr.append([min, max])
                    wrote_info = "camera_{i}_ignore_area_{dir}:".format(i=idx, dir=direction) + str(new_item_arr)
                    info += "\n" + wrote_info + "\n"


            print("set_info_camera_ignore_area - info to wrote:")
            print(info)

            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except:
            if sftp is not None:
                sftp.close()

            if trans is not None:
                trans.close()


    def clear_info_camera_ignore_area(self, idx=None, direction="All"):
        """

        Clear camera ignore area from config file

        @param idx: Laser device index (Int)
        """
        sftp = None;
        trans = None;
        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport( trans );

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');

            to_replace_str_exist = 0
            to_replace_str = ""
            content_str = ""
            info_idx = -1

            infoSplitEnter = info.split("\n");

            if direction != "All":
                for i in infoSplitEnter:
                    infoSplitColon = i.split(":");

                    if("camera_" in infoSplitColon[0]) and ("_ignore_area" in infoSplitColon[0]):
                        info_idx = infoSplitColon[0].split("_")[1]
                        info_direction = infoSplitColon[0].split("_")[-1]
                        info_idx = int(info_idx)

                        if (info_idx == idx) and (info_direction == direction):
                            to_replace_str_exist = 1
                            to_replace_str = i
                            content_str = infoSplitColon[1]
                            break

                if content_str != "":
                    wrote_info = "camera_{i}_ignore_area_{dir}:".format(i=info_idx, dir=direction) + "<none>"

                    #print("wrote_info=", wrote_info)
                    #print("to_replace_str=", to_replace_str)
                    info = info.replace(to_replace_str, "replace_name")
                    info = info.replace("replace_name", wrote_info)

            else:
                for i in infoSplitEnter:
                    infoSplitColon = i.split(":");

                    content_str = ""

                    if("camera_" in infoSplitColon[0]) and ("_ignore_area" in infoSplitColon[0]):
                        info_idx = infoSplitColon[0].split("_")[1]
                        info_direction = infoSplitColon[0].split("_")[-1]
                        info_idx = int(info_idx)

                        to_replace_str_exist = 1
                        to_replace_str = i
                        content_str = infoSplitColon[1]


                    if content_str != "":
                        wrote_info = "camera_{i}_ignore_area_{dir}:".format(i=info_idx, dir=info_direction) + "<none>"

                        #print("wrote_info=", wrote_info)
                        #print("to_replace_str=", to_replace_str)
                        info = info.replace(to_replace_str, "replace_name")
                        info = info.replace("replace_name", wrote_info)




            sftp.open("info.cfg", "w").write(info);

            sftp.close();
            trans.close();

        except:
            if sftp is not None:
                sftp.close()

            if trans is not None:
                trans.close()


    def get_info_laser_ignore_area(self):
        """

        Get laser ignore area from config file
        (AMR will write the laser ignore area from config file to AI automatically after booting)

        @return:
                {} -> Dictionary of laser ignore area, will be empty dictionary if timeout or disconnect (Dictionary)
        """
        laser_ignore_dict = {}

        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport(trans);

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');
            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if("laser_" in infoSplitColon[0]) and ("_ignore_area" in infoSplitColon[0]):
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            lia_json_str = "{" + "\"arr\":" + infoSplitColon[1] + "}"
                            #print("lia_json_str=")
                            #print(lia_json_str)

                            lia_json = json.loads(lia_json_str)

                            item_arr = lia_json['arr']
                            #print("item_arr=", item_arr)


                            idx = infoSplitColon[0].split("_")[1]

                            laser_ignore_dict[idx] = item_arr

                        except:
                             pass

            sftp.close();
            trans.close()

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            # exec(self.try_print_exception_str)
            # return None

        return laser_ignore_dict


    def get_info_camera_ignore_area(self):
        """

        Get camera ignore area from config file
        (AMR will write the camera ignore area from config file to AI automatically after booting)

        @return:
                {} -> Dictionary of camera ignore area, will be empty dictionary if timeout or disconnect (Dictionary)
        """
        laser_ignore_dict = {}

        try:
            trans = paramiko.Transport((self.__ip, 22))
            trans.start_client();
            trans.auth_password(username=self.__sshName, password=self.__sshPwd);
            sftp = paramiko.SFTPClient.from_transport(trans);

            info_file = sftp.open("info.cfg", "r")
            info = info_file.read().decode('utf-8');
            infoSplitEnter = info.split("\n");
            for i in infoSplitEnter:
                infoSplitColon = i.split(":");
                if("camera_" in infoSplitColon[0]) and ("_ignore_area" in infoSplitColon[0]):
                    if (infoSplitColon[1]!="<none>"):
                        try:
                            lia_json_str = "{" + "\"arr\":" + infoSplitColon[1] + "}"
                            #print("lia_json_str=")
                            #print(lia_json_str)

                            lia_json = json.loads(lia_json_str)

                            item_arr = lia_json['arr']
                            #print("item_arr=", item_arr)


                            idx = infoSplitColon[0].split("_")[1]
                            direction = infoSplitColon[0].split("_")[-1]

                            laser_ignore_dict[idx+"_"+direction] = item_arr

                        except:
                             pass

            sftp.close();
            trans.close()

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            # exec(self.try_print_exception_str)
            # return None

        return laser_ignore_dict



    def add_laser_ignore_area(self, name, idx=None, min_rad=None, max_rad=None, t=-1):
        """

        Add laser ignore area to AI

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param idx: Laser device index (Int)
        @param min_rad: Minimum radian, unit: radian (Float)
        @param max_rad: Maximum radian, unit: radian (Float)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if ( type(idx) is int) and (type(max_rad) is float) and (type(min_rad) is float):

            name = self.AI_name_UUID_postfix(name)
            self.given_ai_name = name

            for trycount in range(EXCEPT_MAX_RETRY_TIMES):
                try:
                    #print("size_data_str=", size_data_str)
                    data_str = ""

                    data_str += "@laser{i}_ADD:{r1},{r2}#".format(i=idx, r1="%.3f"%min_rad, r2="%.3f"%max_rad)

                    #start closing map record
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : 221,
                                    "data"  : data_str
                                    },
                                    "mac": self.__myMac}

                    if t > -1:
                        payload['set_job']['time'] = int(t)

                    r = requests.post(url, data=json.dumps(payload))
                    if r.status_code!=200:
                        print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                        continue
                    exec(self.check_no_ok)
                    return r.text
                # except requests.exceptions.RequestException as e:
                #     exec(self.try_print_exception_itself_str)
                #     return None
                except Exception as e:
                    print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                    exec(self.try_print_exception_itself_str)
                    exec(self.try_print_exception_str)
            return None

    def configure_from_string(self, name, the_data_str="", t=-1):
        """

        Add laser ignore area to AI

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param idx: Laser device index (Int)
        @param min_rad: Minimum radian, unit: radian (Float)
        @param max_rad: Maximum radian, unit: radian (Float)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if ( type(the_data_str) is str):
            for trycount in range(EXCEPT_MAX_RETRY_TIMES):

                name = self.AI_name_UUID_postfix(name)
                self.given_ai_name = name

                try:
                    #print("size_data_str=", size_data_str)
                    data_str = the_data_str

                    #start closing map record
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : 221,
                                    "data"  : data_str
                                    },
                                    "mac": self.__myMac}

                    if t > -1:
                        payload['set_job']['time'] = int(t)

                    r = requests.post(url, data=json.dumps(payload))
                    if r.status_code!=200:
                        print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                        continue
                    exec(self.check_no_ok)
                    return r.text
                # except requests.exceptions.RequestException as e:
                #     exec(self.try_print_exception_itself_str)
                #     return None
                except Exception as e:
                    print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                    exec(self.try_print_exception_itself_str)
                    exec(self.try_print_exception_str)
            return None


    def clear_laser_ignore_area(self, name, idx_arr=[], t=-1):
        """

        Clear laser ignore area from AI

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param idx_arr: List of laser device index (List of Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if (idx_arr is not None) and (type(idx_arr) is list):
            if len(idx_arr) > 0:

                name = self.AI_name_UUID_postfix(name)
                self.given_ai_name = name

                for trycount in range(EXCEPT_MAX_RETRY_TIMES):
                    try:
                        #print("size_data_str=", size_data_str)
                        data_str = ""
                        for idx in idx_arr:
                            data_str += "@laser{i}_DEL#".format(i=idx)

                        #start closing map record
                        url = self.__url+"/set_job"
                        payload = { "set_job" : {
                                        "name"  : name[0:MAX_NAME_LENGTH],
                                        "cmd"   : 221,
                                        "data"  : data_str
                                        },
                                        "mac": self.__myMac}

                        if t > -1:
                            payload['set_job']['time'] = int(t)

                        r = requests.post(url, data=json.dumps(payload))
                        if r.status_code!=200:
                            print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                            continue
                        exec(self.check_no_ok)
                        return r.text
                    # except requests.exceptions.RequestException as e:
                    #     exec(self.try_print_exception_itself_str)
                    #     return None
                    except Exception as e:
                        print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                        exec(self.try_print_exception_itself_str)
                        exec(self.try_print_exception_str)
                return None

    def add_camera_ignore_area(self, name, idx=None, min_rad=None, max_rad=None, direction="LR", t=-1):
        """

        Add camera ignore area to AI

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param idx: Camera device index (Int)
        @param min_rad: Minimum radian, unit: radian (Float)
        @param max_rad: Maximum radian, unit: radian (Float)
        @param direction: Ignored direction (String)
                          "LR" -> Left to right
                          "UB" -> Up to Bottom
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if ( type(idx) is int) and (type(max_rad) is float) and (type(min_rad) is float):

            name = self.AI_name_UUID_postfix(name)
            self.given_ai_name = name

            for trycount in range(EXCEPT_MAX_RETRY_TIMES):
                try:
                    #print("size_data_str=", size_data_str)
                    data_str = ""

                    if direction == "LR":
                        data_str += "@camera{i}_ADD:{r1},{r2}#".format(i=idx, r1="%.3f"%min_rad, r2="%.3f"%max_rad)

                    if direction == "UB":
                        data_str += "@camera{i}_LUB:{r1},{r2}#".format(i=idx, r1="%.3f"%min_rad, r2="%.3f"%max_rad)

                    #start closing map record
                    url = self.__url+"/set_job"
                    payload = { "set_job" : {
                                    "name"  : name[0:MAX_NAME_LENGTH],
                                    "cmd"   : 221,
                                    "data"  : data_str
                                    },
                                    "mac": self.__myMac}

                    if t > -1:
                        payload['set_job']['time'] = int(t)

                    r = requests.post(url, data=json.dumps(payload))
                    if r.status_code!=200:
                        print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                        continue
                    exec(self.check_no_ok)
                    return r.text
                # except requests.exceptions.RequestException as e:
                #     exec(self.try_print_exception_itself_str)
                #     return None
                except Exception as e:
                    print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                    exec(self.try_print_exception_itself_str)
                    exec(self.try_print_exception_str)
            return None


    def clear_camera_ignore_area(self, name, idx_arr=[], t=-1):
        """

        Clear camera ignore area from AI

        @param name: Command name, it is always useful to check whether command is executed (String)
        @param idx_arr: List of camera device index (List of Int)
        @param t: Give a UTC time, if AMR's utc time is bigger than this,
                  the command will not be excuted (Float)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        if (idx_arr is not None) and (type(idx_arr) is list):
            if len(idx_arr) > 0:

                name = self.AI_name_UUID_postfix(name)
                self.given_ai_name = name

                for trycount in range(EXCEPT_MAX_RETRY_TIMES):
                    try:
                        #print("size_data_str=", size_data_str)
                        data_str = ""
                        for idx in idx_arr:
                            data_str += "@camera{i}_DEL#".format(i=idx)

                        #start closing map record
                        url = self.__url+"/set_job"
                        payload = { "set_job" : {
                                        "name"  : name[0:MAX_NAME_LENGTH],
                                        "cmd"   : 221,
                                        "data"  : data_str
                                        },
                                        "mac": self.__myMac}

                        if t > -1:
                            payload['set_job']['time'] = int(t)

                        r = requests.post(url, data=json.dumps(payload))
                        if r.status_code!=200:
                            print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                            continue
                        exec(self.check_no_ok)
                        return r.text
                    # except requests.exceptions.RequestException as e:
                    #     exec(self.try_print_exception_itself_str)
                    #     return None
                    except Exception as e:
                        print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                        exec(self.try_print_exception_itself_str)
                        exec(self.try_print_exception_str)
                return None


    def read_file(self, name,cmd=""):
        """

        Read file content from AMR by given file path

        @param name: File path (String)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        for trycount in range(1):
            try:
 
                url = self.__url+"/read_file"
                payload = { "read_file": {
                                "name" : name,
                                "cmd": cmd
                                },
                        "mac": self.__myMac}
                        
                #t1 = time.time();
                #print("Device r pre:",t1)
                r = self.__session_file.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue

                #exec(self.check_no_ok)

                if len(r.content)==0:
                    return None
                else:

                    if DEBUG_READ_FILE == 1:
                        now_time_str = time.strftime("%H:%M:%S,%Y-%m-%d", time.localtime(time.time()))
                        print(f"[{now_time_str}]device - read_file, ip={self.__ip}, name={name}, cmd={cmd}, r.text={r.text}")

                    if "NO_OK" in r.text:
                        return None
                    
                return r.content


            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None



    def write_file(self, name, data=b''):
        """

        Write file content to AMR by given file path

        @param name: File path (String)
        @param data: Written data (String or Bytes)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        for trycount in range(1):
            try:
                url = self.__url+"/write_file"
                
                dataBytes=None
                if type(data) is str:
                    dataBytes=data.encode('utf-8')
                elif type(data) is bytes:
                    dataBytes=data
                
                #print("map size", len(imgCompress))
                data64 = b64encode(dataBytes) #受限 JSON 不支持 binary 傳輸資料只 能先轉成 字串形式來做傳輸
                data64Str = data64.decode(encoding='utf-8')
                payload = { "write_file": {
                                 "name" : name,
                                 "data" : data64Str,
                                },
                        "mac": self.__myMac}
                        
                #t1 = time.time();
                #print("Device r pre:",t1)
                r = self.__session_file.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue

                #exec(self.check_no_ok)

                if len(r.content)==0:
                    return None
                if len(r.content)==5 :
                    if r.text=="NO_OK":
                        return None
                    
                return r.content

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None

    def check_http_api_exist(self, api):
        """

        Check whether HTTP api is exist

        @param api: API (String)
        @return:
                Boolean ->
                           True: Exist
                           False: Not Exist
        """
        url = self.__url + "/" + api
        payload = {
            "read_file": {
            "name": "/home/pyuser/info.cfg",
            },
            "mac": self.__myMac
        }

        try:
            r = self.__session_file.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)

            if r.status_code == 404:
                return False
            else:
                return True

        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
            exec(self.try_print_exception_itself_str)
            exec(self.try_print_exception_str)

            return False


    #volume range:0~100
    def set_volume(self, volume):
        """

        Set sound volume

        @param volume: Volume (Int)
        @return:
                 None    -> If timeout or disconnect (None)
                 "OK"    -> Success request (String)
                 "NO_OK" -> Fail request (String)
        """
        for trycount in range(1):
            try:
                url = self.__url+"/set_audio"
                

                payload = { "audio": {
                                 "volume" : int(volume),
                                },
                        "mac": self.__myMac}
                        
                #t1 = time.time();
                #print("Device r pre:",t1)
                r = self.__session_file.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue

                #exec(self.check_no_ok)

                if len(r.content)==0:
                    return None
                if len(r.content)==5 :
                    if r.text=="NO_OK":
                        return None
                    
                return r.content

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None


    #volume range:0~100
    def get_volume(self, try_count_max=EXCEPT_MAX_RETRY_TIMES, to=REQUEST_CONNECT_TIMEOUT):
        """

        Get sound volume

        @param try_count_max: Max times to retry request (Int)
        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                None    -> If timeout or disconnect (None)
                69      -> Sound volume (Int)
        """
        r=None
        for trycount in range(try_count_max):
            try:
                #機器的相關性能與外觀資訊
                url = self.__url+"/get_audio"
                payload = {"mac": self.__myMac}
                r = self.__session_get_misc.post(url, data=json.dumps(payload), timeout=REQUEST_CONNECT_TIMEOUT )
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                data=r.json()
                print(data)
                return data['audio']['volume']
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)
        return None



    def get_fiducial(self, try_count_max=EXCEPT_MAX_RETRY_TIMES, to=REQUEST_CONNECT_TIMEOUT):
        """
        Get a bunch of basic fiducial from AMR

        @param try_count_max: Max times to retry request (Int)
        @param to: Timeout of every request, unit: seconds (Float)
        @return:
                 None      -> If timeout or disconnect (None)
                 "NO_OK"   -> Fail request (String)
                 JSON data ->
                            {
                              "id": [1,290,300],
                              "pose3d": [[-0.7840,-0.0138,0.2243,0.0032,0.0061,-3.0856], //x<m> y<m> z<m> roll<rad> pitch<rad> yaw<rad>
                                         [-0.6840,-0.0238,0.3243,0.0042,0.0061,-2.0856],
                                         [-0.5840,-0.0338,0.4243,0.0052,0.0061,-1.0856]]
                            }
        """
        r=None
        for trycount in range(try_count_max):
            try:
                #sensor 看到tag的資訊
                url = self.__url+"/get_fiducial"
                payload = {"mac": self.__myMac}
                r = self.__session_get_misc.post(url, data=json.dumps(payload), timeout=to )
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue
                exec(self.check_no_ok)
                data=r.json()
                return data
            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None


    def get_ble(self, timeout=REQUEST_CONNECT_TIMEOUT):
        for trycount in range(1):
            try:
 
                url = self.__url+"/get_ble"
                payload = { "mac": self.__myMac }
                        
                #t1 = time.time();
                #print("Device r pre:",t1)
                r = self.__session_file.post(url, data=json.dumps(payload), timeout=timeout)
                if r.status_code!=200:
                    print("====== {",sys._getframe().f_code.co_name,"} r.status_code!=200 ======")
                    continue

                #exec(self.check_no_ok)

                if len(r.content)==0:
                    return None
                if len(r.content)==5 :
                    if r.text=="NO_OK":
                        return None
                    
                return r.json()

            # except requests.exceptions.RequestException as e:
            #     exec(self.try_print_exception_itself_str)
            #     return None
            except Exception as e:
                print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))
                exec(self.try_print_exception_itself_str)
                exec(self.try_print_exception_str)

        return None
###################################################################################################



