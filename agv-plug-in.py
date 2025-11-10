# -*- coding: utf-8 -*-
"""
Created on Wed Sep 24 10:17:31 2025

@author: Leonli

這程式會運行在FMS端 因此 原有的AMR會透過 FMS IP 取得其他車輛的資訊
用來連接外部的AGV的空間資訊，方便目前運行的AMR可以透過網路詢問到其他車輛的空間資訊來進行交通管理
"""

from flask import Flask, request, jsonify
import time
import math
from typing import List
import numpy as np
import os
import zlib
import struct
import argparse
import json
import sys
import concurrent.futures
import device
import signal
import requests


###################################################################################################

###################################################################################################
def signal_handler(sig, frame):
    print("CTRL+C pressed. Exiting...")
    sys.exit(0)
###################################################################################################
# CRC-16 查表
CRCtbl = [
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
]

def addCRC(CRC, b):
    """計算單位 byte 的 CRC"""
    return ((CRC >> 8) ^ CRCtbl[(CRC & 0xFF) ^ b]) & 0xFFFF

def CRC16(data: bytes) -> int:
    """計算整段 bytes 的 CRC16"""
    CRC = 0
    for b in data:
        CRC = addCRC(CRC, b)
    return CRC

#Str2Uuid('msi3F_2')=30909 check OK
def Str2Uuid(s: str) -> int:
    """將字串轉成 16-bit UUID"""
    data = s.encode('utf-8')  # 轉 bytes
    return CRC16(data)

###################################################################################################
def ToShape(x: float, y: float, a: float, length: float, width: float, center2front: float) -> List[float]:
    """
    計算 AGV 四個角點的全局座標
    :param x: 中心點 x
    :param y: 中心點 y
    :param a: 朝向角度 (rad)
    :param length: 車長
    :param width: 車寬
    :param center2front: 中心到車頭的距離
    :return: [[x0, y0], [x1, y1], [x2, y2], [x3, y3]]
    """
    shape_xy = []
    cos_value = math.cos(a)
    sin_value = math.sin(a)

    # 定義相對於中心的四個角點
    points = [
        (center2front, width / 2.0),            # front-left
        (center2front - length, width / 2.0),   # rear-left
        (center2front - length, -width / 2.0),  # rear-right
        (center2front, -width / 2.0)            # front-right
    ]

    # 旋轉 + 平移到全局座標
    for px, py in points:
        new_x = px * cos_value - py * sin_value + x
        new_y = px * sin_value + py * cos_value + y
        shape_xy.append([new_x, new_y])

    return shape_xy
###################################################################################################
def AGV_pose2AMR_pose(pose=[]):
    return []
###################################################################################################
def utc_to_mod(utc: float) -> int:
    result = round(math.fmod(utc * 10.0, 4294967296.0))
    return int(result)
################################################################################################### 
def mod_to_utc(utc_mod: int, current_time: float) -> float:
    time_plus = current_time / 10.0 + 100.0
    tmp = math.fmod(time_plus, 4294967296.0)

    if utc_mod < tmp:
        return time_plus - (tmp - utc_mod)
    return time_plus - (tmp - utc_mod) - 4294967296.0
################################################################################################### 
# x,y m -> x,y grid 
#player_map2d_t CoordWorld2Map(player_pose2d_t w)
#{
#  player_map2d_t m;
#  m.px = (int)(((w.px - GetOriginX()) / GetResolution() + 0.5)); 
#  m.py = (int)(((w.py - GetOriginY()) / GetResolution() + 0.5));
#  
#  return m;
#};
def XY2Grid(x,y,MapOriginX=81.92,MapOriginY=81.92,MapResolution=0.02):    
    mx=int(round((x-MapOriginX)/MapResolution)) #m -> grid
    my=int(round((y-MapOriginY)/MapResolution))
    
    return mx,my
###################################################################################################
# x,y grid -> x,y m 
#player_pose2d_t CoordMap2World(player_map2d_t m)
#{
#  player_pose2d_t w;
#  w.px = ((double)m.px * GetResolution()) + GetOriginX();
#  w.py = ((double)m.py * GetResolution()) + GetOriginY();
#  w.pa = 0;
#
#  return w;
#};
def Grid2XY(mx,my,MapOriginX=81.92,MapOriginY=81.92,MapResolution=0.02):    
    x=float(mx*MapResolution)-MapOriginX
    y=float(my*MapResolution)-MapOriginY

    
    return x,y
###################################################################################################
#return
#header
#arr [y,x]

def MapName2Numpy(path: str, mapName: str = "map_69.map") -> tuple[dict, np.ndarray]:
    
    HEADER_FORMAT = "<fII ddd B IIIII b"  # little-endian (根據 C 結構)
    HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 58 bytes

    full_path = os.path.join(path, mapName)
    
    
    if not os.path.exists(full_path):
        print(f"Error: {full_path} not found. Exiting...")
        sys.exit(1)
        

    with open(full_path, "rb") as f:
        file_data = f.read()

    # --- 解析 Header ---
    header_data = file_data[:HEADER_SIZE]
    (
        scale, width, height,
        origin_px, origin_py, origin_pa,
        lock, start_col, start_row,
        end_width, end_height,
        data_count, data_range
    ) = struct.unpack(HEADER_FORMAT, header_data)

    header = {
        "scale": scale,
        "width": width,
        "height": height,
        "origin_px": origin_px,
        "origin_py": origin_py,
        "origin_pa": origin_pa,
        "lock": lock,
        "start_col": start_col,
        "start_row": start_row,
        "end_width": end_width,
        "end_height": end_height,
        "data_count": data_count,
        "data_range": data_range,
    }

    print(mapName,"Header 解析結果:", header)

    # --- 嘗試解壓縮 ---
    compressed_data = file_data[HEADER_SIZE:]

    modes = {
        "zlib": zlib.MAX_WBITS,
        "gzip": 16 + zlib.MAX_WBITS,
        "raw": -zlib.MAX_WBITS,
    }
    decompressed_data = None
    for name, wbits in modes.items():
        try:
            decompressed_data = zlib.decompress(compressed_data, wbits=wbits)
            print(mapName,"成功使用 {name} 模式解壓縮")
            break
        except zlib.error:
            continue

    if decompressed_data is None:
        raise RuntimeError(mapName,"解壓縮失敗，檔案格式未知")

    # --- 轉成 numpy ---
    arr = np.frombuffer(decompressed_data, dtype=np.uint8)
    if arr.size != width * height:
        raise ValueError(mapName,"大小不符: {arr.size} != {width*height}")

    arr = arr.reshape((height, width))
    return header, arr
###################################################################################################
#read from /home/pyuser/amr_info.json
class AmrInfo():
    def __init__(self, name, group, ip, port, wifiMac,bleMac):
        self.name=name
        self.group=group
        self.ip=ip
        self.port=port
        self.wifiMac=wifiMac.lower().replace(" ", "") #lowcase
        self.bleMac=bleMac.lower().replace(" ", "")  #lowcase
        self.isOnline=False #wifi online
        self.whenOnline=time.time() #UTC time
###############################################################################
def ReadAMRList(amr_list_file):
    amrInManageList=[]
    # read all AMR had manager by FMS 
    if os.path.exists(amr_list_file):
        try:
            file=amr_list_file
            # JSON file
            f=open(file, "r")
            # Reading from file
            cfg = json.load(f)
            #print(cfg)
            
            # Closing file
            f.close()
            # Iterating through the json
            # list
            print("-------------read who in manage----------------------")
            for amr in cfg['AMR']:
                name="None"
                group="None"
                ip='127.0.0.1'
                port=6660
                wifiMac="00:00:00:00:00:00"
                bleMac="00:00:00:00:00:00"

                if "name" in amr:
                    name=amr['name'].replace("\n", "").strip()

                if "group" in amr:
                    group=amr['group'].replace("\n", "").strip()
                else:
                    group=name

                if "ip" in amr:
                    ip=amr['ip'].replace("\n", "").strip()
                
                if "port" in amr:
                    port=amr['port']     

                if "wifi_mac" in amr:
                    wifiMac=amr['wifi_mac'].lower().replace("\n", "").strip()

                if "ble_mac" in amr:
                    bleMac=amr['ble_mac'].lower().replace("\n", "").strip()

                print("name=%s group=%s ip=%s port=%d wifiMac=%s bleMac=%s" % (name,group,ip,port,wifiMac,bleMac))
                amrInManageList.append(AmrInfo(name,group,ip,port,wifiMac,bleMac))
            print("-----------end read who in manage--------------------")
                    
                    
        except ValueError as e:
            print("ReadAMRList",e)
    
    return amrInManageList
###################################################################################################
#mac         wifi mac
#pose        unit:[m,m,rad]
#vel         unit:[m/s,m/s,rad/s]
#length      unit:m
#width       unit:m
#center2from unit:m
class AGV():
    def __init__(self,agvInfo,posexya=[0,0,0],velxya=[0,0,0],length=0.6,width=0.58,center2from=0.2):
        self.info=agvInfo
        self.pose=posexya #unit m,m,rad
        self.vel=velxya   #unit m/s,m/s,rad/s
        self.length=length
        self.width=width
        self.center2from=center2from
        
        self.area_index=0
        self.area_index_timestamp=0
        self.polygon=ToShape(posexya[0],posexya[1],posexya[2],length,width,center2from)
        self.mapName=""
        self.map_uuid=-1

        
        self.mutexMapInfo=None
        self.mutexMap=None
        
        
        
    def GetMutexMapCell(self,x,y):
        val=0
        try:
            x=int(round(x,0))
            y=int(round(y,0))
            #print("GetMutexMapCell",x,y,self.mutexMapInfo['width'] ,self.mutexMapInfo['height'])
            if isinstance(self.mutexMap, np.ndarray)==False or x<0 or y<0 or x>=self.mutexMapInfo['width'] or y>=self.mutexMapInfo['height']:
                #print("GetMutexMapCell fuck")
                pass
            else:
                val=int(self.mutexMap[y,x]) #[y,x]
                #val1=self.__map.GetCell(x,y)
                #print(x,y,val0,val1)
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(e, exc_type, fname, exc_tb.tb_lineno,"(",x,y,")")
            
        return val
    
    #posexya unit grid, grid , rad 傳入的 座標系統可以透過 FMS 的 grid 點去查 剛好跟 nv2 的上下顛倒
    #velxya  unit grid/s, grid/s , rad/s
    def update(self,posexya=[0,0,0],velxya=[0,0,0],mapPath="",mapName=""):
        if self.mapName!=mapName:
            mutexMapInfo, mutexMap=MapName2Numpy(mapPath+mapName)
            self.mutexMapInfo=mutexMapInfo
            self.mutexMap=mutexMap
            self.mapName=mapName
            
            
            '''
            #for debug
            from PIL import Image
            
            #print("AAAAAAAA", mutexMap.max(),type(mutexMap),mutexMap.shape,mutexMap.dtype)
            debug=np.where(mutexMap != 0, np.uint8(128), np.uint8(0))
            #print("BBBBBBBB", debug.max(),type(debug),debug.shape,debug.dtype)
            im=Image.fromarray(debug)
            im.save("mutex_map.png")
            #for debug
            '''

            
        
        x,y=Grid2XY(posexya[0],self.mutexMapInfo['height']-posexya[1],
                    self.mutexMapInfo['origin_px'],self.mutexMapInfo['origin_py'],self.mutexMapInfo['scale'])
        
        vx,vy=Grid2XY(velxya[0],velxya[1],
                    self.mutexMapInfo['origin_px'],self.mutexMapInfo['origin_py'],self.mutexMapInfo['scale'])
        
        
        self.pose=[x,y,posexya[2]]
        self.vel=[vx,vy,velxya[2]]
        area_index=self.GetMutexMapCell(posexya[0],self.mutexMapInfo['height']-posexya[1])
        
        if area_index>0 and self.area_index==0:
            self.area_index_timestamp=utc_to_mod(time.time())
            print("mac=%s index=%d" % (self.info.wifiMac,area_index))
            print("pose",self.pose)
        elif area_index==0:
            self.area_index_timestamp=0
            print("mac=%s index=%d" % (self.info.wifiMac,area_index))
            print("pose",self.pose)
            
        self.area_index=area_index
        self.map_uuid=Str2Uuid(self.mapName)
        self.polygon=ToShape(self.pose[0],self.pose[1],self.pose[2],self.length,self.width,self.center2from)

###################################################################################################
def RunFlask(agv,map_grup_path,amrIpList):
    app=Flask(agv.info.name)
    #print("AAAAAAAAAA",agv.info.name)
    

    # 模擬 BLE 資訊（可改為實際資料來源）
    BLE_DATA = {
      "me":{"mac":"AA:BB:CC:DD:EE:FF",
            "area_index":0,
            "area_index_timestamp":0,
            "pose":[0,0,0.0,0.0],
            "vel":[0,0,0.0,0.0],
            "polygon":[[0.0,0.0],[0.0,0.0],[0.0,0.0],[0.0,0.0]],
            "map_uuid":-1} 
    }
    BLE_DATA['me']['mac']=agv.info.bleMac #read from 
    

    #x, y unit:m
    def ShowAGVonFMS(agvID,x,y):
        
        try:
            url = "http://127.0.0.1:6600"+"/set_drone"
            
            payload = { "act":"None",            # "None" -> 新增, 更新
                                                 # "DEL" ->  將drone刪除
                        "x": x,              
                        "y": y,
                        "battery": 81,
                        "id": agvID               # drone的ID, 不同的ID會再新增1個drone的物件
                      }
            requests.post(url, data=json.dumps(payload))   
            
        except Exception as e:
            print("====== {func_name} exception ======".format(func_name=sys._getframe().f_code.co_name))

        
        
        
        
        
    def shouldAgvPause(agv_map_uuid,agv_area_index,agv_area_index_timestamp):
        if agv_area_index>0:
            for ip in amrIpList:
                me=device.robot(ip)
                amr=me.get_ble()
                if amr['me']['area_index']==agv_area_index and \
                   amr['me']['map_uuid']==agv_map_uuid and  \
                   amr['me']['area_index_timestamp']<agv_area_index_timestamp:
                       return True
        return False
                    
        
    def round_floats(obj, digits=4):
        if isinstance(obj, float):
            return round(obj, digits)
        elif isinstance(obj, list):
            return [round_floats(x, digits) for x in obj]
        elif isinstance(obj, dict):
            return {k: round_floats(v, digits) for k, v in obj.items()}
        else:
            return obj
    
    
    
    
    #準備好給其他AMR詢問這台 AGV資訊用的介面
    @app.route('/get_ble', methods=['POST'])
    def get_ble():
        try:
            client_ip = request.remote_addr  # <-- 取得連進來的 IP
            print(f"Request from IP: {client_ip}")
            
            # 模擬處理，這裡直接返回 BLE_DATA
            return jsonify(round_floats(BLE_DATA))
        
        except Exception as e:
            return jsonify({"error": str(e)}), 500
        
        
        

    #提供給 AGV 跟新她在 AMR地圖上的空間位置資訊
    #回傳 AGV 是否需要暫停
    @app.route('/update_pose', methods=['POST'])
    def update_pose():
        try:
            """
            接收 JSON 格式:
            {
                "pose": [x, y, theta],
                "vel": [vx, vy, vtheta],
                "map_name": "SMC_manufacture"
            }
            """
            data = request.get_json()
            if not data:
                return jsonify({"error": "No JSON payload received"}), 400
        
            posexya=data.get("pose") #grid,grid,rad
            velxya=data.get("vel")
            mapName=data.get("map_name")
            
            #更新AGV目前的空間位置以及採用的地圖名子
            agv.update(posexya,velxya,map_grup_path,mapName)
            BLE_DATA['me']['area_index']=agv.area_index
            BLE_DATA['me']['area_index_timestamp']=agv.area_index_timestamp
            BLE_DATA['me']['pose']=agv.pose
            BLE_DATA['me']['vel']=agv.vel
            BLE_DATA['me']['polygon']=agv.polygon
            BLE_DATA['me']['map_uuid']=agv.map_uuid
            
            isPause=shouldAgvPause(agv.map_uuid,agv.area_index,agv.area_index_timestamp)
            
            ShowAGVonFMS(agv.info.name,agv.pose[0],agv.pose[1])
            
            PUSE_DATA = {
              "pause": isPause
            }    
            
            return jsonify(PUSE_DATA)
        
        except Exception as e:
            return jsonify({"error": str(e)}), 500    
    

    app.run(host='0.0.0.0',port=agv.info.port)
###################################################################################################
if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)  # 捕捉 CTRL+C
    
    # Initiate the parser
    parser = argparse.ArgumentParser()

    # Add long and short argument
    parser.add_argument("--map_grup_path", "-MGP", type=str, default="..\\..\\map\\", help="set which map group use")
    parser.add_argument("--map_grup_name", "-MGN", type=str, default="20250902_air", help="set which map group use")
    parser.add_argument("--amr_info_list", "-AIL", type=str, default="..\\..\\amr_list.json", help="the amr info list")


    # Read arguments from the command line
    args = parser.parse_args()    
    
    if not os.path.exists(args.amr_info_list):
        print(f"Error: {args.amr_info_list} not found. Exiting...")
        sys.exit(1)

    
    amrInfoList=ReadAMRList(args.amr_info_list)
    
    
    agvList=[]
    amrIpList=[]
    for amrInfo in amrInfoList:
        if amrInfo.port!=6660:
            print("AGV:",amrInfo.name,amrInfo.wifiMac,amrInfo.ip,amrInfo.port)
            agv=AGV(amrInfo)
            agvList.append(agv)
        else:
            print("AMR:",amrInfo.name,amrInfo.wifiMac,amrInfo.ip,amrInfo.port)
            amrIpList.append(amrInfo.ip)
            
        
        
        
    
    
    
    n=len(agvList)
    map_grup_path_list=[args.map_grup_path for i in range(n)]
    amrIpLists=[amrIpList for i in range(n)]
    with concurrent.futures.ThreadPoolExecutor(max_workers=n) as executor:
        results=list(executor.map(RunFlask,agvList,map_grup_path_list,amrIpLists))
        
        
        
        
        