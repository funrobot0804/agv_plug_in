# -*- coding: utf-8 -*-
"""
Created on Wed Sep 24 13:27:34 2025

@author: leonli
"""

import requests

url = "http://127.0.0.1:7001/update_pose"

data = {
    #"pose": [4249,4674, 3.14], # FMS 上面顯示的 grid block
    "pose": [4055,4255,0], # FMS 上面顯示的 grid 
    "vel": [1.01, 2.01, 3.01],
    "map_name": "20250902_air"
}

try:
    response = requests.post(url, json=data, timeout=5)
    print("Status Code:", response.status_code)
    print("Response:", response.json())
except requests.exceptions.RequestException as e:
    print("Error:", e)