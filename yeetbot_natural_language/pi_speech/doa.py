import subprocess
import os
import json
import numpy as np

#begin
os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/odas/bin/")
doa_matrix = subprocess.Popen(["./matrix-odas"], stdout=subprocess.PIPE)
doa_odas = subprocess.Popen(["./odaslive", "-c", "../config/matrix-demo/matrix_voice.cfg"])

mlem = 4

while True:    
    output = doa_matrix.stdout.readline()
    mlem += 1
    if mlem == 9:
        output = json.loads(output[:-2])
        angle = np.arctan2(output.get('y'), output.get('x'))
        print angle
        mlem -= 9

def get_doa(x, y):
    return np.arctan2(y,x)
