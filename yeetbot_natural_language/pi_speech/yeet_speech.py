#!/usr/bin/env python

import subprocess
import signal
import io
import sys
import os
import serial
import multiprocessing
import time
import math
from ctypes import c_bool
from decimal import Decimal as D
from google.cloud import speech
from google.cloud import texttospeech
from google.cloud.speech import enums
from google.cloud.speech import types

ANGLE_ELEMENT_NUM = 10
IDLE = "0"
RECEIVING_REQUEST = "1"
RECEIVING_TOOL_EARLY = "2"
RECEIVING_TOOL_ON_TIME = "3"
RECEIVING_TOOL_LATE = "4"
GIVING_TOOL = "5"
TRAVELLING = "6"

def init():
    global computer
    global buffer_lock
    global read_buffer
    global port
    global doa_matrix
    global doa_odas
    global snowboy_lock
    global wake_word_detected
    global wakeword_process
    global manager
    global state
    global inventory
    
    subprocess.call(["killall", "odaslive"])
    subprocess.call(["killall", "matrix-odas"])

    #serial port initialise and handshake
    computer = serial.Serial(
        port = '/dev/ttyACM0',
        baudrate=115200,
        timeout=0.5)
    computer.write("YEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEET")
    #done = 0
    #while not done:
    #    cmd = computer.read_until(">yeet<")
    #    if cmd == ">yeet<":
    #        done = 1

    #computer.write(">ack<")

    #initialise ros variables
    manager = multiprocessing.Manager()
    state = multiprocessing.Value('i', 0)
    inventory = manager.list()
    ros_buffer = multiprocessing.Queue()
    buffer_lock = multiprocessing.Lock()

    #start doa
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/odas/bin/")
    doa_matrix = subprocess.Popen(["./matrix-odas", ">", "/dev/null/", "2>&1"])
    doa_odas = subprocess.Popen(["./odaslive", "-vc", "../config/matrix-demo/matrix_voice.cfg", ">", "/dev/null/", "2>&1"])
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/")

    #initialise thread for listening to computer and reading buffer and wakeword
    port = multiprocessing.Process(target=listen_serial, args=(ros_buffer,))
    port.start()

    read_buffer = multiprocessing.Process(target=read_ros_buffer, args=(ros_buffer,))
    read_buffer.start()
    
    #start wake word detection
    with open("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/snowboy/examples/Python/detected.txt", "w+") as f:
        f.write("0")
        
    wake_word_detected = multiprocessing.Value(c_bool, False)
    snowboy_lock = multiprocessing.Lock()
    start_wake_word()
    wakeword_process = multiprocessing.Process(target=listen_wake_word)
    wakeword_process.start()
    
    tts("yeetbot 3000, online")

def listen_serial(queue):
    while True:
        time.sleep(0.1)
        cmd = ''
        cmd = computer.read_until("<")
        if cmd:
            with buffer_lock:
                queue.put(cmd)

def start_wake_word():
    global snowboy
    global wakeword_process
    
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/snowboy/examples/Python/")
    snowboy = subprocess.Popen(["python", "demo_arecord.py", "resources/models/snowboy.umdl"])
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/")
    
    wakeword_process = multiprocessing.Process(target=listen_wake_word)
    wakeword_process.start()

def listen_wake_word():
    global snowboy
    global wake_word_detected
    
    while True:
        time.sleep(0.2)
        with open("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/snowboy/examples/Python/detected.txt", "r") as f:
            if f.read() == "1":
                wake_word_detected.value = True
            else:
                wake_word_detected.value = False

def read_ros_buffer(queue):
    while True:
        with buffer_lock:
            if not ros_buffer.empty():
                msg = ros_buffer.get()
                update_states(msg)
            else:
                continue
        time.sleep(0.3)

def update_states(msg):
    global inventory
    topic, msg = msg[:3], msg[3:-1]
    
    #inventory update
    if topic == ">i/":
        inventory[:] = []
        for tool in list(msg.split(",")):
            inventory.append(tool)
    #state update
    elif topic == ">s/":
        state.val = msg
    #make yeetbot speak
    elif topic == ">m/":
        tts(msg)

def deg2rad_mode(angle_list):
    doa = int(max(set(angle_list), key=angle_list.count))
    return str(round(math.radians(doa), 2))

def send_doa():
    with open("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/odas/bin/angles.txt") as f:
        fl = f.readlines()[0:ANGLE_ELEMENT_NUM]
        serial_angle = ">a/" + deg2rad_mode(fl) + "<"
        computer.write(serial_angle)

def send_state(next_state):
    msg = ">s/" + next_state + "<"
    computer.write(msg)

def wake_word_detected():
    doa_restart()
    
def wait_for_input():
    delay = raw_input("input anything for YEETBot to listen; to quit input 'quit'\n")
    if delay == "quit":
        port.terminate()
        read_buffer.terminate()
        read_buffer.join()
        port.terminate()
        port.join()
        
        sys.exit()

def record_speech():
    global snowboy
    global wakeword_process
    
    print("listening...\n")
    snowboy.send_signal(signal.SIGINT)
    wakeword_process.terminate()
    wakeword_process.join()
    subprocess.call(["killall", "arecord"])
    subprocess.call(["arecord", "recording.wav", "-f", "S16_LE", "-r", "44100", "-d", "4", "-D", "hw:2,0"])
    doa_restart()
    start_wake_word()

def tts(text):
    client = texttospeech.TextToSpeechClient()
    synthesis_input = texttospeech.types.SynthesisInput(text=text)
    voice = texttospeech.types.VoiceSelectionParams(
        language_code="en-AU",
        name="en-AU-Wavenet-B",
        ssml_gender=texttospeech.enums.SsmlVoiceGender.MALE)

    audio_config = texttospeech.types.AudioConfig(
        audio_encoding=texttospeech.enums.AudioEncoding.LINEAR16)

    response = client.synthesize_speech(synthesis_input, voice, audio_config)
    with open('yeetbot_talks.wav', 'wb') as out:
        out.write(response.audio_content)

    subprocess.call(["aplay", "yeetbot_talks.wav"])

def doa_restart():
    global doa_matrix
    global doa_odas

    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/odas/bin/")
    #kill process
    doa_odas.send_signal(signal.SIGINT)
    time.sleep(0.2)
    send_doa()
    doa_matrix.send_signal(signal.SIGINT)
    #restart process
    doa_matrix = subprocess.Popen(["./matrix-odas"])
    doa_odas = subprocess.Popen(["./odaslive", "-vc", "../config/matrix-demo/matrix_voice.cfg"])
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/")

def transcribe_file(speech_file):
    client = speech.SpeechClient()

    with io.open(speech_file, 'rb') as audio_file:
        content = audio_file.read()

    audio = types.RecognitionAudio(content=content)
    config = types.RecognitionConfig(
        encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=44100,
        language_code='en-GB')

    response = client.recognize(config, audio)

    #over serial, user response composed as ">u/[choice],[invalid_choice]"

    serial_user_response = ""
    
    for result in response.results:
        sentence = result.alternatives[0].transcript.split()
        serial_user_response = ">u/"
        if "return" in sentence:
            serial_user_response += "-1,f"
        else:
            invalid_choice = True
            for word in sentence:
                for tool in inventory:
                    if word == tool:
                        serial_user_response += str(inventory.index(tool)) + ","
                        invalid_choice = False
			print tool
            serial_user_response += "t" if invalid_choice else "f"
        serial_user_response += "<"

    computer.write(serial_user_response)

def main():
    init()
    while True:
        while not wake_word_detected.value:
            time.sleep(0.05)
            transcribe_file("recording.wav")
        doa_restart()
        wake_word_detected.value = False
        with open("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/snowboy/examples/Python/detected.txt", "w") as f:
            f.write("0")
        while state.val != RECEIVING_REQUEST:
            time.sleep(0.2)
        record_speech()
        transcribe_file("recording.wav")
        while state != IDLE:
            time.sleep(0.2)
        
        
            
if __name__ == '__main__':
    main()
