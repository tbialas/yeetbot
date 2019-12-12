#!/usr/bin/env python
import sys
sys.path.append("./snowboy/examples/Python/")
import snowboydecoder_arecord as snowboydecoder
import subprocess
import signal
import io
#import sys
import os
import serial
import multiprocessing
import time
import math
import json
import numpy as np
from ctypes import c_bool
from decimal import Decimal as D
from google.cloud import speech
from google.cloud import texttospeech
from google.cloud.speech import enums
from google.cloud.speech import types

ANGLE_ELEMENT_NUM = 10
IDLE = 0
RECEIVING_REQUEST = 1
RECEIVING_TOOL_EARLY = 2
RECEIVING_TOOL_ON_TIME = 3
RECEIVING_TOOL_LATE = 4
GIVING_TOOL = 5
TRAVELLING = 6

snowboy = None

def init():
    global computer
    global buffer_lock
    global read_buffer
    global port
    global doa_matrix
    global doa_odas
    global snowboy_lock
    global request_needed
    global manager
    global state
    global inventory
    global ros_buffer
    global request_text
    global wakeword_process

    subprocess.call(["killall", "odaslive"])
    #subprocess.call(["killall", "matrix-odas"])

    computer = serial.Serial(
        port = '/dev/mbed',
        baudrate=115200,
        timeout=0.3)

    #computer.write("pi speech alive")
    #done = 0
    #while not done:
    #    cmd = computer.read_until(">yeet<")
    #    if cmd == ">yeet<":
    #        done = 1

    computer.write(">yeet<")

    #initialise ros variables
    manager = multiprocessing.Manager()
    state = multiprocessing.Value('i', IDLE)
    inventory = manager.list()
    ros_buffer = multiprocessing.Queue()
    buffer_lock = multiprocessing.Lock()

    #start doa
    doa_process = multiprocessing.Process(target=listen_matrix)
    doa_process.start()

    #initialise thread for listening to computer and reading buffer and wakeword
    port = multiprocessing.Process(target=listen_serial, args=(ros_buffer,))
    port.start()

    read_buffer = multiprocessing.Process(target=read_ros_buffer, args=(ros_buffer,))
    read_buffer.start()
    
    #start wake word detection   
    request_needed = multiprocessing.Value(c_bool, False)
    snowboy_lock = multiprocessing.Lock()
    wakeword_process = multiprocessing.Process(target=listen_wake_word)
    wakeword_process.start()

    
    #start_wake_word()
    
    tts("yeetbot 3000, online")

def listen_serial(queue):
    while True:
        cmd = ''
        time.sleep(0.1)
        try:
            cmd = computer.read_until("<")
            if cmd:
                with buffer_lock:
                    queue.put(cmd)
                    if queue.qsize() > 2:
                        queue.get()
        except:
            time.sleep(0.1)

def listen_matrix():
    global computer

    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/odas/bin/")
    doa_matrix = subprocess.Popen(["./matrix-odas"], stdout=subprocess.PIPE)
    doa_odas = subprocess.Popen(["./odaslive", "-c", "../config/matrix-demo/matrix_voice.cfg"])
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/")

    offset=4
    while True:
        output = doa_matrix.stdout.readline()
        offset += 1
        if offset == 9:
            output = json.loads(output[:-2])
            angle = np.arctan2(output.get('x'), output.get('y'))
            serial_angle = ">a/" + str(angle) + "<"
            computer.write(serial_angle)
            offset -= 9

def wakeword_detected():
    global request_needed
    global snowboy
    global computer

    print("wakeword detected")
    request_needed.value = True
    serial_detect = ">d/<"
    computer.write(serial_detect)
    snowboy.terminate()

def listen_wake_word():
    global snowboy

    snowboy = snowboydecoder.HotwordDetector("snowboy/examples/Python/resources/models/snowboy.umdl", sensitivity=0.5)
    print("starting thread snowboy")
    snowboy.start(detected_callback=wakeword_detected, sleep_time=0.25)

def read_ros_buffer(queue):
    global ros_buffer
    global wakeword_process
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
        state.value = int(msg)
        if state.value == IDLE:
            try:
                wakeword_process = multiprocessing.Process(target=listen_wake_word)
                wakeword_process.start()
            except:
                pass
    #make yeetbot speak
    elif topic == ">m/":
        tts(msg)

def record_speech():
    global snowboy
    global wakeword_process
    global request_needed

    print("listening...\n")
    wakeword_process.terminate()
    wakeword_process.join()
    subprocess.call(["killall", "arecord"])
    time.sleep(0.1)
    subprocess.call(["arecord", "recording.wav", "-f", "S16_LE", "-r", "44100", "-d", "4", "-D", "hw:2,0"])
    transcribe_file("recording.wav")
    request_needed.value = False
    wakeword_process = multiprocessing.Process(target=listen_wake_word)
    wakeword_process.start()
  

def tts(text):
    print text
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
    global request_needed
    
    init()
    while True:
        if state.value == IDLE:
            time.sleep(0.05)

        elif state.value == RECEIVING_REQUEST:
            if request_needed.value:
                record_speech()
            else:
                time.sleep(0.05)

        elif state.value == RECEIVING_TOOL_EARLY:
            time.sleep(0.05)
        elif state.value == RECEIVING_TOOL_LATE:
            time.sleep(0.05)
        elif state.value == RECEIVING_TOOL_ON_TIME:
            time.sleep(0.05)
        elif state.value == GIVING_TOOL:
            time.sleep(0.05)
        elif state.value == TRAVELLING:
            time.sleep(0.05)
        else:
            time.sleep(0.05)

if __name__ == '__main__':
    main()
