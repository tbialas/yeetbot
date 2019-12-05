#!/usr/bin/env python

import subprocess
import io
import sys
import serial
import multiprocessing
import time
from collections import deque
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

def init():
    global computer
    global state
    global inventory
    global ros_buffer
    global buffer_lock

    #serial port initialise and handshake
    computer = serial.Serial(
        port = '/dev/ttyACM0',
        baudrate=115200,
        timeout=0.5)
    computer.write(">yeet<")
    
    done = 0
    while not done:
        cmd = computer.read_until(">ack<")
        if cmd == ">ack<":
            done = 1

    #initialise ros variables
    state = 0
    inventory = []
    ros_buffer = deque()
    buffer_lock = multiprocessing.Lock()

    #initialise thread for listening to computer and reading buffer
    port = multiprocessing.Process(target=listen_serial)
    port.start()

    read_buffer = multiprocessing.Process(target=read_ros_buffer)
    read_buffer.start()

def listen_serial():
    while True:
        time.sleep(0.1)
        cmd = ''
        cmd = computer.read_until("<")
        if cmd:
            with buffer_lock:
                ros_buffer.append([cmd])

def read_ros_buffer():
    while True:
        with buffer_lock:
            if ros_buffer:
                msg = ros_buffer.popleft()
                update_states(msg)
            else:
                continue
        time.sleep(0.3)

def update_states(msg):
    global state
    global inventory

    topic, msg = msg[:3], msg[3:-1]
    
    #inventory update
    if topic == ">i/":
        inventory = list(msg.split(","))
    #state update
    elif topic == ">s/":
        state = int(msg)

def listen_wake_word():
    print('mlem')

def wait_for_input():
    delay = raw_input("input anything for YEETBot to listen; to quit input 'quit'\n")
    if delay == "quit":
        sys.exit()

def record_speech():
    if state == 1:
        print("listening...\n")
        subprocess.call(["arecord", "recording.wav", "-f", "S16_LE", "-r", "16000", "-d", "3"])

def transcribe_file(speech_file):
    if state == 1:
        client = speech.SpeechClient()

        with io.open(speech_file, 'rb') as audio_file:
            content = audio_file.read()

        audio = types.RecognitionAudio(content=content)
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code='en-GB')

        response = client.recognize(config, audio)

        #over serial, user response composed as ">u/[choice],[invalid_choice]"
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
                serial_user_response += "t" if invalid_choice else "f"
            serial_user_response += "<"

        computer.write(serial_user_response)

def main():
    init()
    while True:
        wait_for_input()
        record_speech()
        transcribe_file("recording.wav")

if __name__ == '__main__':
    main()
