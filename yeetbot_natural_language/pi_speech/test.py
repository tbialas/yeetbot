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
    global read_buffer
    global port

    #initialise ros variables
    state = 0
    inventory = ["hammer", "pliers", "screwdriver"]
    ros_buffer = deque()
    buffer_lock = multiprocessing.Lock()

def listen_wake_word():
    print('mlem')

def wait_for_input():
    delay = raw_input("input anything for YEETBot to listen; to quit input 'quit'\n")
    if delay == "quit":
        sys.exit()

def record_speech():
    print("listening...\n")
    subprocess.call(["arecord", "recording.wav", "-f", "S16_LE", "-r", "44100", "-d", "3", "-D", "hw:3,0"])

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
    if response.results:
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

        print "\n"
        print serial_user_response
    else:
        print "no words recognized!\n"

def main():
    init()
    while True:
        wait_for_input()
        if state == 0:
            record_speech()
            transcribe_file("recording.wav")

if __name__ == '__main__':
    main()
