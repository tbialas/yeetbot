#!/usr/bin/env python

import subprocess
import signal
import io
import sys
import os
import serial
import multiprocessing
import time
from collections import deque
from google.cloud import speech
from google.cloud import texttospeech
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
    global doa_matrix
    global doa_odas
    global text
    
    subprocess.call(["killall", "odaslive"])
    subprocess.call(["killall", "matrix-odas"])

    #initialise ros variables
    state = 1
    inventory = ["hammer", "pliers", "screwdriver"]
    ros_buffer = deque()
    buffer_lock = multiprocessing.Lock()

    #start doa
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/odas/bin/")
    doa_matrix = subprocess.Popen(["./matrix-odas"])
    doa_odas = subprocess.Popen(["./odaslive", "-vc", "../config/matrix-demo/matrix_voice.cfg"])
    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/")
    
    text = "yeetbot 3000, online"

def listen_wake_word():
    print('mlem')

def wait_for_input():
    delay = raw_input("input anything for YEETBot to listen; to quit input 'quit'\n")
    if delay == "quit":
        sys.exit()

def record_speech():
    print("listening...\n")
    subprocess.call(["arecord", "recording.wav", "-f", "S16_LE", "-r", "44100", "-d", "3", "-D", "hw:2,0"])

def doa_restart():
    global doa_matrix
    global doa_odas

    os.chdir("/home/pi/yeetbot/yeetbot_natural_language/pi_speech/odas/bin/")
    #kill process
    doa_odas.send_signal(signal.SIGINT)
    time.sleep(0.2)
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

def main():
    global text 
    init()
    while True:
        wait_for_input()
        if state == 0:
            record_speech()
    	    doa_restart()
            transcribe_file("recording.wav")
        elif state == 1:
            tts(text)

if __name__ == '__main__':
    main()