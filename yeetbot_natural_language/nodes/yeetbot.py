#!/usr/bin/env python
import subprocess
import io
import sys
import rospy
from yeetbot_msgs.msg import YEETBotUserResponse, YEETBotUserChoices, YEETBotState
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types

def init():
    global pub
    global state
    global inventory
    state = 0
    inventory = []
    rospy.init_node('yeet_speech', anonymous=True)
    pub = rospy.Publisher("/user_response", YEETBotUserResponse, queue_size=10)
    rospy.Subscriber("/user_choices", YEETBotUserChoices, choice_callback)
    rospy.Subscriber("/yeetbot_state", YEETBotState, state_callback) 

def choice_callback(user_choices):
    global inventory
    print "inventory updated"
    inventory = user_choices.user_options
    
def state_callback(yeetbot_state):
    global state
    print yeetbot_state.current_state
    state = yeetbot_state.current_state

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

        for result in response.results:
            sentence = result.alternatives[0].transcript.split()
            msg = YEETBotUserResponse()
            if "return" in sentence:
                msg.choice = -1
                msg.invalid_choice = False
            else:
                msg.invalid_choice = True
                for word in sentence:
                    for tool in inventory:
                        if word == tool:
                            msg.choice = inventory.index(tool)
                            msg.invalid_choice = False
            pub.publish(msg)

def main():
    init()
    while not rospy.is_shutdown():
        wait_for_input()
        record_speech()
        transcribe_file("recording.wav")

if __name__ == '__main__':
    main()
