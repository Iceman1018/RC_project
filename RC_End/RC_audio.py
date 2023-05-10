import socket
import time
import wave
import pyaudio
import numpy
import threading
from pyaudio import PyAudio
from threading import Thread


selfip=''
#HOST="s3.z100.vip"
HOST="103.45.161.236"
PORT_4=14729 #5555
PORT_5=25989 #4444

buffSize=1500
CHUNK = 441
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 22050
data=0

event=threading.Event()

def send_audio():
    global data
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
                    channels=CHANNELS,
                    rate=RATE,
                    input=True,
                    output=False,
                    frames_per_buffer=CHUNK)
    while True:
        data = stream.read(CHUNK+100)
        event.set()
    stream.stop_stream()
    stream.close()
    p.terminate()

t=Thread(target=send_audio,args=())
t.start()

tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# 链接服务器
tcp_client_socket.connect((HOST, PORT_4))
#tcp_client_socket.setblocking(False)

while True:
    if event.isSet():
        tcp_client_socket.send(data)
        event.clear()
    else:
        event.wait()
    
tcp_client_socket.close()

