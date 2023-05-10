import cv2
import sys
import os
import numpy
import socket
import struct
import time
import smbus
import pigpio
import RPi.GPIO as GPIO
import wave
import pyaudio
import sys
import subprocess
import datetime
import engine_factory
from audio_device import AudioDevice
import random

from queue import Queue
from threading import Thread
from multiprocessing import Process,Semaphore
from pyaudio import PyAudio
from ping3 import ping
from subprocess import Popen, PIPE

date=datetime.datetime.now()
date="{:0>2}".format(date.day)+"{:0>2}".format(date.month)+str(date.year)[2:]

selfip=''
#HOST="s3.z100.vip"
HOST="103.45.161.236"
PORT=6666
PORT_2=22659 #3333
PORT_3=28380 #7777
PORT_4=14729 #5555
PORT_5=25989 #4444
PORT_6=15511 #1111
PORT_7=30658 #2222

INET_TYPE=socket.AF_INET


address = 0x48  # address 器件的地址(硬件地址 由器件决定)
A0 = 0x40       # A0 器件某个端口的地址（数据存储的寄存器）
A1 = 0x41
A2 = 0x42
A3 = 0x43
bus = smbus.SMBus(1)  # 开启总线 创建一个smbus实例

buffSize=1500
CHUNK = 1024*2
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 22050


Device_Address = 0x68   # MPU6050 device address
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def send_GPS():
	client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	server_address = (HOST, PORT_2)  # 接收方 服务器的ip地址和端口号
	client_socket.connect(server_address)
	process = subprocess.Popen('sudo minicom -b 115200 -o -D /dev/ttyUSB0', shell=True, stdout=PIPE, bufsize=1, preexec_fn=os.setsid)
	while True:
		line = str(process.stdout.readline())
		index=line.find('$GNRMC')
		index2=line.find(date)
		if index!=-1:
			line=line[index+7:index2-1]
			#line="032656.000,A,3203.30931,N,11847.58698,E,2.100,2.36"
			client_socket.sendall(line.encode('utf-8'))
			print(line)
			

def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value


def read_gforce():
    global Ax_real, Ay_real, angle_e, fix_value
    MPU_Init()
    t = 0
    sum_e = 0.0
    sum_a1 = 0
    sum_a2 = 0
    while t < 100:
        gyro_x = read_raw_data(GYRO_XOUT_H)
        sum_e += gyro_x
        t += 1
        time.sleep(0.05)
    average_e = sum_e / 100
    straight = 0
    smooth = 0


    while True:
        # Read Accelerometer raw value
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_x = read_raw_data(ACCEL_XOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H) - average_e  # use 2.5ms
        # Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ay = acc_y / 16384.0
        Ax = acc_x / 16384.0
        Gz = gyro_z / 131.0
        # print(Gx)
        if signal_steer < 0.01 or signal_steer>-0.01:
            straight += 1
        else:
            straight = 0
        if straight > 2 and straight < 9:
            angle_e += Gz
        elif straight == 9:
            angle_e += Gz
            straight = 0
            if angle_e > 0.28 and fix_value>-18:
                fix_value -= 2
            elif angle_e < -0.28 and fix_value<18:
                fix_value += 2
            #print(fix_value)
            angle_e = 0

        if smooth != 4:
            sum_a1 += Ay
            sum_a2 += Ax
            smooth += 1
        else:
            sum_a1 += Ay
            sum_a2 += Ax
            Ay_real = sum_a1 / 5
            Ax_real = sum_a2 / 5
            sum_a1 = 0
            sum_a2 = 0
            smooth = 0

        time.sleep(0.005)
        #print(angle_e)

def send_gforce(sema2):
    half_voltage=130
    last_value=130
    stable=0
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_address = (HOST, PORT_7)  # 接收方 服务器的ip地址和端口号
    client_socket.connect(server_address)
    t = Thread(target=read_gforce, args=())
    t.start()

    while True:
        sema2.acquire()
        bus.write_byte(address, A1)  # 获取传感器的那个端口的数据
        value = bus.read_byte(address)
        sema2.release()
        if value==last_value:
            stable+=1
        else:
            stable=0
        if stable>5:
            half_voltage=value
        last_value=value
        result=float((value-half_voltage)*100/half_voltage)
        data='{:.2f}'.format(Ay_real)+':'+'{:.2f}'.format(Ax_real)+':'+'{:.1f}'.format(result)+':'+'{:.1f}'.format(angle_e)+':'+str(fix_value)
        client_socket.sendall(data.encode('utf-8'))
        time.sleep(0.05)



def send_delay(sema):
	tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# 链接服务器
	tcp_client_socket.connect((HOST, PORT_6))
	while True:
		second=ping(HOST)
		try:
			second='{:.3f}'.format(second)
		except Exception as e:
			continue
		try:
			tcp_client_socket.send(second.encode('utf-8'))
		except Exception as e:
			sema.release()
			
		time.sleep(0.2)

def lock_RC(sema):
	global high_acc
	global high_steer
	t=0
	while True:
		sema.acquire()
		high_acc=600
		high_steer=600
		high_cam1=600
		high_cam2=600
		print('lock')
		time.sleep(1)
		t+=1
		if t==10:
			sys.exit()


def sensor_message(sema2):
    # 创建socket
    tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # 链接服务器
    tcp_client_socket.connect((HOST, PORT_3))
    while True:
        # 提示用户输入数据
        #send_data = input("请输入要发送的数据：")
        sema2.acquire()
        bus.write_byte(address, A0)  # 获取传感器的那个端口的数据
        value = bus.read_byte(address)
        bus.write_byte(address, A0)  # 获取传感器的那个端口的数据
        value = bus.read_byte(address)
        sema2.release()
        result=value/10.28
        tcp_client_socket.send('{:.2f}'.format(result).encode("utf-8"))
        bus.write_byte(address, A1)  # 获取传感器的那个端口的数据
        value = bus.read_byte(address)
        #tcp_client_socket.send(send_data.encode("utf-8"))
        # 接收对方发送过来的数据，最大接收1024个字节
        #recvData = tcp_client_socket.recv(1024)
        #print('接收到的数据为:', recvData.decode('utf-8'))
        time.sleep(2)
    # 关闭套接字
    tcp_client_socket.close()

def control():
        global RPM
        global high_acc
        global high_steer
        global high_cam1
        global high_cam2
        global light1
        global light2
        global high_brake
        global direction
        global signal_steer
        global auto_fix
        limit=0.2
        mode=0
        acc=-0.95
        brake=-0.95
        server_socket = socket.socket(INET_TYPE, socket.SOCK_DGRAM)
        server_socket.bind((selfip,PORT))
        while True:
                '''			
                data,address=server_socket.recvfrom(buffSize)
                print(data)
                
                if len(data)!=4: #进行简单的校验，长度值是int类型，占四个字节
                        #print(data)
                        length=0
                else:
                        length=struct.unpack('i',data)[0] #长度值
                        #print(length)
                '''
                data,address=server_socket.recvfrom(buffSize)
                '''
                if length!=len(data): #进行简单的校验
                        #print(len(data))
                        continue
                '''
                data=data.decode('utf-8')
                last_acc=acc
                last_brake=brake
                acc,brake,steer,downshift,upshift,light_trigger1,light_trigger2,hat_x,hat_y,para=data.split(':')
                acc=float(acc)
                brake=float(brake)
                auto_fix=int(para)
                RPM=int(5000+3000*acc)
                if downshift!=upshift:
                        if int(downshift)==1 and mode>0:
                                mode=mode-1
                        if int(upshift)==1 and mode<3:
                                mode=mode+1
                if mode==0:
                        limit=0.08
                elif mode==1:
                        limit=0.15
                elif mode==2:
                        limit=0.3
                else:
                        limit=0.5

                if (acc>-0.95 and brake<-0.95) or (acc>-0.95 and brake>-0.95 and last_acc<-0.95) or (brake>-0.95 and acc>-0.95 and last_brake>-0.95 and last_acc>-0.95 and high_acc>0):
                        high_acc=(acc+1)*98*limit+4
                elif (brake>-0.95 and acc<-0.95) or (brake>-0.95 and acc>-0.95 and last_brake<-0.95) or (brake>-0.95 and acc>-0.95 and last_brake>-0.95 and last_acc>-0.95 and high_acc<0):
                        high_acc=-(brake+1)*90-20
                else:
                        high_acc=0
                steer=float(steer)
                high_steer=-steer*200+600
                light_trigger1=int(light_trigger1)
                light_trigger2=int(light_trigger2)
                     
                if light_trigger1==1:
                        light1=GPIO.HIGH
                else:
                        light1=GPIO.LOW

                if light_trigger2==1:
                        light2=GPIO.HIGH
                else:
                        light2=GPIO.LOW
                if steer>0.5:
                    high_cam1 = high_cam1 - int(hat_x) * 2 + (steer-0.5)*10
                elif steer<-0.5:
                    high_cam1 = high_cam1 - int(hat_x) * 2 + (steer+0.5)*10
                else:
                    high_cam1=high_cam1-int(hat_x)*2+float(steer)
                high_cam2=high_cam2+int(hat_y)*2
                #print('control:',high_acc,high_steer,light)
                signal_steer=float(steer)
                time.sleep(0.01)
   

subprocess.Popen('raspivid -vf -n -w 720 -h 480 -fps 30 -o - -t 0 -b 2000000 | nc -k -l 9999',shell=True)
time.sleep(5)
subprocess.Popen('python3 /home/pi/workspace/RC_audio.py', shell=True)
#light
GPIO.setmode(GPIO.BCM)
GPIO.setup(5, GPIO.OUT)
GPIO.setmode(GPIO.BCM)
GPIO.setup(6, GPIO.OUT)
   
pi = pigpio.pi()
pwmPin_acc = 14
pwmPin_steer = 13
pwmPin_cam2 = 21
pwmPin_cam1 = 26
pi.set_mode(pwmPin_acc, pigpio.OUTPUT)
pi.set_mode(pwmPin_steer, pigpio.OUTPUT)
pi.set_mode(pwmPin_cam1, pigpio.OUTPUT)
pi.set_mode(pwmPin_cam2, pigpio.OUTPUT)
freq = 400
div = 1000

RPM=2000

auto_fix=1
angle_e=0
fix_value=0
Ay_real=0
Ax_real=0

high_acc=0
high_steer=600
high_cam1=600
high_cam2=600
signal_steer=0
direction=1
light1=GPIO.LOW
light2=GPIO.LOW

engine = engine_factory.v_8_LS()
audio_device = AudioDevice()
stream = audio_device.play_stream(engine.gen_audio)

pi.set_PWM_frequency(pwmPin_acc, freq)#设定引脚产生的pwm波形的频率为Hz
pi.set_PWM_range(pwmPin_acc, div) 
pi.set_PWM_dutycycle(pwmPin_acc,high_acc)

pi.set_PWM_frequency(pwmPin_steer, freq)#设定引脚产生的pwm波形的频率为Hz
pi.set_PWM_range(pwmPin_steer, div) 
pi.set_PWM_dutycycle(pwmPin_steer,high_steer)

pi.set_PWM_frequency(pwmPin_cam1, freq)#设定引脚产生的pwm波形的频率为Hz
pi.set_PWM_range(pwmPin_cam1, div)
pi.set_PWM_dutycycle(pwmPin_cam1,high_cam1)

pi.set_PWM_frequency(pwmPin_cam2, freq)#设定引脚产生的pwm波形的频率为Hz
pi.set_PWM_range(pwmPin_cam2, div)
pi.set_PWM_dutycycle(pwmPin_cam2,high_cam2)

sema=Semaphore(0)
sema2=Semaphore(1)

t=Thread(target=control,args=())
t.start()

t2=Thread(target=sensor_message,args=(sema2,))
t2.start()

t3=Thread(target=lock_RC,args=(sema,))
t3.start()

t4=Thread(target=send_gforce,args=(sema2,))
t4.start()

t5=Thread(target=send_GPS,args=())
t5.start()

t6=Thread(target=send_delay,args=(sema,))
t6.start()



while True:
        if auto_fix==1:
                steer_angle=high_steer+fix_value
        else:
                steer_angle=high_steer
        pi.set_PWM_dutycycle(pwmPin_steer,steer_angle)
        pi.set_PWM_dutycycle(pwmPin_acc,600+direction*high_acc)
        pi.set_PWM_dutycycle(pwmPin_cam1,high_cam1)
        pi.set_PWM_dutycycle(pwmPin_cam2,high_cam2)
        engine.set_RPM(RPM)
        GPIO.output(5, light1)
        GPIO.output(6, light2)

