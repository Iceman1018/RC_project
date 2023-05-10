import socket
import time
import cv2
import struct
import pygame
import numpy as np
import win32gui, win32ui, win32con
from threading import Thread
from ping3 import ping
import wave
import pyaudio
import math
import subprocess

class LPF(object):
    def __init__(self,fc,Ts,start):
        self.fc=fc
        self.Ts=Ts
        self.last_out=start
    def filter(self,input_signal):
        a=self.Ts/(self.Ts+1/(2*math.pi*self.fc))
        new_out=self.last_out*(1-a)+a*input_signal
        self.last_out=new_out
        return new_out


class WindowCapture:

    # properties
    w = 0
    h = 0
    hwnd = None
    cropped_x = 0
    cropped_y = 0
    offset_x = 0
    offset_y = 0

    # constructor
    def __init__(self, window_name):
        # find the handle for the window we want to capture
        self.hwnd = win32gui.FindWindow(None, window_name)
        if not self.hwnd:
            raise Exception('Window not found: {}'.format(window_name))

        # get the window size
        window_rect = win32gui.GetWindowRect(self.hwnd)
        self.w = window_rect[2] - window_rect[0]
        self.h = window_rect[3] - window_rect[1]

        # account for the window border and titlebar and cut them off
        border_pixels = 8
        titlebar_pixels = 30
        self.w = self.w - (border_pixels * 2)
        self.h = self.h - titlebar_pixels - border_pixels
        self.cropped_x = border_pixels
        self.cropped_y = titlebar_pixels

        # set the cropped coordinates offset so we can translate screenshot
        # images into actual screen positions
        self.offset_x = window_rect[0] + self.cropped_x
        self.offset_y = window_rect[1] + self.cropped_y

    def get_screenshot(self):

        # get the window image data
        wDC = win32gui.GetWindowDC(self.hwnd)
        dcObj = win32ui.CreateDCFromHandle(wDC)
        cDC = dcObj.CreateCompatibleDC()
        dataBitMap = win32ui.CreateBitmap()
        dataBitMap.CreateCompatibleBitmap(dcObj, self.w, self.h)
        cDC.SelectObject(dataBitMap)
        cDC.BitBlt((0, 0), (self.w, self.h), dcObj, (self.cropped_x, self.cropped_y), win32con.SRCCOPY)

        # convert the raw data into a format opencv can read
        #dataBitMap.SaveBitmapFile(cDC, 'debug.bmp')
        signedIntsArray = dataBitMap.GetBitmapBits(True)
        img = np.fromstring(signedIntsArray, dtype='uint8')
        img.shape = (self.h, self.w, 4)

        # free resources
        dcObj.DeleteDC()
        cDC.DeleteDC()
        win32gui.ReleaseDC(self.hwnd, wDC)
        win32gui.DeleteObject(dataBitMap.GetHandle())

        # drop the alpha channel, or cv.matchTemplate() will throw an error like:
        #   error: (-215:Assertion failed) (depth == CV_8U || depth == CV_32F) && type == _templ.type()
        #   && _img.dims() <= 2 in function 'cv::matchTemplate'
        img = img[...,:3]

        # make image C_CONTIGUOUS to avoid errors that look like:
        #   File ... in draw_rectangles
        #   TypeError: an integer is required (got type tuple)
        # see the discussion here:
        # https://github.com/opencv/opencv/issues/14866#issuecomment-580207109
        img = np.ascontiguousarray(img)

        return img

INET_TYPE=socket.AF_INET
selfip=''
#HOST='s3.z100.vip'
HOST='103.239.247.113'
PORT = 25709
PORT_2=3333
PORT_3=7777
PORT_4 = 5555
PORT_5 = 4444
PORT_6 = 1111
PORT_7 = 2222

def capacity_refresh(voltage):
    voltage=float(voltage)
    global capacity
    if capacity==100:
        if voltage>12.5:
            capacity=100
        elif voltage>12.3:
            capacity=95
        elif voltage>11.94:
            capacity=85
        elif voltage>11.78:
            capacity=68
        elif voltage>11.66:
            capacity=58
        elif voltage>11.49:
            capacity=50
        elif voltage>11.38:
            capacity=35
        elif voltage>11.25:
            capacity=25
        elif voltage>11.15:
            capacity=18
        elif voltage>11:
            capacity=9
        else:
            capacity=0
    elif capacity==95:
        if voltage > 12.3:
            capacity = 95
        elif voltage > 11.94:
            capacity = 85
        elif voltage > 11.78:
            capacity = 68
        elif voltage > 11.66:
            capacity = 58
        elif voltage > 11.49:
            capacity = 50
        elif voltage > 11.38:
            capacity = 35
        elif voltage > 11.25:
            capacity = 25
        elif voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==85:
        if voltage > 11.94:
            capacity = 85
        elif voltage > 11.78:
            capacity = 68
        elif voltage > 11.66:
            capacity = 58
        elif voltage > 11.49:
            capacity = 50
        elif voltage > 11.38:
            capacity = 35
        elif voltage > 11.25:
            capacity = 25
        elif voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==68:
        if voltage > 11.78:
            capacity = 68
        elif voltage > 11.66:
            capacity = 58
        elif voltage > 11.49:
            capacity = 50
        elif voltage > 11.38:
            capacity = 35
        elif voltage > 11.25:
            capacity = 25
        elif voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==58:
        if voltage > 11.66:
            capacity = 58
        elif voltage > 11.49:
            capacity = 50
        elif voltage > 11.38:
            capacity = 35
        elif voltage > 11.25:
            capacity = 25
        elif voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==50:
        if voltage > 11.49:
            capacity = 50
        elif voltage > 11.38:
            capacity = 35
        elif voltage > 11.25:
            capacity = 25
        elif voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==35:
        if voltage > 11.38:
            capacity = 35
        elif voltage > 11.25:
            capacity = 25
        elif voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==25:
        if voltage > 11.25:
            capacity = 25
        elif voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==18:
        if voltage > 11.15:
            capacity = 18
        elif voltage > 11:
            capacity = 9
        else:
            capacity = 0
    elif capacity==9:
        if voltage > 11:
            capacity = 9
        else:
            capacity = 0
    else:
        capacity=0


def draw_capacity(image,capacity):
    cv2.rectangle(image,(100,80),(130,95),(255,255,255),2)
    cv2.rectangle(image,(102,82),(102+int(26*capacity/100),93),(0,255,0),-1)
    cv2.rectangle(image,(132,84),(134,91),(255,255,255),2)
    return image


def draw_gforce(image,gx,gy,g,x,y):
    cv2.circle(image, (x, y), 40, (255, 255, 255), 3)
    cv2.circle(image, (x, y), 20, (255, 255, 255), 2)
    cv2.circle(image, (int(x - 30 * gx), int(y + 30 * gy)), 7, (0, 0, 255), -1)
    cv2.putText(image, '{:.2f}'.format(g) + ' g', (x+45, y+45), font, 0.7, (255, 255, 255), 2)
    return image

def draw_current(image,current,max_current,x,y,size):
    if current<0:
        current=0
    for i in range(int(size*8)):
        cv2.ellipse(image, (x, y), (int(40 * size)+i*2, int(40 * size)+i*2), 0, 120, 121 + int(max_current / 100 * 300), (0, 0, 180),2)
    if current>100:
        cv2.ellipse(image, (x, y), (int(60 * size), int(60 * size)), 0, 123, 360, (0, 255, 255), 8)
        cv2.ellipse(image, (x, y), (int(60 * size), int(62 * size)), 0, 5, 60,(0, 0, 255), 8)
        cv2.rectangle(image, (x + int(60 * size) - 4, y), (x + int(60 * size) + 4, y + 4), (0, 0, 255), -1)
    elif current>78:
        cv2.ellipse(image, (x, y), (int(60 * size), int(60 * size)), 0, 123, 360, (0, 255, 255),8)
        cv2.ellipse(image, (x, y), (int(60 * size), int(62 * size)), 0, 5, 5+int((current-78)*55/22), (0, 0, 255), 8)
        cv2.rectangle(image, (x + int(60 * size) - 4, y), (x + int(60 * size) + 4, y + 4), (0, 0, 255), -1)
    elif current>75:
        cv2.ellipse(image, (x, y), (int(60 * size), int(60 * size)), 0, 123, 360, (0, 255, 255), 8)
        cv2.rectangle(image, (x + int(60 * size) - 4, y), (x + int(60 * size) +4 , y + 4), (0, 0, 255), -1)
    else:
        cv2.ellipse(image, (x, y), (int(60 * size), int(60 * size)), 0, 123, 124+int(240*current/75), (0, 255, 255), 8)
    cv2.putText(image, str(current) + "A", (x + int(size * 65), y + int(65 * size)), font, size * 1, (255, 255, 255),
                int(size * 3))
    cv2.putText(image, str(max_current), (x - int(size * 36), y + int(10 * size)), font, size * 1,
                (255, 255, 255), int(size * 3))
    cv2.putText(image,"A(max)" , (x - int(size * 28), y + int(60 * size)), font, size * 0.5,
                (255, 255, 255), int(size * 2))
    cv2.ellipse(image, (x, y), (int(67 * size), int(67 * size)), 0, 120, 420, (255, 255, 255), 2)
    cv2.ellipse(image, (x, y), (int(53 * size), int(53 * size)), 0, 120, 420, (255, 255, 255), 2)

    return image

def draw_speed(image,velocity,shift,direction,x,y,size):
    #a=time.time()
    velocity=int(velocity)
    rate=velocity/(shift*0.8)
    num=int(shift*0.8/10)
    cv2.ellipse(image, (x, y), (int(100*size), int(100*size)), 0, 90, 90+int(270*rate), (255, 200, 100), int(size*5))
    for i in range(num):
        angle=-math.pi/2-math.pi*3/2*i/num
        p1=math.cos(angle)
        p2=-math.sin(angle)
        cv2.line(image,(int(x+p1*90*size),int(y+p2*90*size)),(int(x+p1*110*size),int(y+p2*110*size)),(255,255,255),int(3*size))
    cv2.line(image, (x + int(size*90), y), (x + int(size*110), y),(255,255,255),int(size*3))
    cv2.putText(image, str(velocity), (x+int(size*32), y+int(size*70)), font, size*1.85, (255, 255, 255), int(size*5))
    cv2.putText(image, "km/h", (x + int(size*32), y + int(size*110)), font, size*0.8, (255, 255, 255), int(size*3))
    cv2.putText(image, "N", (x - int(size * 5), y - int(size * 50)), font, size * 0.8, (255, 255, 255), int(size * 3))
    pa1=math.cos(math.pi*(direction-90)/180)
    pa2=math.sin(math.pi*(direction-90)/180)
    cv2.arrowedLine(image, (x -int(size*5*pa1),y-int(size*5*pa2)),(x +int(size*25*pa1),y+int(size*25*pa2)),(0,0,255),int(size*3),tipLength=0.4)
    if gps==1:
        cv2.putText(screenshot, 'GPS: OK', (1210, 200), font, 0.7, (0, 255, 0), 2)
    else:
        cv2.putText(screenshot, 'GPS: off-line', (1210, 200), font, 0.7, (0, 0, 255), 2)
    #print(time.time()-a)

    return image


def get_GPS():
    global gps,direction,velocity,longitude,latitude
    server_socket = socket.socket(INET_TYPE, socket.SOCK_DGRAM)
    server_socket.bind((selfip, PORT_2))
    while True:
        data, address = server_socket.recvfrom(100)
        data=data.decode("utf-8")
        print(data)
        try:
            d0, d1, d2, d3, d4, d5, d6, d7 = data.split(",")
        except Exception as e:
            continue
        else:
            print(d1,d6)
            if d1=="V":
                gps=0
            else:
                gps=1
                velocity=float(d6)*1.852
                direction=float(d7)


def get_gforce():
    global gy,gx,last_gx,g,current,max_current
    server_socket = socket.socket(INET_TYPE, socket.SOCK_DGRAM)
    server_socket.bind((selfip, PORT_7))
    #lpf1 = LPF(5, 0.05, 0)
    #lpf2 = LPF(5, 0.05, 0)
    while True:
        data, address=server_socket.recvfrom(30)
        data=data.decode("utf-8")
        gy,gx,current,angle_e,fix=data.split(":")
        print(angle_e)
        gy=float(gy)
        gx=float(gx)
        #gy=lpf1.filter(float(gy))
        #gx=lpf2.filter(float(gx))
        g=math.sqrt(gy**2+gx**2)
        if float(current)>float(max_current):
            max_current=current
        '''
        if gps==1:
            velocity=velocity+(gx+last_gx)*1.8*0.05
        '''
        last_gx=gx
        #print(gy,gx)
        time.sleep(0.03)



def get_delay():
    global server_delay
    global server_delay2
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server_socket.bind((selfip, PORT_6))
    tcp_server_socket.listen(128)
    client_socket, clientAddr = tcp_server_socket.accept()

    t=0
    while True:
        data = client_socket.recv(5)
        ms=int(float(data.decode("utf-8"))*1000)
        t=t+1
        if t==10:
            try:
                ms2 = int(ping('103.45.161.236') * 1000)
                server_delay=ms
                server_delay2=ms2
            except Exception as e:
                t=t-1
            else:
                t=0




def send_audio():

    time.sleep(10)
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server_socket.bind((selfip, PORT_4))
    tcp_server_socket.listen(128)
    client_socket, clientAddr = tcp_server_socket.accept()
    while True:
        if phone==1:
            p = pyaudio.PyAudio()
            stream = p.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=RATE,
                            input=True,
                            output=False,
                            frames_per_buffer=CHUNK)
            print("send_audio")
            while phone==1:
                data = stream.read(CHUNK)
                client_socket.send(data)
            stream.stop_stream()
            stream.close()
            p.terminate()
        else:
            time.sleep(1)


def get_audio():
    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT,
    channels=CHANNELS,
    rate=RATE,
    input=False,
    output=True,
    frames_per_buffer=441)

    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server_socket.bind((selfip, PORT_5))
    tcp_server_socket.listen(128)
    #val = struct.pack("Q", 150)
    #tcp_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVTIMEO, val)
    client_socket, clientAddr = tcp_server_socket.accept()
    #client_socket.setblocking(False)
    print("get_audio")
    while True:
        data = client_socket.recv(15000)
        data=np.frombuffer(data,dtype=np.int16)
        data=data*10
        data=data.tobytes()
        stream.write(data)
        #cv2.waitKey(3)


def get_sensor():
    global voltage
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tcp_server_socket.bind((selfip,PORT_3))

    # 使用socket创建的套接字默认的属性是主动的，使用listen将其变为被动的，这样就可以接收别人的链接了
    tcp_server_socket.listen(128)

    # 如果有新的客户端来链接服务器，那么就产生一个新的套接字专门为这个客户端服务
    # client_socket用来为这个客户端服务
    # tcp_server_socket就可以省下来专门等待其他新客户端的链接
    client_socket, clientAddr = tcp_server_socket.accept()

    while True:
        # 接收对方发送过来的数据
        recv_data = client_socket.recv(1024)  # 接收1024个字节
        voltage=recv_data.decode('utf-8')
        #print('接收到的数据为:', recv_data.decode('utf-8'))
        # 发送一些数据到客户端
        #client_socket.send("thank you !".encode('utf-8'))

        if recv_data.decode('utf-8') == "quit":
            # 关闭为这个客户端服务的套接字，只要关闭了，就意味着为不能再为这个客户端服务了，如果还需要服务，只能再次重新连接
            client_socket.close()
            break


command =['mplayer','-fps','200','-demuxer','h264es','ffmpeg://tcp://103.239.247.113:31741']
pipe = subprocess.Popen(command)
time.sleep(1)
wincap = WindowCapture('MPlayer - The Movie Player')

hat_x=0
hat_y=0
imu_control=1
imu_control_trigger=0
acc_control=1
steer_bias=0
lock_bias=0
control_para=2
capacity=100
gps=0
direction=0
velocity=0
longitude=0.0
latitude=0.0
gx=0
last_gx=0
gy=0
g=0
acc=0
brake=0
steer=0
phone=0
light=0
light1=0
light2=0
backcar=0
full=0
loss=0
loss_rate="0%"
last_time=time.time()
fps_t=0
sum_time=0.5
voltage='13'
current='0.0'
max_current='0.0'
shift=25
server_delay=0
server_delay2=0
font = cv2.FONT_HERSHEY_SIMPLEX

buffSize=1500
CHUNK = 441
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 22050


pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
# 手柄对象初始化
joystick.init()
clock = pygame.time.Clock()

t=Thread(target=get_sensor,args=())
t.start()
t2=Thread(target=get_audio,args=())
t2.start()
#t3=Thread(target=send_audio,args=())
#t3.start()
t4=Thread(target=get_delay,args=())
t4.start()
t5=Thread(target=get_gforce,args=())
t5.start()
t6=Thread(target=get_GPS,args=())
t6.start()
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (HOST, PORT)  # 接收方 服务器的ip地址和端口号
client_socket.connect(server_address)
b1=0
b12=0
b2=0
b22=0
b3=0
b32=0
b4=0
b42=0
b5=0
b52=0
b6=0
b62=0
b7=0
b72=0
b8=0
b82=0
b9=0
b92=0
acc_last=-1
time_last=time.time()
while(True):
    start_t=time.time()
    pygame.event.get()
    acc = 0-joystick.get_axis(1)
    brake = 0-joystick.get_axis(2)
    # print("accelerator: " + str(acc))
    # print("brake: " + str(brake))
    steer = joystick.get_axis(0)
    # print("steering :" + str(steer))

    #hat_x,hat_y=joystick.get_hat(0)

    b12 = joystick.get_button(5)
    if b1==0 and b12==1:
        downshift=1
    else:
        downshift=0
    b1=b12
    b22 = joystick.get_button(4)
    if b2 == 0 and b22 == 1:
        upshift = 1
    else:
        upshift = 0
    b2=b22
    b32 = joystick.get_button(7)
    if b3 == 0 and b32 == 1:
        phone_trigger = 1
    else:
        phone_trigger = 0
    b3=b32
    b42 = joystick.get_button(6)
    if b4 == 0 and b42 == 1:
        light_trigger = 1
    else:
        light_trigger = 0
    b4 = b42

    b52 = joystick.get_button(0)
    if b5 == 0 and b52 == 1:
        acc_control_trigger = 1
    else:
        acc_control_trigger = 0
    b5 = b52

    b62 = joystick.get_button(1)
    if b6 == 0 and b62 == 1:
        lock_bias = 1
    else:
        lock_bias = 0
    b6 = b62

    b72 = joystick.get_button(3)
    if b7 == 0 and b72 ==1:
        imu_control_trigger=1
    else:
        imu_control_trigger=0
    b7=b72

    b82 = joystick.get_button(8)
    if b8 == 0 and b82 == 1:
        light_trigger1 = 1
    else:
        light_trigger1 = 0
    b8 = b82

    b92 = joystick.get_button(9)
    if b9 == 0 and b92 == 1:
        light_trigger2 = 1
    else:
        light_trigger2 = 0
    b9 = b92

    mode = -1
    for i in range(4):
        if joystick.get_button(i) == 1:
            mode = i
            break
    if phone_trigger == 1:
        phone = (phone + 1) % 2
    if light_trigger == 1:
        light = (light + 1) % 2
    if light_trigger1 == 1:
        light1 = (light1 + 1) % 2
    if light_trigger2 == 1:
        light2 = (light2 + 1) % 2
    if acc_control_trigger==1:
        acc_control=(acc_control+1)%2
    if imu_control_trigger==1:
        imu_control=(imu_control+1)%2
    if lock_bias==1:
        steer_bias=steer+steer_bias

    if downshift!=upshift:
        if downshift == 1 and shift >25:
            shift=shift-25
            control_para=control_para*1.8
        if upshift == 1 and shift <100:
            shift=shift+25
            control_para=control_para/1.8
            acc_last=(acc_last+1)/2-1

    if acc_control==1:
        time_now = time.time()
        acc_fix = acc_last + (time_now - time_last) * control_para
        time_last=time_now
        #print(acc,acc_fix)
        if acc > acc_fix:
            acc = acc_fix
        acc_last=acc

    send_data = '{:.2f}'.format(acc) + ":" + '{:.2f}'.format(brake) + ":" + '{:.2f}'.format(-steer-steer_bias) + ":" + str(downshift) + ":" + str(upshift) + ":" + str(light1)+ ":" + str(light2)+ ":"+str(imu_control)
    # send_data = str(acc) + ":" + str(brake) + ":" + str(steer)
    # client_socket.sendall(struct.pack('i', len(send_data.encode('utf-8'))))  # 发送编码后的字节长度，这个值不是固定的
    client_socket.sendall(send_data.encode('utf-8'))
    print(time.time()-start_t)
    # print("send")
    joystick_count = pygame.joystick.get_count()
    capacity_refresh(voltage)
    try:
        # get an updated image of the game
        screenshot = wincap.get_screenshot()
        screenshot = cv2.resize(screenshot, (1440, 960))
        #screenshot = cv2.flip(screenshot, 1)
        screenshot = cv2.flip(screenshot, 0)
        fps_t = fps_t + 1
        if fps_t == 10:
            now = time.time()
            sum_time = now - last_time
            last_time = now
            fps_t = 0

        cv2.putText(screenshot, 'throttle limit: ' + str(shift)+"%", (1210, 650), font, 0.7, (255, 0, 255), 2)
        cv2.putText(screenshot, 'fps: ' + str(int(10 / sum_time)), (1210, 50), font, 0.7, (255, 255, 255), 2)
        if server_delay>200:
            cv2.putText(screenshot, 'RC: ' + str(server_delay) + 'ms', (1210, 100), font, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(screenshot, 'RC: ' + str(server_delay)+'ms', (1210, 100), font, 0.7, (0, 255, 0), 2)
        if server_delay2>200:
            cv2.putText(screenshot, 'PC: ' + str(server_delay2) + 'ms', (1210, 150), font, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(screenshot, 'PC: ' + str(server_delay2)+'ms', (1210, 150), font, 0.7, (0, 255, 0), 2)

        cv2.rectangle(screenshot, (100, 900), (150, 750), (0, 0, 255), 4)
        cv2.rectangle(screenshot, (1290, 900), (1340, 750), (0, 0, 255), 4)
        cv2.rectangle(screenshot, (104, 896), (146, int(896 - 71 * (brake + 1))), (100, 100, 100), -1)
        cv2.rectangle(screenshot, (1294, 896), (1336, int(896 - 71 * (acc + 1))), (100, 100, 100), -1)

        cv2.putText(screenshot, 'Bias:'+'{:.3f}'.format(steer_bias), (1210, 350), font, 0.7, (255, 255, 255), 2)
        if acc_control==1:
            cv2.putText(screenshot, 'TC:on ', (1210, 450), font, 0.7, (255, 255, 255), 2)
        else:
            cv2.putText(screenshot, 'TC:off ' , (1210, 450), font, 0.7, (255, 255, 255), 2)
        if imu_control==1:
            cv2.putText(screenshot, 'imu fix:on ', (1210, 400), font, 0.7, (255, 255, 255), 2)
        else:
            cv2.putText(screenshot, 'imu fix:off ', (1210, 400), font, 0.7, (255, 255, 255), 2)
        if light1==1:
            cv2.putText(screenshot, 'low beam:on ', (1210, 500), font, 0.7, (255, 255, 0), 2)
        else:
            cv2.putText(screenshot, 'low beam:off ' , (1210, 500), font, 0.7, (255, 255, 255), 2)
        if light2==1:
            cv2.putText(screenshot, 'high beam:on ', (1210, 550), font, 0.7, (255, 255, 0), 2)
        else:
            cv2.putText(screenshot, 'high beam:off ' , (1210, 550), font, 0.7, (255, 255, 255), 2)
        if phone==1:
            cv2.putText(screenshot, 'phone:on ', (1210, 600), font, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(screenshot, 'phone:off ' , (1210, 600), font, 0.7, (255, 255, 255), 2)
        if backcar==1:
            cv2.putText(screenshot, 'BACK', (1210, 450), font, 0.7, (0, 0, 255), 3)

        if float(voltage)<11:
            cv2.putText(screenshot, 'voltage: ' + voltage + ' V', (100, 50), font, 0.7, (0, 0, 255), 2)
            cv2.line(screenshot,(20,67),(50,17),(0,0,255),2)
            cv2.line(screenshot, (80, 67), (50, 17), (0, 0, 255), 2)
            cv2.line(screenshot, (80, 67), (20, 67), (0, 0, 255), 2)

            cv2.line(screenshot, (32, 59), (50, 29), (0, 0, 255), 2)
            cv2.line(screenshot, (68, 59), (50, 29), (0, 0, 255), 2)
            cv2.line(screenshot, (68, 59), (32, 59), (0, 0, 255), 2)
            cv2.putText(screenshot, 'warning', (10, 90), font, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(screenshot, 'voltage: ' + voltage + ' V', (100, 50), font, 0.7, (100, 255, 100), 2)
        draw_speed(screenshot,velocity,shift,direction,140,400,0.7)
        draw_current(screenshot,float(current),float(max_current),125,600,0.7)
        draw_gforce(screenshot,gx,gy,g,250,540)
        draw_capacity(screenshot,capacity)

        cv2.imshow('RC_control', screenshot)

        # press 'q' with the output window focused to exit.
        # waits 1 ms every loop to process key presses
        if cv2.waitKey(1) == ord('q'):
            cv2.destroyAllWindows()
            break
    except Exception as e:
        time.sleep(0.03)
