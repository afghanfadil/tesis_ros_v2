#=========Obstacle Avoidance of Autonomous Truck Trailer Script===========
#Lab ICoDeS, Teknik Fisika, ITB, 2020
#13316008 I Komang Agus Jony Wirawan
#13316028 Indah Radityo Putri
#======================================================================
from ev3dev.ev3 import *
from time import sleep
from time import time
from threading import Thread
import math
import serial
import sys
import struct
import csv
import termios, atexit
from select import select
import decimal

def sqrt(n):
    assert n > 0
    with decimal.localcontext() as ctx:
        ctx.prec += 2 # increase precision to minimize round off error
        x, prior = decimal.Decimal(n), None
        while x != prior: 
            prior = x
            x = (x + n/x) / 2 # quadratic convergence 
    return +x # round in a global context

class KBHit:
    def __init__(self):
        '''Creates a KBHit object that you can call to do various keyboard things.
        '''
        if os.name == 'nt':
            pass
        else:
            # Save the terminal settings
            self.fd = sys.stdin.fileno()
            self.new_term = termios.tcgetattr(self.fd)
            self.old_term = termios.tcgetattr(self.fd)
            # New terminal setting unbuffered
            self.new_term[3] = (self.new_term[3] & ~termios.ICANON & ~termios.ECHO)
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.new_term)
            # Support normal-terminal reset at exit
            atexit.register(self.set_normal_term)
    def set_normal_term(self):
        ''' Resets to normal terminal.  On Windows this is a no-op.
        '''
        if os.name == 'nt':
            pass
        else:
            termios.tcsetattr(self.fd, termios.TCSAFLUSH, self.old_term)
    def getch(self):
        ''' Returns a keyboard character after kbhit() has been called.
            Should not be called in the same program as getarrow().
        '''
        s = ''
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        else:
            return sys.stdin.read(1)
    def getarrow(self):
        ''' Returns an arrow-key code after kbhit() has been called. Codes are
        0 : up
        1 : right
        2 : down
        3 : left
        Should not be called in the same program as getch().
        '''
        if os.name == 'nt':
            msvcrt.getch() # skip 0xE0
            c = msvcrt.getch()
            vals = [72, 77, 80, 75]
        else:
            c = sys.stdin.read(3)[2]
            vals = [65, 67, 66, 68]
        return vals.index(ord(c.decode('utf-8')))
    def kbhit(self):
        ''' Returns True if keyboard character was hit, False otherwise.
        '''
        if os.name == 'nt':
            return msvcrt.kbhit()
        else:
            dr,dw,de = select([sys.stdin], [], [], 0)
            return dr != []

def signum(x):              #Define Sign Function
    if x > 0:
        return 1
    elif x < 0:
        return -1
    elif x == 0:
        return 0
    else:
        return x

def sinc(x):                #Define Sinc Function
    if x == 0:
        return 1
    if x != 0:
        return math.sin((math.pi*(x)/180)/(math.pi*(x)/180))

#Defining Parameters and Variables
#======================Start Defining=================================

#Initial State Statement
Dat1 = [0,0,0,0,0,0,0,0,0]
Dat2 = [0,0]

kb = KBHit()

#Input initial value (Change if needed)
x1 = 0
x2 = 4
y1 = 0
y2 = 1
V1s= 1.54
V2s = 2.54
xoffset = 136
yoffset = -204
r = 0.5
cnt = 6000
lidar_to_head_dist = 0.26                    #distance from lidar (front side) to head (front side)

obj = 2*r + lidar_to_head_dist               #object distance before changing path, do not change
grads = V2s - V1s

#Initial value of variable
xp = 0
xp2 = 0
xp3 = 0
xp4 = 0
yp = 0
lh = 0.145          #length of head truck in meter
lt = 0.313          #length of trailer in meter
ea = 0
vh_max = 0.03*1100/63               #Maximum value of head velocity in m/s, 0.03 because LiDAR has huge delay time
vh1 = vh_max*0.5                    #Head velocity
MagX = 0            #Magnetometer X
MagY = 0            #Magnetometer Y
Qh = 0              #the angle of head truck to x axis
Qt = 0              #the angle of trailer to x axis
Qht = 0             #Angle at joint between head and trailer
Qs = 0              #steering angle
Qd = ea             #desired angle
Qj = 0              #truck joint angle
Vs = 0              #Voltage of Pot in Steer
Vj = 0              #Voltage of Pot in Joint
dh = 0              #the difference angle between head and trailer
dh_d = 0            #desired angle between head and trailer
dQs = 0
xh_e = 0            #error of x position (head)
yh_e = 0            #error of y position (head)
xt_e = 0            #error of x position (trailer)
yt_e = 0            #error of y position (trailer)
Qh_e = 0            #error of head truck angle
Qt_e = 0            #error of trailer angle
Qht_e = 0           #error of angle between head and trailer
h_eh = 0            #error distance of head (to desired path)
h_et = 0            #error distance of trailer (to desired path)
Qt_ee = 0
Qh_ee = 0
kh_eh = 1           #Coefficient of head distance error
kQh_e = 1           #Coefficient of head angle error
kh_et= 1            #Coefficient of trailer distance error
kQt_e = 1           #Coefficient of trailer angle error
psi = 0

kondisi = '1'
line = '1'           #right lane
i = 0
#========================End Defining=================================

#Serial communication Arduino Mega 2560 - EV3
arduino1 = serial.Serial(port='/dev/tty_ev3-ports:in1', baudrate=9600, timeout=0.01)    #arduino1 connect with two marvelmind(s)
ev3_port1 = LegoPort('in1')
ev3_port1.mode = 'other-uart'
arduino2 = serial.Serial(port='/dev/tty_ev3-ports:in2', baudrate=57600, timeout=0.01)   #arduino2 connect with LiDAR
ev3_port2 = LegoPort('in2')
ev3_port2.mode = 'other-uart'
#EV3 Related Part
m = MediumMotor('outC')     #Steering Motor
mr = LargeMotor('outA')     #Motor for Right Wheel
ml = LargeMotor('outB')     #Motor for Left Wheel
print("Kalibrasi Steering")
m.run_to_abs_pos(position_sp = 0, speed_sp = 500)
m.stop_action = 'brake'
mr.stop_action = 'brake'
ml.stop_action = 'brake'
sleep(3)
print("posisi awal = " + str(m.position))

j = 0
while j < 10 :
    arduino1.readline().strip().decode()
    arduino2.readline().strip().decode()
    j = j + 1

#===========================Autonomous Process==================================
#Create different .csv file for each condition, DO NOT USE THE SAME FILE or data won't be written correctly
fileCsv1 = open("data_lurus.csv", "w")                  #to write kondisi 1, 4 and 7
writer1 = csv.writer(fileCsv1, quoting=csv.QUOTE_NONE)
fileCsv2 = open("data_belok2.csv", "w")                 #to write kondisi 2
writer2 = csv.writer(fileCsv2, quoting=csv.QUOTE_NONE)
fileCsv3 = open("data_belok3.csv", "w")                 #to write kondisi 3
writer3 = csv.writer(fileCsv3, quoting=csv.QUOTE_NONE)
fileCsv5 = open("data_belok5.csv", "w")                 #to write kondisi 5
writer5 = csv.writer(fileCsv5, quoting=csv.QUOTE_NONE)
fileCsv6 = open("data_belok6.csv", "w")                 #to write kondisi 6
writer6 = csv.writer(fileCsv6, quoting=csv.QUOTE_NONE)
counting = 0
counting1 = 0
counting2 = 0
counting3 = 0
while True :
    try :
        t = time()
        data1 = arduino1.readline().strip().decode()
        data2 = arduino2.readline().strip().decode()
        datas1 = data1.split('\t')
        datas2 = data2.split('\t')
        lgt1 = len(datas1)
        lgt2 = len(datas2)
        if lgt1 == 9 :
            Dat1[0] = datas1[0]    #Steering Voltage
            Dat1[1] = datas1[1]    #Magnetometer X Trailer
            Dat1[2] = datas1[2]    #Magnetometer Y Trailer
            Dat1[3] = datas1[3]    #Magnetometer X Head
            Dat1[4] = datas1[4]    #Magnetometer Y Head
            Dat1[5] = datas1[5]    #Trailer X position in mm
            Dat1[6] = datas1[6]    #Trailer Y position in mm
            Dat1[7] = datas1[7]    #Head X position in mm
            Dat1[8] = datas1[8]    #Head Y position in mm
        if lgt2 == 2 :
            Dat2[0] = datas2[0]    #Angle in degree
            Dat2[1] = datas2[1]    #Distance in mm
        Vs = float(Dat1[0])
        MagXt = float(Dat1[1]) - xoffset
        MagYt = float(Dat1[2]) - yoffset
        MagXh = float(Dat1[3])
        MagYh = float(Dat1[4])
        xt = float(Dat1[5])/1000
        yt = float(Dat1[6])/1000
        xh = float(Dat1[7])/1000
        yh = float(Dat1[8])/1000
        angle1 = float(Dat2[0])
        distance1 = float(Dat2[1])/1000
        if angle1 > 300 and angle1 < 360:
            angle = 360 - angle1
            distance = distance1*math.cos(angle*math.pi/180)
        elif angle1 > 0 and angle1 < 60:
            distance = distance1*math.cos(angle1*math.pi/180)
        print(distance)

        Qt = math.atan2(MagYt, MagXt) * 180 / math.pi 
        if Qt < 0 :
            Qt = Qt + 360
        Qs = -36 + (Vs - V1s) * 72 / (grads) #Voltage to Angle converter (steer)
        if abs(Qs) > 36 :
            Qs = signum(Qs) * 36
        Qh = math.atan2(MagYh, MagXh) * 180 / math.pi 
        if Qh < 0 :
            Qh = Qh + 360
       
        #======================Controller Part===================================
        if (kondisi == '1') and (distance != 0) :
            print("kondisi 1, line 1")
            if (xh < x2) and (distance >= obj):
                h_et = yt - y1
                h_eh = yh - y1
                Qd = 0
                v_ev3 = vh1*1000*100*63/(1100*250)*10
                mr.run_forever(speed_sp = int(-v_ev3))
                ml.run_forever(speed_sp = int(-v_ev3))
                m.run_to_rel_pos(position_sp = int(Qd), speed_sp = 500)
            else :
                xp = xh
                yp = r
                xp2 = xh + obj
                kondisi = '2'
                Qd = 90
        elif kondisi == '2' :
            print("kondisi 2", xp )
            if yh < yp :
                Qd = math.atan(-(xh-xp)/(yh-yp))*180/math.pi
                m.run_to_rel_pos(position_sp = int(Qd), speed_sp = 500)
                h_et = sqrt((xt-xp)**2+(yt-yp)**2)-r
                h_eh = sqrt((xh-xp)**2+(yh-yp)**2)-r
            else :
                kondisi = '3'
                Qd = 90
        elif kondisi == '3' :
            print("kondisi 3", xp2)
            if xh < xp2 :
                Qd = math.atan(-(xh-xp2)/(yh-yp))*180/math.pi
                m.run_to_rel_pos(position_sp = int(-Qd), speed_sp = 500)
                h_et = sqrt((xt-xp2)**2+(yt-yp)**2)-r
                h_eh = sqrt((xh-xp2)**2+(yh-yp)**2)-r
            else :
                kondisi = '4'
                Qd = 0
                line = '2'
        elif kondisi == '4' :
            print("kondisi 4, line 2")
            if (xh < (x2-r)) and (distance >= obj):
                h_et = y2 - yt
                h_eh = y2 - yt
                Qd = 0
            elif (xh >= (x2-r)) :   #The end of line2
                print("stop")
                # m.stop()
                # mr.stop()
                # ml.stop()
            else:
                xp3 = xh
                yp = r
                xp4 = xh + obj - 0.5
                kondisi='5'
                Qd= 270
        elif kondisi == '5':
            print("kondisi 5", xp3)
            if yh > yp :
                Qd  = math.atan(-(xh-xp3)/(yh-yp))*180/math.pi
                m.run_to_rel_pos(position_sp = int(Qd), speed_sp = 500)
                h_et = sqrt((xt-xp3)**2+(yt-yp)**2)-r
                h_eh = sqrt((xh-xp3)**2+(yh-yp)**2)-r
            else :
                kondisi='6'
                Qd  = 270
        elif kondisi == '6' :
            print("kondisi 6", xp4)
            if xh < xp4 :
                Qd  = math.atan(-(xh-xp4)/(yh-yp))*180/math.pi
                m.run_to_rel_pos(position_sp = int(-Qd), speed_sp = 500)
                h_et = sqrt((xt-xp4)**2+(yt-yp)**2)-r
                h_eh = sqrt((xh-xp4)**2+(yh-yp)**2)-r
            else:
                kondisi='7'
                Qd  = 0
                line= '1'
        elif kondisi == '7' :
            print("kondisi 7, line 1")
            if (xh < (x2-r)) :
                h_et  = yt  - y1
                h_eh  = yh  - y1
                Qd = 0
            else:               #The end of line 1
                kondisi='8'
                print("stop")
                # m.stop()
                # mr.stop()
                # ml.stop()
                    
        Qh_e = abs((Qh - Qd) * math.pi / 180)
        Qt_e = abs((Qt - Qd) * math.pi / 180)
        Qh_ee = (Qh - Qd) * math.pi / 180
        Qt_ee = (Qt - Qd) * math.pi / 180

        #Velocity Control Function
        if kondisi == '1' :
            vh1 = vh_max / (1 + 0.8 * kQh_e * abs(Qt_e) + 0.8 * kQh_e * abs(Qht_e) + 0.8*kh_eh * abs(h_et))
        else :
            vh1 = vh_max / (1 + 0.8 * kQh_e * abs(Qh_e) + 0.8 * kh_eh * abs(h_eh))

        #Steering Angle Control Function
        Qht_e = Qh_e - Qt_e
        if abs(Qht_e) >= math.pi/5 :
            Qht_e = signum(Qht_e) * math.pi/5
        else :
            Qht_e = Qht_e
        psi1 = -math.atan(lt/((vh1)*math.cos(dQs))*(0.01*kQt_e*Qt_e + 0.01*(vh1)*h_et*math.cos(Qht_e)*sinc(Qt_e/math.pi)))
        psidot = psi1 - psi
        z = psi1 - Qht_e
        dQs = math.atan(lh/(vh1)*(0.01*kQh_e*z + (vh1)/lt*math.sin(Qht_e)+psidot))
        psi = psi1

        if abs(dQs) >= 36/180 * math.pi :
            dQs = signum(dQs) * 36 * math.pi / 180
        else :
            dQs = dQs
        drj = (dQs * 180 / math.pi)
        
        belok = [kondisi, xh, yh, xt, yt, distance, Qh, Qt, h_eh, h_et, Qh_e, Qh_ee, Qt_e, Qt_ee, Qht_e, psi1, vh1, v_ev3, Qd, dQs, pos, drj, Qs, ei, ep, speed_drj]
        writer1.writerow(belok)

        if kondisi == '2':
            while counting < cnt:
                counting = counting + 1
                data1 = arduino1.readline().strip().decode()
                # data2 = arduino2.readline().strip().decode()
                datas1 = data1.split('\t')
                # datas2 = data2.split('\t')
                lgt1 = len(datas1)
                # lgt2 = len(datas2)
                if lgt1 == 9 :
                    Dat1[0] = datas1[0]    #Steering Voltage
                    Dat1[1] = datas1[1]    #Magnetometer X Trailer
                    Dat1[2] = datas1[2]    #Magnetometer Y Trailer
                    Dat1[3] = datas1[3]    #Magnetometer X Head
                    Dat1[4] = datas1[4]    #Magnetometer Y Head
                    Dat1[5] = datas1[5]    #Trailer X position in mm
                    Dat1[6] = datas1[6]    #Trailer Y position in mm
                    Dat1[7] = datas1[7]    #Head X position in mm
                    Dat1[8] = datas1[8]    #Head Y position in mm
                # if lgt2 == 2 :
                #     Dat2[0] = datas2[0]    #Angle in degree
                #     Dat2[1] = datas2[1]    #Distance in mm
                Vs = float(Dat1[0])
                MagXt = float(Dat1[1]) - xoffset
                MagYt = float(Dat1[2]) - yoffset
                MagXh = float(Dat1[3])
                MagYh = float(Dat1[4])
                xt = float(Dat1[5])/1000
                yt = float(Dat1[6])/1000
                xh = float(Dat1[7])/1000
                yh = float(Dat1[8])/1000
                # angle1 = float(Dat2[0])
                # distance1 = float(Dat2[1])/1000
                # if angle1 > 300 and angle1 < 360:
                #     angle = 360 - angle1
                #     distance = distance1*math.cos(angle*math.pi/180)
                # elif angle1 > 0 and angle1 < 60:
                #     distance = distance1*math.cos(angle1*math.pi/180)
                angle = 0           #doesn't need angle when changing lane
                distance = 0        #doesn't need distance when changing lane
                Qt = math.atan2(MagYt, MagXt) * 180 / math.pi  #64,130 offset magnetometer y dan x, 7.11.. bias
                if Qt < 0 :
                    Qt = Qt + 360
                Qs = -36 + (Vs - V1s) * 72 / (grads) #Voltage to Angle converter (steer)
                if abs(Qs) > 36 :
                    Qs = signum(Qs) * 36
                Qh = math.atan2(MagYh, MagXh) * 180 / math.pi 
                if Qh < 0 :
                    Qh = Qh + 360
                belok = [kondisi, xh, yh, xt, yt, distance, Qh, Qt, h_eh, h_et, Qh_e, Qh_ee, Qt_e, Qt_ee, Qht_e, psi1, vh1, v_ev3, Qd, dQs, pos, drj, Qs, ei, ep, speed_drj]
                writer2.writerow(belok)
        
        elif kondisi == '3':
            while counting1 < cnt:
                counting1 = counting1 + 1
                data1 = arduino1.readline().strip().decode()
                # data2 = arduino2.readline().strip().decode()
                datas1 = data1.split('\t')
                # datas2 = data2.split('\t')
                lgt1 = len(datas1)
                # lgt2 = len(datas2)
                if lgt1 == 9 :
                    Dat1[0] = datas1[0]    #Steering Voltage
                    Dat1[1] = datas1[1]    #Magnetometer X Trailer
                    Dat1[2] = datas1[2]    #Magnetometer Y Trailer
                    Dat1[3] = datas1[3]    #Magnetometer X Head
                    Dat1[4] = datas1[4]    #Magnetometer Y Head
                    Dat1[5] = datas1[5]    #Trailer X position in mm
                    Dat1[6] = datas1[6]    #Trailer Y position in mm
                    Dat1[7] = datas1[7]    #Head X position in mm
                    Dat1[8] = datas1[8]    #Head Y position in mm
                # if lgt2 == 2 :
                #     Dat2[0] = datas2[0]    #Angle in radian
                #     Dat2[1] = datas2[1]    #Distance in mm
                Vs = float(Dat1[0])
                MagXt = float(Dat1[1]) - xoffset
                MagYt = float(Dat1[2]) - yoffset
                MagXh = float(Dat1[3])
                MagYh = float(Dat1[4])
                xt = float(Dat1[5])/1000
                yt = float(Dat1[6])/1000
                xh = float(Dat1[7])/1000
                yh = float(Dat1[8])/1000
                # angle1 = float(Dat2[0])
                # distance1 = float(Dat2[1])/1000
                # if angle1 > 300 and angle1 < 360:
                #     angle = 360 - angle1
                #     distance = distance1*math.cos(angle*math.pi/180)
                # elif angle1 > 0 and angle1 < 60:
                #     distance = distance1*math.cos(angle1*math.pi/180)
                angle = 0
                distance = 0
                Qt = math.atan2(MagYt, MagXt) * 180 / math.pi  #64,130 offset magnetometer y dan x, 7.11.. bias
                if Qt < 0 :
                    Qt = Qt + 360
                Qs = -36 + (Vs - V1s) * 72 / (grads) #Voltage to Angle converter (steer)
                if abs(Qs) > 36 :
                    Qs = signum(Qs) * 36
                Qh = math.atan2(MagYh, MagXh) * 180 / math.pi 
                if Qh < 0 :
                    Qh = Qh + 360
                belok = [kondisi, xh, yh, xt, yt, distance, Qh, Qt, h_eh, h_et, Qh_e, Qh_ee, Qt_e, Qt_ee, Qht_e, psi1, vh1, v_ev3, Qd, dQs, pos, drj, Qs, ei, ep, speed_drj]
                writer3.writerow(belok)

        elif kondisi == '5':
            while counting2 < cnt:
                counting2 = counting2 + 1
                data1 = arduino1.readline().strip().decode()
                # data2 = arduino2.readline().strip().decode()
                datas1 = data1.split('\t')
                # datas2 = data2.split('\t')
                lgt1 = len(datas1)
                # lgt2 = len(datas2)
                if lgt1 == 9 :
                    Dat1[0] = datas1[0]    #Steering Voltage
                    Dat1[1] = datas1[1]    #Magnetometer X Trailer
                    Dat1[2] = datas1[2]    #Magnetometer Y Trailer
                    Dat1[3] = datas1[3]    #Magnetometer X Head
                    Dat1[4] = datas1[4]    #Magnetometer Y Head
                    Dat1[5] = datas1[5]    #Trailer X position in mm
                    Dat1[6] = datas1[6]    #Trailer Y position in mm
                    Dat1[7] = datas1[7]    #Head X position in mm
                    Dat1[8] = datas1[8]    #Head Y position in mm
                # if lgt2 == 2 :
                #     Dat2[0] = datas2[0]    #Angle in radian
                #     Dat2[1] = datas2[1]    #Distance in mm
                Vs = float(Dat1[0])
                MagXt = float(Dat1[1]) - xoffset
                MagYt = float(Dat1[2]) - yoffset
                MagXh = float(Dat1[3])
                MagYh = float(Dat1[4])
                xt = float(Dat1[5])/1000
                yt = float(Dat1[6])/1000
                xh = float(Dat1[7])/1000
                yh = float(Dat1[8])/1000
                # angle1 = float(Dat2[0])
                # distance1 = float(Dat2[1])/1000
                # if angle1 > 300 and angle1 < 360:
                #     angle = 360 - angle1
                #     distance = distance1*math.cos(angle*math.pi/180)
                # elif angle1 > 0 and angle1 < 60:
                #     distance = distance1*math.cos(angle1*math.pi/180)
                angle = 0
                distance = 0
                Qt = math.atan2(MagYt, MagXt) * 180 / math.pi  #64,130 offset magnetometer y dan x, 7.11.. bias
                if Qt < 0 :
                    Qt = Qt + 360
                Qs = -36 + (Vs - V1s) * 72 / (grads) #Voltage to Angle converter (steer)
                if abs(Qs) > 36 :
                    Qs = signum(Qs) * 36
                Qh = math.atan2(MagYh, MagXh) * 180 / math.pi 
                if Qh < 0 :
                    Qh = Qh + 360
                belok = [kondisi, xh, yh, xt, yt, distance, Qh, Qt, h_eh, h_et, Qh_e, Qh_ee, Qt_e, Qt_ee, Qht_e, psi1, vh1, v_ev3, Qd, dQs, pos, drj, Qs, ei, ep, speed_drj]
                writer5.writerow(belok)
        
        elif kondisi == '6':
            while counting3 < cnt:
                counting3 = counting3 + 1
                data1 = arduino1.readline().strip().decode()
                # data2 = arduino2.readline().strip().decode()
                datas1 = data1.split('\t')
                # datas2 = data2.split('\t')
                lgt1 = len(datas1)
                # lgt2 = len(datas2)
                if lgt1 == 9 :
                    Dat1[0] = datas1[0]    #Steering Voltage
                    Dat1[1] = datas1[1]    #Magnetometer X Trailer
                    Dat1[2] = datas1[2]    #Magnetometer Y Trailer
                    Dat1[3] = datas1[3]    #Magnetometer X Head
                    Dat1[4] = datas1[4]    #Magnetometer Y Head
                    Dat1[5] = datas1[5]    #Trailer X position in mm
                    Dat1[6] = datas1[6]    #Trailer Y position in mm
                    Dat1[7] = datas1[7]    #Head X position in mm
                    Dat1[8] = datas1[8]    #Head Y position in mm
                # if lgt2 == 2 :
                #     Dat2[0] = datas2[0]    #Angle in radian
                #     Dat2[1] = datas2[1]    #Distance in mm
                Vs = float(Dat1[0])
                MagXt = float(Dat1[1]) - xoffset
                MagYt = float(Dat1[2]) - yoffset
                MagXh = float(Dat1[3])
                MagYh = float(Dat1[4])
                xt = float(Dat1[5])/1000
                yt = float(Dat1[6])/1000
                xh = float(Dat1[7])/1000
                yh = float(Dat1[8])/1000
                # angle1 = float(Dat2[0])
                # distance1 = float(Dat2[1])/1000
                # if angle1 > 300 and angle1 < 360:
                #     angle = 360 - angle1
                #     distance = distance1*math.cos(angle*math.pi/180)
                # elif angle1 > 0 and angle1 < 60:
                #     distance = distance1*math.cos(angle1*math.pi/180)
                angle = 0
                distance = 0
                Qt = math.atan2(MagYt, MagXt) * 180 / math.pi  #64,130 offset magnetometer y dan x, 7.11.. bias
                if Qt < 0 :
                    Qt = Qt + 360
                Qs = -36 + (Vs - V1s) * 72 / (grads) #Voltage to Angle converter (steer)
                if abs(Qs) > 36 :
                    Qs = signum(Qs) * 36
                Qh = math.atan2(MagYh, MagXh) * 180 / math.pi 
                if Qh < 0 :
                    Qh = Qh + 360
                belok = [kondisi, xh, yh, xt, yt, distance, Qh, Qt, h_eh, h_et, Qh_e, Qh_ee, Qt_e, Qt_ee, Qht_e, psi1, vh1, v_ev3, Qd, dQs, pos, drj, Qs, ei, ep, speed_drj]
                writer6.writerow(belok)

        count = 0
        ei = 0
        while count < 5 :
            ep = drj - Qs
            ei = ei + ep
            
            speed_drj = 20 * (ep + 0.1 * ei)
            #if ei < 0.1 :
            #speed_drj = 16 * (ep + 1 * ei)
            m.run_to_abs_pos(speed_sp = speed_drj)
            count = count + 1

        mr.run_forever(speed_sp = int(-v_ev3))
        ml.run_forever(speed_sp = int(-v_ev3))
        m.run_to_rel_pos(position_sp = int(-drj), speed_sp = 500)
        
        pos = m.position

        #=====================End of Controller Part===========================
        dt = time() - t
        #Writing to Excel File
        if kb.kbhit():
            c = kb.getch()
            if ord(c) == 27:
                break

    except Exception as e:
        pass
m.stop()
mr.stop()
ml.stop()
writer1.close()
writer2.close()
writer3.close()
writer5.close()
writer6.close()
sys.exit()