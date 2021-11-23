#=========Movement Manual Check Autonomous Truck Trailer Script========
#Lab ICoDeS, Teknik Fisika, ITB
#Husnul Amri
#======================================================================

from ev3dev.ev3 import *
from time import sleep

#EV3 Related Part
v_ev3 = 30
Qd = 10
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

while True : 
    mr.run_forever(speed_sp = int(-v_ev3))
    ml.run_forever(speed_sp = int(-v_ev3))
    m.run_to_rel_pos(position_sp = int(Qd), speed_sp = 500)
    print("maju kiri")
    sleep(3)
    mr.stop()
    ml.stop()
    m.run_to_abs_pos(position_sp = 0, speed_sp = 500)
    m.stop_action = 'brake'
    print("diam lurus")
    sleep(3)
    mr.run_forever(speed_sp = int(v_ev3))
    ml.run_forever(speed_sp = int(v_ev3))
    m.run_to_rel_pos(position_sp = int(-Qd), speed_sp = 500)
    print("mundur kanan")
    sleep(3)
    mr.stop()
    ml.stop()
    m.run_to_abs_pos(position_sp = 0, speed_sp = 500)
    m.stop_action = 'brake'
    print("diam lurus")
    sleep(3)