import serial
import time
import math

val = "\n"
val_1 = "0\n"
##val_2 = "60\n"
bad_char = ['\\','r','b','n',"'"]
ser_Left = serial.Serial('/dev/ttyUSB1',9600)
ser_Right = serial.Serial('/dev/ttyUSB0',9600)


value = input("enter the initial PWM value for R = ")
value1 = input("enter the initial PWM value for L = ")
value = value+val
value2 = value1+val
time.sleep(1)

ser_Left.write(value2.encode())
ser_Right.write(value.encode())

time.sleep(2)

L = 20 # 20 cm
W=0
D_R=0.0
D_L=0.0
r=0.0
delta=1
theta_new=0
theta=0
x=0
y=0
x_del=0
y_del=0


x_goal=100
y_goal=100
goal_angle=0
e_theta=0
tyre_Radius=3.3

v_R=0
v_L=0
last_e=0

kp=0.8
kp_L=1.0
kp_R=1.0
pwm_L=0
pwm_R=0
error_R=0
error_L=0

def goal_achived():
    while True:
        print("GOAL ACHIVED")
        ser_Left.write(val_1.encode())
        ser_Right.write(val_1.encode())

try:
    while True:
        data_L = ser_Left.readline()
        data_R = ser_Right.readline()
        rpm_L = str(data_L)
        rpm_R = str(data_R)

        for i in bad_char:
            rpm_L = rpm_L.replace(i,'')
            rpm_R = rpm_R.replace(i,'')

        rpm_L = float(rpm_L)
        rpm_R = float(rpm_R)
##        print("rpm_L=",rpm_L,"rpm_R=",rpm_R)
        
##        print(" ")
        D_L = (20.724*rpm_L)/60
        D_R = (20.724*rpm_R)/60

##        if D_R != D_L:
##            r=(L*(D_R+D_L))/(2*(D_R-D_L))

##        print("D_L=",D_L,"D_R=",D_R)
        
        D_C = (D_R+D_L)/2
        phi = (D_R-D_L)/L
        
        W_L = D_L/tyre_Radius
        W_R = D_R/tyre_Radius

        theta_new = theta + phi
        
        x_del = x + D_C*math.cos(theta_new)
        y_del = y + D_C*math.sin(theta_new)

        dx = x_goal-x_del
        dy = y_goal-y_del

        goal_angle = math.atan2(dy,dx)
        e_theta = goal_angle - theta_new
        e_theta = math.atan2(math.sin(e_theta),math.cos(e_theta))

        omega = kp*e_theta
        
        v_R = (2*(D_C)+omega*L)/(2*tyre_Radius)
        v_L = (2*(D_C)-omega*L)/(2*tyre_Radius)

##        print("v_R=",v_R,"v_L=",v_L)
        
        w_r = v_R/tyre_Radius
        w_l = v_L/tyre_Radius
        

##        print("w_r=",w_r,"w_l=",w_l)

        d_r = math.degrees(w_r)/tyre_Radius
        d_l = math.degrees(w_l)/tyre_Radius

        rpm_r_r = (d_r*60)/20.724
        rpm_r_l = (d_l*60)/20.724
        
        print("x_del=",round(x_del,2),"y_del=",round(y_del,2),"rpm_r_r=",round(rpm_r_r,2),"rpm_r_l=",round(rpm_r_l,2))#,"e_theta=",round(math.degrees(e_theta)))

        print(" ")
        error_L = rpm_r_l - rpm_L
        pwm_L = kp_L*error_L
        if pwm_L > 55:
            pwm_L=55
        if pwm_L < 35:
            pwm_L=35
        pwm_L = str(pwm_L)+val
        
        error_R = rpm_r_r - rpm_R
        pwm_R = kp_R*error_R
        if pwm_R > 55:
            pwm_R=55
        if pwm_R < 35:
            pwm_R=35
        pwm_R = str(pwm_R)+val

        print("pwm_L=",pwm_L,"pwm_R=",pwm_R)
        
        if x_del > 80 and x_del < 120:
            if y_del > 80 and y_del < 120:
                ser_Left.write(val_1.encode())
                ser_Right.write(val_1.encode())
                time.sleep(1)
                goal_achived()
            
        ser_Left.write(pwm_L.encode())
        ser_Right.write(pwm_R.encode())
        
        
       
        theta = theta_new
        x = x_del
        y = y_del



        
except KeyboardInterrupt:
    ser_Left.write(val_1.encode())
    ser_Right.write(val_1.encode())
##    time.sleep(1)

       
