#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import std_msgs.msg
import RPi.GPIO as GPIO
import time
import VL53L0X
tof = VL53L0X.VL53L0X(i2c_bus=1,i2c_address=0x29)

#vl53.measurement_timing_budget = 200000
servopin = 12
GPIO.setmode(GPIO.BCM)
GPIO.setup(servopin, GPIO.OUT)
pwm=GPIO.PWM(servopin,50)
pwm.start(0)

def servocntrl(state, angle):
    if (bool(state)) :
       tof.open()
       tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
       duty = ((angle/18)+2)
       GPIO.output(servopin,True)
       pwm.ChangeDutyCycle(duty)
       time.sleep(0.5)
       pwm.ChangeDutyCycle(0)
       start_scan_time = rospy.Time.now()
       distance = (tof.get_distance()/100)
       end_scan_time = rospy.Time.now()
       diff = (start_scan_time - end_scan_time).toSec()  #*1e-3
       return distance,diff
    else :
       GPIO.output(servopin,False)
       tof.stop_ranging()
       tof.close()

def publisher() :
    pub = rospy.Publisher('scan',LaserScan,queue_size=1000)  #scan is the topic
    rospy.init_node('tof_node',anonymous=True)  #node name
    num_readings = 180
    laser_frequency = 2   #not sure,recheck
    scan = LaserScan()
    #rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        for angle in range(0,num_readings) :
            scan.header.stamp = rospy.Time.now()
            scan.header.frame_id= frame_id    #rospy.get_param('~frame_id') 
            scan.angle_min= 0
            scan.angle_max= 180*0.0175  #radians
            scan.angle_increment = 0.0175 #1 degree steps in radians
            scan.time_increment = (1/laser_frequency) / (num_readings)
            scan.range_min = 0
            scan.range_max = 1.2
            scan.ranges,scan.scan_time = servocntrl(True,angle)
            pub.publish(scan)
            #rate.sleep()

if __name__ == '__main__':
   try:
       publisher()
   except rospy.ROSInterruptException
       pass

pwm.stop()
servocntrl(False,0);
GPIO.cleanup()
