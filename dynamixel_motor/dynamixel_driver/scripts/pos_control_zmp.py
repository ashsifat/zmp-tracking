#!/usr/bin/env python
__author__ = 'Ash'
__copyright__ = '...'

#__license__ = 'BSD'
__maintainer__ = 'Ashrarul Sifat'
__email__ = 'ashrar7@vt.edu'

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
#import roslib; roslib.load_manifest('numpy_tutorials')
import sys
#from optparse import OptionParser
import math
import sys
import errno
import time
import roslib
roslib.load_manifest('dynamixel_driver')
from dynamixel_driver import dynamixel_io
import rospy
import numpy as np
from std_msgs.msg import Byte
#import dynamixel_io
from dynamixel_driver.dynamixel_const import *

from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import MotorStateList

def twos_complement(val, nbits):
    	"""Compute the 2's complement of int value val"""
    	if val < 0:
             val = (1 << nbits) + val
    	else:
             if (val & (1 << (nbits - 1))) != 0:
            	# If sign bit is set.
            	# compute negative value.
            	val = val - (1 << nbits)
    	return val


def pos_control():
    pub = rospy.Publisher('current', Byte, queue_size=1)
    rospy.init_node('pos_control', anonymous=True)
    rate = rospy.Rate(50) # 10hz
#    angles=np.genfromtxt('athena_pseudo_walk_joint_angles',usecols=(11,12,13,17,18,19))	#r-hip-p(0), r-knee-p(1), r-ankl-p(2), l-hip-p(3),l-knee-p(4), l-ankle-p(5)
#    #print 'Turning left torque %s for motor %d' % (torque_on, motor_id)
#    angles=angles*2048/3.14159
    print 'Turning left hip torque %s for motor %d' % (torque_on, motor_id)
    if dxl_io_0.ping(motor_id):
                torque_response= dxl_io_0.set_torque_enabled(5,1)
		print "torque enable2 response", torque_response
		time.sleep(1) 
		torque_response= dxl_io_0.set_torque_enabled(6,1)
		print "torque enable1 response", torque_response
		time.sleep(1)
                print 'done'
    else:
    	        print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id
     
#    torque_response= dxl_io_0.set_torque_enabled(5,1)
#    print "torque enable2 response", torque_response
#    time.sleep(1) 
#    torque_response= dxl_io_0.set_torque_enabled(6,1)
#    print "torque enable1 response", torque_response
#    time.sleep(1)
#    print 'done'
      
#    print 'Turning right hip torque %s for motor %d' % (torque_on, motor_id)
#    if dxl_io_1.ping(motor_id):
#                torque_response= dxl_io_1.set_torque_enabled(5,1)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
#		torque_response= dxl_io_1.set_torque_enabled(6,1)
#		print "torque enable1 response", torque_response
#		time.sleep(1)
#                print 'done'
#    else:
#    	        print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id_3

#    print 'Turning right knee torque %s for motor %d' % (torque_on, motor_id_3)
#    if dxl_io_2.ping(motor_id_3):
#                torque_response= dxl_io_2.set_torque_enabled(7,1)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
#		torque_response= dxl_io_2.set_torque_enabled(8,1)
#		print "torque enable1 response", torque_response
#		time.sleep(1)
#		torque_response= dxl_io_2.set_torque_enabled(9,1)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
#		torque_response= dxl_io_2.set_torque_enabled(10,1)
#		print "torque enable1 response", torque_response
#		time.sleep(1)
#                print 'done'
#    else:
#    	        print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id_3
#    	            	        
#    print 'Turning left hip torque %s for motor %d' % (torque_on, motor_id_3)
#    if dxl_io_3.ping(motor_id_3):
#                torque_response= dxl_io_3.set_torque_enabled(7,1)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
#		torque_response= dxl_io_3.set_torque_enabled(8,1)
#		print "torque enable1 response", torque_response
#		time.sleep(1)
#		torque_response= dxl_io_3.set_torque_enabled(9,1)
#		print "torque enable2 response", torque_response
#		time.sleep(1) 
#		torque_response= dxl_io_3.set_torque_enabled(10,1)
#		print "torque enable1 response", torque_response
#		time.sleep(1)
#                print 'done'
#    else:
#    	        print 'ERROR: motor %d did not respond. Make sure to specify the correct baudrate.' % motor_id_3

#    torque_response= dxl_io_3.set_torque_enabled(7,1)
#    print "torque enable2 response", torque_response
#    time.sleep(1) 
#    torque_response= dxl_io_3.set_torque_enabled(8,1)
#    print "torque enable1 response", torque_response
#    time.sleep(1)
#    print 'done'

#    write_address= [0x74, 0x00]
#    write_data = [0xAA, 0x02, 0x00, 0x00 ]
#    write_response = dxl_io.write(1,write_address, write_data)
#    print "write done, response:", write_response
    #sync_write_address= [0x66, 0x00]  		#current
    sync_write_address= [0x74, 0x00]		#position
    val = 2048
    nbits = 32
    ############### lhp #########################

    write_data_whole1= twos_complement(val, nbits)
    write_data_11 = write_data_whole1 & 0xFF
    write_data_21 = (write_data_whole1>>8) & 0xFF
    write_data_31 = (write_data_whole1>>16) & 0xFF
    write_data_41 = (write_data_whole1>>32) & 0xFF
    #for motor2 value
    write_data_whole2 = twos_complement(val, nbits)
    write_data_12 = write_data_whole2 & 0xFF
    write_data_22 = (write_data_whole2>>8) & 0xFF
    write_data_32 = (write_data_whole2>>16) & 0xFF
    write_data_42 = (write_data_whole2>>24) & 0xFF
    sync_write_data = [0x04, 0x00, 0x05, write_data_11, write_data_21, write_data_31, write_data_41, 0x06, write_data_12,write_data_22, write_data_32, write_data_42 ]
    dxl_io_0.sync_write(sync_write_address, sync_write_data)
    print "sync_write done, position control prepared"
    time.sleep(1)
    print "5"
    time.sleep(1)
    print "4"
    time.sleep(1)
    print "3"
    time.sleep(1)
    print "2"
    time.sleep(1)
    print "1"
    last_time=time.time()
#    write_=[]
#    read_cur1=[]
#    read_cur2=[]
    read_pos1=[]
#    read_vel2=[]
#    write_time=[]
#    read_timecur1=[]
#    read_timecur2=[]
#    read_time2=[]
#    read_time3=[]
#    torque_1=[]
    last_read_vel=0
    i=1
    last_read_time=last_time
	#rr = rospy.Rate(500) # 10hz
#	while not rospy.is_shutdown() :
#	#read position
#		read_response=dxl_io.read(1,DXL_PRESENT_POSITION,4)
#		read_val_pos= read_response[9] | (read_response[10] << 8)| (read_response[11] << 16)| (read_response[12] << 24)
#		print read_val_pos

    while not rospy.is_shutdown() :
		    current_time =time.time()
		    ############### lhp #########################
		    val1_lhp=int(round(val+100*math.sin(current_time)))
#		    val2_lhp=int(round(val-25*math.sin(current_time)))
		    val2_lhp=val
		    write_data_whole1= twos_complement(val1_lhp, nbits)
		    write_data_11 = write_data_whole1 & 0xFF
		    write_data_21 = (write_data_whole1>>8) & 0xFF
		    write_data_31 = (write_data_whole1>>16) & 0xFF
		    write_data_41 = (write_data_whole1>>32) & 0xFF
		    #for motor2 value
		    write_data_whole2 = twos_complement(val2_lhp, nbits)
		    write_data_12 = write_data_whole2 & 0xFF
		    write_data_22 = (write_data_whole2>>8) & 0xFF
		    write_data_32 = (write_data_whole2>>16) & 0xFF
		    write_data_42 = (write_data_whole2>>24) & 0xFF
		    sync_write_data = [0x04, 0x00, 0x05, write_data_11, write_data_21, write_data_31, write_data_41, 0x06, write_data_12,write_data_22, write_data_32, write_data_42 ]
		    dxl_io_0.sync_write(sync_write_address, sync_write_data)
	
		    print "sync_write done, position control", val1_lhp, val2_lhp
		    i=i+1
		    #read position
#		    read_response=dxl_io_0.read( 5,DXL_PRESENT_POSITION,4)
#		    print read_response
#		    read_val_pos= read_response[9] | (read_response[10] << 8)| (read_response[11] << 16)| (read_response[12] << 24)
#		    print "position, motor 5: ", read_val_pos
#		    #read_val_pos=twos_complement(read_val3, 32)
#		    read_pos1.append(read_val_pos)
##		    read_time3.append(read_response[15])
#		    pub.publish(val)
		
		    if  (current_time-last_time)>20:
#		    if  angles[i,:]==[]:
#			val = 2048
#			val1=int(round(val))#write_.append(val1)
#    			write_data_whole1= twos_complement(val1, nbits)
#		write_data_L1 = write_data_whole1 & 0xFF
#		write_data_H1 = (write_data_whole1>>8) & 0xFF
			val = 2048
			val1=int(round(val))#write_.append(val1)
    			write_data_whole1= twos_complement(val1, nbits)
    			write_data_1 = write_data_whole1 & 0xFF
    			write_data_2 = (write_data_whole1>>8) & 0xFF
    			write_data_3 = (write_data_whole1>>16) & 0xFF
    			write_data_4 = (write_data_whole1>>32) & 0xFF
			sync_write_data = [0x04, 0x00, 0x05, write_data_1,write_data_2,write_data_3,write_data_4, 0x06, write_data_1, write_data_2, write_data_3, write_data_4 ]
			dxl_io_0.sync_write(sync_write_address, sync_write_data)
#			sync_write_data = [0x04, 0x00, 0x07, write_data_1,write_data_2,write_data_3,write_data_4, 0x08, write_data_1, write_data_2, write_data_3, write_data_4, 0x09, write_data_1,write_data_2,write_data_3,write_data_4, 0x0A, write_data_1, write_data_2, write_data_3, write_data_4 ]
#			dxl_io_3.sync_write(sync_write_address, sync_write_data)
			print "sync_write end, position control", i
			break

    		    rate.sleep()

if __name__ == '__main__':
    port_name0='/dev/ttyUSB0'
    port_namespace0='ttyUSB0'
#    port_name1='/dev/ttyUSB1'
#    port_namespace1='ttyUSB1'
#    port_name2='/dev/ttyUSB2'
#    port_namespace2='ttyUSB2'
#    port_name3='/dev/ttyUSB3'
#    port_namespace3='ttyUSB3'
    baud_rate=3000000
    min_motor_id=1
    max_motor_id=25
    update_rate=5
    diagnostics_rate=1
    error_level_temp=75
    warn_level_temp=70
    readback_echo=False
    torque_on=True
    motor_id = 5
    motor_id_3 = 7
    r=0.126
    m=842
    I=m*r*r
    c=r*m*9.81
    #i=0
#    sync_write_address= [0x66, 0x00]
    try:
        dxl_io_0 = dynamixel_io.DynamixelIO(port_name0, baud_rate)		#left hip
#        dxl_io_1 = dynamixel_io.DynamixelIO(port_name1, baud_rate)		#right hip
#        dxl_io_2 = dynamixel_io.DynamixelIO(port_name2, baud_rate)		#right knee
#        dxl_io_3 = dynamixel_io.DynamixelIO(port_name3, baud_rate)		#left knee
    except dynamixel_io.SerialOpenError, soe:
        print 'ERROR:', soe
    else:
	pos_control()
