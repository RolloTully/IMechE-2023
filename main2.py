import pymavlink
from pymavlink import mavutil
import time
from multiprocessing import Process, active_children
from time import sleep
import signal
import sys,json
connection = mavutil.mavlink_connection("udp:192.168.4.2:14550",autoreconnect=True)
print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))
#connection.wait_heartbeat()
print("heart beat")
#connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
#connection.mav.request_data_stream_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 115200, 1)
connection.mav.command_long_send(connection.target_system,connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,33,100000 ,1,0,0,0,0)
connection.recv_match(type='COMMAND_ACK', blocking = True)
connection.mav.command_long_send(connection.target_system,connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,83,100000 ,1,0,0,0,0)
connection.recv_match(type='COMMAND_ACK', blocking = True)
print("here")
#connection.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,"QGC will read this".encode())
#connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
rc_channel_values = [65535 for _ in range(18)]
rc_channel_values[2- 1] = 1500
commection.mav.rc_channels_override_send(connection.target_system,connection.target_component,*rc_channel_values)
while True:
    connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
    connection.wait_heartbeat()
    #connection.recv_match(type='COMMAND_ACK', blocking = True)
    print(connection.recv_match(type='ATTITUDE',blocking = True))
    print(connection.recv_match(type='GLOBAL_POSITION_INT',blocking = True))

    #print(connection.messages['WIND'])
    #print(connection.messages['ATTITUDE'])
    #print(connection.messages['RANGEFINDER'])
#print(connection.messages[''])
#msg = connection.recv_match(type='SYS_STATUS', condition='SYS_STATUS.mode==2 and SYS_STATUS.nav_mode==4', blocking=True)
#print(msg)
#connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,21196,0,0,0,0,0)
#sleep(20)

#connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,21196,0,0,0,0,0)
#connection.mav.command_long_send(connection.target_system, connection.target_component, 16,0,0,21196,0,0,0,0,0)
