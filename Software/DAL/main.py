import pymavlink
from pymavlink import mavutil, mavwp
import time
from multiprocessing import Process, active_children
import numpy as np
from time import sleep
import signal
import sys
import json

global FailSafe_Ready
FailSafe_Ready = False
'''For saftey at no point should the plane be capable of self re-arming, re-arming must only be possible from the GCS, only self disarming may be possible'''
class Link(object):
    def __init__(self, address):
        self.Waypoint_Loader = mavwp.MAVWPLoader()
        self.connection = mavutil.mavlink_connection(address)
        print("Connection Established.")
        self.connection.wait_heartbeat()
        print("Heartbeat recived.")
        self.connection.mav.ping_send(int(time.time() * 1e6),0,0,0)
        self.heart_task = Process(target = self.send_heartbeat)
        self.heart_task.start()
        print("Link Ready.")
    def Set_Messages(self):
        ''' Sets up the nessecary message intervals'''
        self.message_list = [33,83]# Filtered predicted positon, Commanded Attitude 
        for cmd in self.message_list:
            self.connection.mav.command_long_send(self.connection.target_system,self.connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,cmd,2000,1,0,0,0,0) # Sets up a request for the predicted system position every 2000 micro seconds
            self.connection.recv_match(type='COMMAND_ACK', blocking = True)

    def send_heartbeat(self):
        while True:
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
            self.connection.wait_heartbeat()
            sleep(1)
    def set_pwm(self, channel, position):
        self.rc_channel_values = [65535 for _ in range(18)]
        self.rc_channel_values[channel- 1] = position
        self.commection.mav.rc_channels_override_send(self.connection.target_system,self.connection.target_component,*self.rc_channel_values)
    def disarm(self):
        '''Force disarms drone'''
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,0,21196,0,0,0,0,0)
        return self.connection.recv.match(type='COMMAND_ACK', blocking = True)
    def is_landed(self):
        return self.connection.messages['MAV_LANDED_STATE']
    def set_course(self, items):
        '''Uploads a new waypoint mission'''
        self.connection.mav.send()
        self.connection.recv.match(type='MISSION_REQUEST_INT', blocking = True)
        for item in items:
            self.connection.mav.send()
            self.connection.recv.match(type='COMMAND_ACK', blocking = True)
        pass
    def set_geofence(self):
        pass
    #def send_Command(self,command,data):
    #    self.connection.mav.command_long_send(self.connection.target_system)

    #def Recv_Response(self):



    def Upload_Waypoint(self,mission_array):
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MISSION_COUNT,0,1,21196,0,0,0,0,0) # Command 44(Mission Count)
        self.resp = self.connection.recv_match(type='MISSION_REQUEST_INT', blocking = True)
        print(self.resp.to_dict())


class Waypoint(object):
    def __init__(self, index, latitude = None, longditude = None, radius= 30, altitude = 60, flag = None, cont= None):
        self.mission_index = index   #Mission Item index, defines the order in which the items will be executed
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.radius = radius         #Way point acceptance radius
        self.altitude = altitude     #Target waypoint altitude
        self.flag = flag             #Waypoint flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class HALO(object):
    '''The Dynamic Auto Landing mission item instructs the flight director to perform the relevent actions'''
    def __init__(self, index, latitude = None, longditude = None, radius= 30, altitude = 60, flag = None):
        self.mission_index = index
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.radius = 10             #Landing target radius
        self.altitude = altitude     #Landing waypoint altitude
        self.flag = "DAL"            #DAL flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class PCR(object):
    '''This mission item in formed the flight director to monitor the aircraft position and release the cargo at the optimal moment'''
    def __init__(self, index, latitude = None, longditude = None, radius= 30, altitude = 60, flag = None):
        self.mission_index = index
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.radius = radius         #Way point acceptance radius
        self.altitude = altitude     #Target waypoint altitude
        self.flag = "PCR"             #Waypoint flag, Precision Cargo Release flag
    def __call__(self):
        return np.array([self.latitude, self.longditude, self.radius, self.altitude, self.flag])

class Mission(object):
    def __init__(self):
        '''The Mission path is composed of an array of objects that represent specific mission elements such as cargo release, DAL, and mission waypoints'''
        self.path = None
        self.mission_index = 0
    def next_mission_item(self):
        self.mission_index += 1
        return self.path[self.mission_index]

class main():
    def __init__(self):
        '''Parameter List'''
        self.cargo_release_time = 0.25 #Time taken for the cargo to exit the aircraft
        self.cargo_relased = False
        self.FS_Maximum_Period = 1 #second
        self.Velocity_FS_Threshold = [0,30]# Meters per second, keeps the done in line with competiton rules
        self.Altitude_FS_Threshold = [0,120]#Above altitude, keeps the drone in line with regulations
        self.Battery_Voltage_FS_Threshold = [0,24]#Volts, prevents brown outs
        self.Battery_Current_FS_Threshold = [0,100]#Amps, prevents catastrophic short circuit
        '''Runs at system boot'''
        self.link = Link("/dev/ttyACM0") #Establishes communciation with the flight controller.
        signal.signal(signal.SIGINT, self.keyboard_interupt_handler) #Stop people accidently crashing the flight controller while in flight.
        '''Start the Failsafe watchdog'''
        self.FailSafe_Process = Process(target = self.Failsafe_Watchdog) #Defines the Failsafe watch dog daughter process.
        self.FailSafe_Process.start() #Starts the Failsafe process, this must be started before all other processes to ensure saftey.
        print("Watchdog Active.")
        '''Load Plane Parameters'''
        self.ret = self.Load_Parameters()
        if not self.ret:
            pass
        print("Params Loaded.")
        '''Loads mission from json'''
        self.ret = self.Load_mission()
        if not self.ret:
            pass
        print("Mission Loaded Successfully.")
        self.mainloop_process = Process(target = self.mainloop) #Define the
        self.mainloop_process.start()

    def Get_Home(self):

    def Heuristic_Automatic_Landing_Operation(self):# HALO
        '''A fully autinimious heuristic algorithum that optimised the landing heading and flight path angle'''
        pass

    def Precision_Cargo_Release(self):# PCR
        '''Uses onboard velocity time and position estimations to accuratly release the cargo'''
        self.cargo_relased = False
        self.cargo_release_time_threshold = 0.1 #seconds
        while not self.cargo_relased:
            self.link.connection.messages
            self.position = np.array([])
            self.v_x = self.v*np.cos(self.heading+self.d_heading)
            self.v_Y = self.v*np.sin(self.heading+self.d_heading)
            self.rho = self.d_heading*self.v
            self.centre_or_rot = self.position + np.array([self.rho*np.cos(self.heading),self.rho*np.sin(self.heading)])
            self.prop_forward = []
            #Extrapolate the planes current flight path to predict the time at which the cargo should be released
            pass
    def Load_Parameters(self):
        '''Loads Program Parameters'''
        self.failure = False
        self.param_data = json.load(open("param.json","r"))
        self.param_items = self.param_data.keys()

        return self.failure


    def Load_mission(self):
        '''Loads in mission waypoints from a JSON file on an sd card'''
        self.failure = False
        self.Mission_objects = []
        self.mission_data = json.load(open("Mission.json","r"))
        self.mission_items = self.mission_data.keys()
        for self.mission_item in self.mission_items:
            if self.mission_data[self.mission_item]["Flag"] == "Waypoint": #Mission item is a waypoint
                self.Mission_objects.append(Waypoint(self.mission_data[self.mission_item]["Mission_Item_Index"], self.mission_data[self.mission_item]["Latitude"],self.mission_data[self.mission_item]["Longditude"],self.mission_data[self.mission_item]["Acceptance_Radius"], self.mission_data[self.mission_item]["Flag"]))
            elif self.mission_data[self.mission_item]["Flag"] == "HALO": #Mission Landing command
                self.Mission_objects.append(HALO(self.mission_data[self.mission_item]["Mission_Item_Index"], self.mission_data[self.mission_item]["Latitude"],self.mission_data[self.mission_item]["Longditude"],self.mission_data[self.mission_item]["Acceptance_Radius"], self.mission_data[self.mission_item]["Flag"]))
            elif self.mission_data[self.mission_item]["Flag"] == "PCR": #Cargo release supervision
                self.Mission_objects.append(PCR(self.mission_data[self.mission_item]["Mission_Item_Index"], self.mission_data[self.mission_item]["Latitude"],self.mission_data[self.mission_item]["Longditude"],self.mission_data[self.mission_item]["Acceptance_Radius"], self.mission_data[self.mission_item]["Flag"]))
            else:
                print("Dont know how you've ended up here.")
                self.failure = True
                break
        self.Mission_objects.sort(key = lambda x:x.mission_index)
        self.mission = Mission()
        self.mission.path = np.array(self.Mission_objects)
        return self.failure

    def Failsafe_Action(self):
        '''
        The actions of the FTS must aim to safely land the UA as soon as possible after initiation.
        For Fixed Wing aircraft, the throttle shall be set to ‘engine off’ and the control surfaces
        set to initiate a rapid spiral descent.
        For hybrid aircraft, the aircraft shall not transition between hover and forward flight
        following any activation of FTS or Failsafes.
        Other actions could include deployment of a recovery parachute.
        Full aileron left
        Full up elevator
        Full rudder left
        Initiates a death spiral
        '''
        # If in no failsafe region override.
        self.link.set_pwm(2,1900)# Elevator
        self.link.set_pwm(3,1900)# Aileron
        self.link.set_pwm(4,1900)# Rudder
        self.link.set_pwm(1,500) #Throttle

    def Failsafe_Watchdog(self):
        '''Monitors system parameters to ensure complicate with all failsafe conditions'''
        '''This method also enacts the failsafe protocol when nessecary to maintain the saftey of ground staff'''
        '''
        This method must monitor:
        The failsafe watchdog to ensure proper functioning, is the failsafe watchdog running sufficently fast?
        geofence compliance
        minimum altitude compliance
        maximum speed compliance
        battery voltage compliance
        minimum speed compliance
        '''
        self.FS_Maximum_Period = 1 #second
        self.Velocity_FS_Threshold = [0,30]# Meters per second, keeps the done in line with competiton rules
        self.Altitude_FS_Threshold = [0,120]#Above altitude, keeps the drone in line with regulations
        self.Battery_Voltage_FS_Threshold = [0,24]#Volts, prevents brown outs
        self.Battery_Current_FS_Threshold = [0,100]#Amps, catastrophic short circuit
        print("Watchdog Boot.")
        while True:
            sleep(1)
            print("Set state")
            FailSafe_Ready = True
            pass
    def keyboard_interupt_handler(self,q,p):
        '''Execute order 66, kills all the children'''
        '''Interupts the standard keyboard interupt to ensure all processes are correctly terminated.'''
        if not self.link.is_landed():
            print("Craft in flight, command override active.")
        else:
            self.link.disarm()
            self.active = active_children()
            print(len(self.active),"children found.")
            for self.child in self.active:
                self.child.kill()
            print("All child processes terminated.")
            print("Terminating flight director.")
            sys.exit()
    def mainloop(self):
        '''hold while plane is not armed'''
        while not FailSafe_Ready:
            sleep(1)
            print(FailSafe_Ready)
            pass
        print(self.link.connection.motors_armed_wait())
        print("Armed")
        '''Upload Mission'''



        '''Monitor waypoint mission progress.'''

        while True:
            '''What is the current'''
            sleep(0.1)

        ''' Move to cargo drop, Start PCR'''
        self.PCR()



        '''Monitor waypoint mission progress.'''
        while True:
            '''what is the current waypoint.'''
            sleep(0.1)
        '''Start HALO.'''


        sleep(5)
        self.link.disarm()
        #while True:
        #    pass


if __name__ == "__main__":
    main()
