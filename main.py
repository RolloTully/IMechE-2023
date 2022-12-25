'''
__/\\\\\\\\\\\\\\\___/\\\\\\\\\\\\\\\______/\\\\\\\\\______/\\\\____________/\\\\_
 _\///////\\\/////___\/\\\///////////_____/\\\\\\\\\\\\\___\/\\\\\\________/\\\\\\_
  _______\/\\\________\/\\\_______________/\\\/////////\\\__\/\\\//\\\____/\\\//\\\_
   _______\/\\\________\/\\\\\\\\\\\______\/\\\_______\/\\\__\/\\\\///\\\/\\\/_\/\\\_
    _______\/\\\________\/\\\///////_______\/\\\\\\\\\\\\\\\__\/\\\__\///\\\/___\/\\\_
     _______\/\\\________\/\\\______________\/\\\/////////\\\__\/\\\____\///_____\/\\\_
      _______\/\\\________\/\\\______________\/\\\_______\/\\\__\/\\\_____________\/\\\_
       _______\/\\\________\/\\\\\\\\\\\\\\\__\/\\\_______\/\\\__\/\\\_____________\/\\\_
        _______\///_________\///////////////___\///________\///___\///______________\///__
__/\\\________/\\\___________________/\\\\\\________________________________________________________________
 _\/\\\_______\/\\\__________________\////\\\________________________________________________________________
  _\/\\\_______\/\\\_____________________\/\\\_______________________/\\\__/\\\_______________________________
   _\/\\\\\\\\\\\\\\\___/\\\\\\\\\________\/\\\_________/\\\\\\\\____\//\\\/\\\_______/\\\\\______/\\/\\\\\\___
    _\/\\\/////////\\\__\////////\\\_______\/\\\_______/\\\//////______\//\\\\\______/\\\///\\\___\/\\\////\\\__
     _\/\\\_______\/\\\____/\\\\\\\\\\______\/\\\______/\\\______________\//\\\______/\\\__\//\\\__\/\\\__\//\\\_
      _\/\\\_______\/\\\___/\\\/////\\\______\/\\\_____\//\\\__________/\\_/\\\______\//\\\__/\\\___\/\\\___\/\\\_
       _\/\\\_______\/\\\__\//\\\\\\\\/\\___/\\\\\\\\\___\///\\\\\\\\__\//\\\\/________\///\\\\\/____\/\\\___\/\\\_
        _\///________\///____\////////\//___\/////////______\////////____\////____________\/////______\///____\///__


This is a pre-alpha, it contains all the usual bug of pre-alpha code, its buggy, its unstable and, it is entirly untested with no redundent saftey.

DO NOT FLY THIS CODE

'''

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

class Link(object):
    def __init__(self, address):
        self.Waypoint_Loader = mavwp.MAVWPLoader()
        self.connection = mavutil.mavlink_connection(address)
        print("Connection Established.")
        self.connection.wait_heartbeat()
        print("Heartbeat recived.")
        #self.connection.mav.ping_send(int(time.time() * 1e6),0,0,0)
        #self.heart_task = Process(target = self.send_heartbeat)
        #self.heart_task.start()
        print("Link Ready.")


        self.connection.set_mode_manual()

    def Set_Messages(self):
        ''' Sets up the nessecary message intervals'''
        self.message_list = [33, 83]# Filtered predicted positon, Commanded Attitude
        for cmd in self.message_list:
            self.connection.mav.command_long_send(self.connection.target_system,self.connection.target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,cmd,2000,1,0,0,0,0) # Requests the required messages
            self.connection.recv_match(type='COMMAND_ACK', blocking = True) # Waits for confirmation.

    def Do_Takeoff(self, mission_item):# Working
        self.latitude, self.longditude, self.altitude, self.minimum_Pitch = mission_item()
        print("Takeoff", self.latitude, self.longditude, self.altitude, self.minimum_Pitch)
        self.point = mavutil.mavlink.MAVLink_mission_item_message(self.connection.target_system, self.connection.target_component, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, self.minimum_Pitch[0], 0, 0, 0, (float(self.latitude[0])), (float(self.longditude[0])), self.altitude[0])

        self.Waypoint_Loader.add(self.point)
        self.Send_waypoints()

    def Do_Waypoints(self, mission_item):
        ''' Uploads a sequence of way points'''
        self.latitude, self.longditude, self.radius, self.pass_radius, self.altitude = mission_item()
        print(mission_item())
        for self.mission_item_index in range(0,len(self.latitude)):
            self.point = mavutil.mavlink.MAVLink_mission_item_message(self.connection.target_system, self.connection.target_component, self.mission_item_index, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, (float(self.radius[self.mission_item_index])), (float(self.pass_radius[self.mission_item_index])), 0, (float(self.latitude[self.mission_item_index])), (float(self.longditude[self.mission_item_index])), self.altitude[self.mission_item_index])
            self.Waypoint_Loader.add(self.point)
        self.Send_waypoints()
    def Do_Landing(self, mission_item):
        '''Needs 2 waypoints to define the landing slope'''
        self.latitude, self.longditude, self.altitude = mission_item()


    def Send_waypoints(self):#SOMETHING IS WRONG HERE
        self.connection.waypoint_clear_all_send()
        self.connection.waypoint_count_send(self.Waypoint_Loader.count())
        print("waypoint count",self.Waypoint_Loader.count())
        for i in range(self.Waypoint_Loader.count()):
            print("Mission item", i)
            self.msg = self.connection.recv_match(type=['MISSION_REQUEST'],blocking=True)
            print(self.Waypoint_Loader.wp(self.msg.seq))
            self.connection.mav.send(self.Waypoint_Loader.wp(self.msg.seq)) #This line is broken
            print('Sending waypoint {0}'.format(self.msg.seq))
            print("Mission item", i, "uploaded")
        self.msg = self.connection.recv_match(type=['MISSION_ACK'],blocking=True) # OKAY
        print(self.msg)

    def Send_Message_To_GCS(self, message):
        pass

    def send_heartbeat(self):
        '''Runs async'''
        while True:
            self.connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
            self.connection.wait_heartbeat()
            sleep(0.9)

    def set_pwm(self, channel, position):
        self.rc_channel_values = [65535 for _ in range(18)]
        self.rc_channel_values[channel- 1] = position
        self.connection.mav.rc_channels_override_send(self.connection.target_system,self.connection.target_component,*self.rc_channel_values)
    def disarm(self):
        '''Force disarms drone'''
        self.connection.mav.command_long_send(self.connection.target_system, self.connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,0,21196,0,0,0,0,0)
        return self.connection.recv.match(type='COMMAND_ACK', blocking = True)
    def is_landed(self):
        return self.connection.messages['MAV_LANDED_STATE']

    def set_geofence(self):
        pass



class Path(object):
    '''Defines a path for the drone to follow'''
    def __init__(self, latitude = None, longditude = None, radius = 5, p_rad = 0, altitude = 60):
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.altitude = altitude     #Landing waypoint altitude
        self.radius = radius         #Way point acceptance radius
        self.pass_radius = p_rad        #Direction to pass the way point
        self.altitude = altitude     #Target waypoint altitude
    def __call__(self):
        return self.latitude, self.longditude, self.radius, self.pass_radius, self.altitude
class Takeoff(object):
    "Defines the takeoff"
    def __init__(self, latitude = None, longditude = None, altitude = 50, minimum_pitch = 15):
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.altitude = altitude     #Landing waypoint altitude
        self.minimum_Pitch = minimum_pitch
    def __call__(self):
        return self.latitude, self.longditude, self.altitude, self.minimum_Pitch
class HALO(object):
    '''The Dynamic Auto Landing mission item instructs the flight director to perform the relevent actions'''
    def __init__(self, index, latitude = None, longditude = None, radius= 30, altitude = 60):
        self.latitude = latitude     #Landing Latitude
        self.longditude = longditude #Landing Longditude
        self.altitude = altitude     #Landing waypoint altitude
    def __call__(self):
        return self.latitude, self.longditude, self.altitude
class PCR(object):
    '''This mission item in formed the flight director to monitor the aircraft position and release the cargo at the optimal moment'''
    def __init__(self, latitude = None, longditude = None, altitude = 30):
        self.latitude = latitude     #Latitude
        self.longditude = longditude #Longditude
        self.altitude = altitude     #Target waypoint altitude
    def __call__(self):
        return self.latitude, self.longditude, self.altitude

class main():
    def __init__(self):
        print("PRE RELEASE - DO NOT FLY")
        '''WGS84 is used with the meridian at greenwich so no angular offset is needed'''

        '''Parameter List'''
        self.earth_geoid_radius = 6378137.0 #Meters
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
        #self.FailSafe_Process = Process(target = self.Failsafe_Watchdog) #Defines the Failsafe watch dog daughter process.
        #self.FailSafe_Process.start() #Starts the Failsafe process, this must be started before all other processes to ensure saftey.
        print("Watchdog Active.")

        '''Load Plane Parameters'''
        self.ret = self.Load_Parameters()
        if not self.ret:
            print("Params Loaded.")
            pass
        else: # PARAM LOADING FAILED
            pass

        '''Loads mission from json'''
        self.ret, self.mission = self.Load_mission()
        if not self.ret:
            print("Mission Loaded Successfully.")
            pass
        else: # MISSION LOADING FAILED
            pass # ADD FAILURE PROTOCOL

        self.mainloop_process = Process(target = self.mainloop) #Define the
        self.mainloop_process.start()
    def Heuristic_Automatic_Landing_Operation(self):# HALO
        '''A fully autinimious heuristic algorithum that optimised the landing heading and flight path angle'''
        pass

    def Precision_Cargo_Release(self, item):# PCR
        '''Uses onboard velocity time and position estimations to accuratly release the cargo'''
        self.cargo_relased = False
        self.lead_lag_distance = 10 #Meters
        self.angular_leag_lag_distance = 10/self.earth_geoid_radius #Radians
        self.cargo_release_time_threshold = 0.1 #seconds
        self.wp_latitude, self.wp_longditude, self.wp_alt = item()
        #Requests wind vectors data stream
        self.msg = pymavlink.mavutil.mavlink.MAVLink_request_data_stream_message(self.link.connection.target_system,self.link.connection.target_component,pymavlink.mavutil.mavlink.MAV_DATA_STREAM_ALL,1,1)
        #Sends message
        self.link.connection.mav.send(self.msg)
        print("Message sent")
        #Wait for message to be recived
        self.msg = self.link.connection.recv_match(type='WIND_CONV', blocking=True, timeout = 2)
        print(self.msg)
        if self.msg == None:
            print("No wind convention available.")
        else:
            print("Wind convention found.")
        '''Extract wind vector data'''

        '''Computes wind heading'''
        self.wind_heading = np.atan2([self.E_wind],[self.N_wind]) #Radians
        '''Compute leading and lagging points'''
        self.lagging_point = [self.wp_latitude, self.wp_longditude] + [self.angular_leag_lag_distance*np.cos(self.wind_heading),self.angular_leag_lag_distance*np.sin(self.wind_heading)] #Its not unresonable to approximate the sphere to a flat plain at this scale and location
        self.leading_point = [self.wp_latitude, self.wp_longditude] - [self.angular_leag_lag_distance*np.cos(self.wind_heading),self.angular_leag_lag_distance*np.sin(self.wind_heading)]
        '''Generate the path'''
        self.Latitudes = [self.leading_point[0], self.wp_latitude, self.lagging_point[0]]
        self.Longditudes = [self.leading_point[1], self.wp_longditude, self.lagging_point[1]]
        self.PCR_path = Path(self.Latitudes, self.Longditudes,[5,5,5], [0,0,0], [self.wp_alt,self.wp_alt,self.wp_alt])
        '''Upload Path'''
        self.link.Do_Waypoints(self.PCR_path)

        '''
        At this point the aircraft should be heading toward the start of the drop run, we should wait for the first waypoint to be passed
        once its passed we can move to active monitoring and wait for the drop time to be zero
        '''
        self.passed = False
        while self.passed == False:
            #Collect current mission message
            self.msg = self.link.connection.recv_match(type='MISSION_CURRENT', blocking=True, timeout = 2)
            #Extract the current mission sequence index
            if :
                self.passed = True
            else:
                pass
        '''Starts monitoring position and release time'''

        '''Release cargo, does t<=0'''
        while not self.cargo_relased:
            #Radius of instantanious path
            self.radius = self.v/self.d_heading
            #Compute the location of the centre of rotation
            if self.d_heading>0: # Centre of rotation will be to te right of the aircraft
                self.COR = [self.radius*np.cos(self.heading),-self.radius*np.sin(self.heading)]+self.position# The global position of the centre of rotation
            elif self.d_heading<0: # Centre of rotation will be to the left of the aircraft
                self.COR = [-self.radius*np.cos(self.heading),self.radius*np.sin(self.heading)]+self.position
            else: # Centre of rotation  is at infinity
                pass
            #Angle from North to plane WRT centre of rotation
            self.plane_angle_from_cor =  np.atan2()
            #Angle from North to target WRT centre of rotation
            self.target_angle_from_cor = np.atan2()
            #Time to closest approach
            self.time_to_CA = (self.target_angle_from_cor-self.plane_angle_from_cor)/self.d_heading
            if self.time_to_CA<=self.cargo_release_time_threshold:
                self.link.set_pwm(7,2200)# Moves release servo to open position
                self.cargo_relased = True


    def Load_Parameters(self):
        '''Loads Program Parameters'''
        self.failure = False
        self.param_data = json.load(open("param.json","r"))
        self.param_items = self.param_data.keys()
        return self.failure


    def Load_mission(self):
        '''Loads in mission waypoints from a JSON file on an sd card'''
        self.Mission_objects = []
        self.ret = False
        try:
            self.mission_data = json.load(open("Mission.json","r"))
            self.mission_sections = self.mission_data.keys()
            for self.mission_section in self.mission_sections:
                self.section_type = self.mission_data[self.mission_section]["Type"]
                if self.section_type == "TKOF":
                    self.Minimum_Pitch = [float(self.mission_data[self.mission_section]["TKOF_Options"]["Minimum_Pitch"])]
                    self.Latitude = [float(self.mission_data[self.mission_section]["TKOF_Options"]["Latitude"])]
                    self.Longditude = [float(self.mission_data[self.mission_section]["TKOF_Options"]["Longditude"])]
                    self.Altitude = [float(self.mission_data[self.mission_section]["TKOF_Options"]["Altitude"])]
                    self.Mission_objects.append(Takeoff(self.Latitude, self.Longditude, self.Altitude, self.Minimum_Pitch))
                elif self.section_type == "WP":
                    self.Latitude = []
                    self.Longditude = []
                    self.Altitude = []
                    self.Acceptance_Radius = []
                    self.Pass_Distance = []
                    for self.Mission_item in self.mission_data[self.mission_section]["Waypoints"].keys():
                        self.Latitude.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Latitude"])
                        self.Longditude.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Longditude"])
                        self.Altitude.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Altitude"])
                        self.Acceptance_Radius.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Acceptance_Radius"])
                        self.Pass_Distance.append(self.mission_data[self.mission_section]["Waypoints"][self.Mission_item]["Pass_Distance"])
                    self.Mission_objects.append(Path(self.Latitude, self.Longditude, self.Acceptance_Radius, self.Pass_Distance, self.Altitude))
                elif self.section_type == "PCR":
                    self.Latitude = [self.mission_data[self.mission_section]["PCR_Data"]["Latitude"]]
                    self.Longditude = [self.mission_data[self.mission_section]["PCR_Data"]["Longditude"]]
                    self.Mission_objects.append(PCR(self.Latitude, self.Latitude))
                elif self.section_type == "HALO":
                    self.Latitude = [self.mission_data[self.mission_section]["HALO_Data"]["Latitude"]]
                    self.Longditude = [self.mission_data[self.mission_section]["HALO_Data"]["Longditude"]]
                    self.Altitude = [self.mission_data[self.mission_section]["HALO_Data"]["Altitude_Abort"]]
                    self.Mission_objects.append(HALO(self.Latitude, self.Latitude, self.Altitude))
                else:
                    print("Mission item unknown")
        except:
            self.ret = True
        return self.ret, self.Mission_objects

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
        #MUST CHANGE MODE TO MANUAL
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
        #while not FailSafe_Ready:
        #    sleep(1)
        #    print(FailSafe_Ready)
        #    pass


        '''Wait for arming'''
        '''checks once a second if the drone is armed'''
        print("Waiting for arming!")
        self.link.connection.motors_armed_wait()
        print("Motors armed")
        while not self.link.connection.motors_armed():
            sleep(2)
            print(self.link.connection.motors_armed())
        print("Motor arm confirmed")

        '''Monitor waypoint mission progress.'''
        print("here")
        for self.mission_item in self.mission:
            if isinstance(self.mission_item, Takeoff):
                print("Takeoff", self.mission_item)
                self.link.Do_Takeoff(self.mission_item) #Uploads mission waypoints

            elif isinstance(self.mission_item, Path):
                print("Path")
                self.link.Do_Waypoints(self.mission_item) #Uploads mission waypoints

            elif isinstance(self.mission_item, PCR):
                print("PCR")
                self.Precision_Cargo_Release(self.mission_item) #Monitors and releases cargo

            elif isinstance(self.mission_item, HALO):
                print("Landing")
                self.link.Do_Landing(self.mission_item) #Uploads mission waypoints
            else:
                pass
        '''Mission Complete'''

        sleep(5)
        self.link.disarm()

if __name__ == "__main__":
    main()
