import numpy as np
from std_msgs.msg import String,Bool


class FSM:

    def __init__(self):

        pass


    def transform_disarm(self,data_hub):
        """
        State : 'disarm'
        
        1. disarm =(/mission)====> arm
        """
<<<<<<< HEAD
=======
        avaliable_mission = ["arm"]
>>>>>>> main

        ''' disarm ===> arm (by /mission_msgs from ground_station) '''  
        if data_hub.mission == "arm" and data_hub.transform_trigger:

            print("disarm ---> arm")
            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("arm") # send a mission to trajectory
<<<<<<< HEAD
            
            # inputing mission is not avaliable until the mission "arm" is done
            data_hub.pub2ground.publish(False)
            
            data_hub.cur_state = "arm"

        else:

            # print("Invalid mission recieved. please input the avaliable mission")
=======
            data_hub.cur_state = "arm" # update the current state

            # inputing mission is not avaliable until the mission "arm" is done
            data_hub.pub2ground.publish(False)
            

        elif not data_hub.mission in avaliable_mission: 

>>>>>>> main
            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(True)


<<<<<<< HEAD
=======

>>>>>>> main
    def transform_arm(self,data_hub):

        """
        State : 'arm'
        
        1. arm =(/mission)====> disarm
        2. arm =(auto_disarm)=> disarm
        3. arm =(/mission)====> take_off
        """
<<<<<<< HEAD
=======
        available_mission = ["disarm","take_off"]
>>>>>>> main

        ''' arm ===> disarm (by /mission_msgs from ground_station) '''  
        if data_hub.mission == "disarm" and data_hub.transform_trigger:

            print("arm ---> disarm")
            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("disarm") # send a mission to trajectory
<<<<<<< HEAD
            
            # inputing mission is not avaliable until the mission "disarm" is done
            permission = Bool()
            permission.data = False
            data_hub.pub2ground.publish(permission)

            ''' arm ===> take_off (by /mission_msgs from ground_station) '''  

            data_hub.cur_state = "disarm"

        elif data_hub.mission == "take_off" and data_hub.transform_trigger:
=======
            data_hub.cur_state = "disarm" # update the current state
            
            # inputing mission is not avaliable until the mission "disarm" is done
            data_hub.pub2ground.publish(False)

        ''' arm ===> take_off (by /mission_msgs from ground_station) '''  
        if data_hub.mission == "take_off" and data_hub.transform_trigger:
>>>>>>> main

            print("arm ---> take_off")
            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("take_off") # send a mission to trajectory
<<<<<<< HEAD
=======
            data_hub.cur_state = "take_off" # update the current state

>>>>>>> main
            data_hub.is_performing_action = True # the drone starts to performing an action
            
            # inputing mission is not avaliable until the mission "take_off" is done
            data_hub.pub2ground.publish(False)

<<<<<<< HEAD
            data_hub.cur_state = "take off"

        else:

            # print("Invalid mission recieved. please input the avaliable mission")
            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(False)
=======
        elif not data_hub.mission in available_mission:

            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(True)
>>>>>>> main



    def transform_take_off(self,data_hub):

        """
        State : 'take_off'
        
        1. take_off =(/is_done)====> hold
        """

        ''' take_off ===> hold (by /is_done from motion_controller) '''  
<<<<<<< HEAD
        if data_hub.is_performing_action == False and data_hub.transform_trigger:

            print("take_off ---> hold")
            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("hold") # send a mission to trajectory
            
            # inputing mission is not avaliable until the mission "disarm" is done
            data_hub.pub2ground.publish(False)

            data_hub.cur_state = "hold"
=======
        if data_hub.is_performing_action == False: # if take_off mission is done

            print("take_off ---> hold")
            # data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("hold") # send a mission to trajectory
            data_hub.cur_state = "hold" # update the current state
>>>>>>> main



    def transform_hold(self,data_hub):

        """
        State : 'hold'
        
<<<<<<< HEAD
        1. arm =(/mission)====> disarm
        2. arm =(auto_disarm)=> disarm
        3. arm =(/mission)====> take_off
        """

        ''' arm ===> disarm (by /mission_msgs) '''  
        if data_hub.mission == "disarm" and data_hub.transform_trigger:

            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("disarm") # send a mission to trajectory
            
            # inputing mission is not avaliable until the mission "disarm" is done
            data_hub.pub2ground.publish(False)


        ''' arm ===> take_off (by /mission_msgs) '''  
        if data_hub.mission == "take_off" and data_hub.transform_trigger:

            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("take_off") # send a mission to trajectory
            
            # inputing mission is not avaliable until the mission "take_off" is done
            data_hub.pub2ground.publish(False)

        else:
=======
        1. hold =(/mission)====> land
        2. hold =(/mission)====> park
        3. hold =(/mission)====> search 
        """

        available_mission = ["land","park","search"]

        ''' hold ===> land (by /mission_msgs from ground station) '''  
        if data_hub.mission == "land" and data_hub.transform_trigger:

            print("hold ---> land")
            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("land") # send a mission to trajectory
            data_hub.cur_state = "land" # update the current state
            data_hub.is_performing_action = True # the drone starts to performing an action
            
            # inputing mission is not avaliable until the mission "land" is done
            data_hub.pub2ground.publish(False)

        ''' arm ===> park (by /mission_msgs from ground station) '''  
        if data_hub.mission == "park" and data_hub.transform_trigger:

            print("hold ---> park")
            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("park") # send a mission to trajectory
            data_hub.cur_state = "park" # update the current state
            data_hub.is_performing_action = True # the drone starts to performing an action

            # inputing mission is not avaliable until the mission "park" is done
            data_hub.pub2ground.publish(False)

        ''' arm ===> search (by /mission_msgs from ground station) '''  
        if data_hub.mission == "search" and data_hub.transform_trigger:

            print("hold ---> search")
            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("search") # send a mission to trajectory
            data_hub.cur_state = "search" # update the current state
            data_hub.is_performing_action = True # the drone starts to performing an action
            
            # inputing mission is not avaliable until the mission "search" is done
            data_hub.pub2ground.publish(False)

        elif not data_hub.mission in available_mission:
>>>>>>> main

            # print("Invalid mission recieved. please input the avaliable mission")
            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(True)



    def transform_land(self,data_hub):
<<<<<<< HEAD
        """
        State : 'arm'
        
        1. arm =(/mission)====> disarm
        2. arm =(auto_disarm)=> disarm
        3. arm =(/mission)====> take_off
        """
        ''' arm ===> disarm (by /mission_msgs) '''  
        if data_hub.mission == "disarm" and data_hub.transform_trigger:

            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("disarm") # send a mission to trajectory
            
            # inputing mission is not avaliable until the mission "disarm" is done
            data_hub.pub2ground.publish(False)

        ''' arm ===> take_off (by /mission_msgs) '''  
        if data_hub.mission == "take_off" and data_hub.transform_trigger:

            data_hub.transform_trigger = False # turn off the trigger
            data_hub.pub2trajec.publish("take_off") # send a mission to trajectory
            
            # inputing mission is not avaliable until the mission "take_off" is done
            data_hub.pub2ground.publish(False)

        else:

            # print("Invalid mission recieved. please input the avaliable mission")
            data_hub.transform_trigger = False # turn off the trigger
            
            # give the input permission to input the valid mission
            data_hub.pub2ground.publish(True)
=======

        """
        State : 'land'
        
        1. land =(/is_done)====> disarm
        """

        ''' land ===> disarm (by /is_done from motion_controller) '''  
        if data_hub.is_performing_action == False:

            print("land ---> disarm")
            data_hub.cur_state = "disarm" # update the current state

            # inputing mission is not avaliable until the mission "land" is done
            data_hub.pub2ground.publish(False)


>>>>>>> main



    def transform_state(self,data_hub):

        #FSM Algorithm here

        if data_hub.cur_state == "arm":
            
            self.transform_arm(data_hub)

<<<<<<< HEAD
=======

>>>>>>> main
        elif data_hub.cur_state == "disarm":
        
            self.transform_disarm(data_hub)

        
        elif data_hub.cur_state == "take_off":
        
            self.transform_take_off(data_hub)
        
        elif data_hub.cur_state == "hold":
        
<<<<<<< HEAD
            self.transfom_hold(data_hub)
=======
            self.transform_hold(data_hub)
>>>>>>> main

        
        elif data_hub.cur_state == "land":
        
<<<<<<< HEAD
            pass
=======
            self.transform_land(data_hub)
>>>>>>> main

        
        elif data_hub.cur_state == "park":
        
            pass


        elif data_hub.cur_state == "search":
        
            pass
<<<<<<< HEAD

=======
>>>>>>> main
