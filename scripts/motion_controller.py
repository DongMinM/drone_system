#!/usr/bin/python3
import asyncio
import rospy

from mavsdk             import System
from drone_system.msg   import Status
from std_msgs.msg       import String,Bool,Float32MultiArray
from mavsdk.offboard    import VelocityBodyYawspeed,VelocityNedYaw,PositionNedYaw,OffboardError


class MotionController:

    def __init__(self):
        
        rospy.init_node("motion_controller")
        rospy.Subscriber("/motion_msgs",String,self.motion_update)
        rospy.Subscriber("/trajec_msgs",Float32MultiArray,self.motion_update)
        self.pub2trajec = rospy.Publisher("/trajectory_request",Bool,queue_size=1)
        self.pub2fsm = rospy.Publisher("/is_done",Bool,queue_size=1)
        self.motion_rate = 10 #hz

        self.motion_command = None

        self.drone = System()

    def motion_update(self,motion_command):

       self.motion_command = motion_command.data


    async def connect(self):

        # await self.drone.connect(system_address="serial:///dev/ttyUSB0:921600")
        await self.drone.connect(system_address="udp://:14540")

        print("Connecting in motion...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"Connected in motion              ",end="\r")
                break
        

    async def start_offboard(self):

        print("-- Starting offboard")
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
        await self.drone.offboard.set_position_ned(PositionNedYaw(0,0,0,0))
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
        
        try:
            await self.drone.offboard.start()
        except OffboardError as error:
            print(f"Starting offboard mode failed with error code: \
                {error._result.result}")
            # print("-- Disarming")
            # await self.drone.action.disarm()
            return


    async def arm(self):
        
        print("Armed")
        await self.drone.action.arm()


    async def disarm(self):
        
        print("Disarmed")
        await self.drone.action.disarm()


    async def take_off(self):

        print("take_off")
        # await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,-2.0,0))
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,-2.0,0))
        await asyncio.sleep(5)


    async def hold(self):

        print("hold         ",end="\r")
        # await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0,0,0,0))
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(0,0,0,0))
        await asyncio.sleep(1/self.motion_rate)
    
    async def land(self):

        print("land")
        await self.drone.action.land()


    async def trajectory_tracking(self,trajectory):

        print("tracking")


    async def is_done(self):

        self.pub2fsm.publish(True)


    async def action_handler(self):

        while not rospy.is_shutdown():

            if self.motion_command != None:

                if type(self.motion_command) == str: # if its the simple mission


                    if self.motion_command == "arm":
                        
                        self.motion_command = None
                        await self.arm()
                        await self.is_done()


                    elif self.motion_command == "disarm":

                        self.motion_command = None
                        await self.disarm()
                        await self.is_done()


                    elif self.motion_command == "take_off":

                        self.motion_command = None
                        await self.start_offboard()
                        await self.take_off()
                        await self.is_done()
                        

                    elif self.motion_command == "hold":

                        # self.motion_command = None
                        await self.hold()
                        # await self.is_done()


                    elif self.motion_command == "land":

                        self.motion_command = None
                        await self.land()
                        await self.is_done()


                else: # if it is the mission requirs planning a trajectory


                    # "0" means that it didnt reached to the destination
                    if self.motion_command[0] == 0: 
                        
                        await self.trajectory_tracking(self.motion_command[1:])

                        # send a signal if the drone had done
                        # tracking the trajectory recieved
                        self.pub2trajec.publish(True) 

                    # "1" means that its a final trajectory for the destination
                    elif self.motion_command[0] == 1:

                        await self.trajectory_tracking(self.motion_command[1:])

                        # send a signal if the drone had done
                        # the mission recieved
                        await self.is_done()


    async def main(self):

        await self.connect()
        await self.action_handler()


if __name__ == "__main__":

    M = MotionController()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(M.main())