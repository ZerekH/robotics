from mbot_bridge.api import MBot
import time

def drive_square(squares:int):
    my_robot = MBot()  # intialize the robot

    for i in range(squares):
        for j in range(3):
            # move for half a second
            my_robot.drive(1, 0, 0) 
            time.sleep(.5)
            # stop
            my_robot.stop()

            time.sleep(.25)
            

            # turn 90 degrees(1.5708 radians) for 1 second
            my_robot.drive(0, 0, 1.68)
            time.sleep(1.05) 
            # stoping the rotation
            my_robot.stop()
            
    

            # sleep
            time.sleep(.25)

drive_square(4)
