import urx

ip = '192.168.50.205'

try:

    robot = urx.Robot(ip)

    robot.movej([0, 0, 0, 0, 0, 0], 1.0, 1.0)
except urx.RobotException as e:
    print(e)   
finally:
    robot.close()