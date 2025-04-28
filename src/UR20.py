
import socket
import time 

# Define robot IP and port (usually port 30002 for RTDE or 30003 for URScript)
robot_ip = '192.168.50.205'  # Replace with the actual robot's IP
port = 30002  # URScript's default port for communication

# URScript that moves the shoulder joint by 90 degrees from the current position
ur_script = """
def move_robot():
    # Get the current joint positions
    current_positions = get_actual_joint_positions()
    # Adjust the shoulder joint (first joint, index 0) by 90 degrees (π/2 radians)
    target_positions = current_positions
    target_positions[0] = current_positions[0] + 1.57  # Bend the first joint (shoulder) by 90 degrees
    # Move the robot to the new target positions
    movej(target_positions, a=1.4, v=1.05)  # Move with specified acceleration and velocity

# Call the function to move the robot
move_robot()
"""

###
# @param host string, endpoint
# @param port number, port number
# return socket
###
def connect(host: str, port: int):
    # Create socket and connect to the robot
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    return s


# Create socket and connect to the robot
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sock.connect((robot_ip, port))





# sock.close()


def main():
    HOST = '192.168.50.205'  
    PORT = 30002
    socket = connect(HOST, PORT)
    try:
        # Send URScript to the robot
        socket.sendall(ur_script.encode())
        print("Command sent to the robot!")
        time.sleep(2)
    finally:
        socket.close()

if __name__ == "__main__":
    main()