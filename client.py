import socket
import math
mysocket = socket.socket()
mysocket.connect(('127.0.0.1',12345))

def close_socket(thissocket):
    try:
        thissocket.shutdown(socket.SHUT_RDWR)
        thissocket.close()
        thissocket = None
    except socket.error as e:
        pass
    print("socket is closed")

try:
    while True:
        data = mysocket.recv(1024)
        message = data.decode()
        if message == "Done":
            break
        joints = eval(message)
        print(joints)
        joints_deg = [math.degrees(joint) for joint in joints]
        # print(message)
        print(joints_deg)
except KeyboardInterrupt:
    print("closing socket...")
    close_socket(mysocket)

print("Isaac Sim Connection Stopped")