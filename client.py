import socket

s = socket.socket()
s.connect(('127.0.0.1',12345))
while True:
    data = s.recv(1024)
    message = data.decode()

    if message == "Done":
        break
    
    print(message)

print("Isaac Sim Connection Stopped")