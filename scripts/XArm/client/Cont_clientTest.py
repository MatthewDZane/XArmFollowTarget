# server.py
import socket

HOST = "localhost"
PORT = 8211

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

while True:
    conn, addr = s.accept()
    with conn:
        data = conn.recv(1024)
        if not data:
            break
        print("Received:", data)
        conn.sendall(data)

# client.py
import socket

HOST = "localhost"
PORT = 50000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

s.sendall("Hello, world!".encode())
data = s.recv(1024)

print("Received:", data.decode())

s.close()