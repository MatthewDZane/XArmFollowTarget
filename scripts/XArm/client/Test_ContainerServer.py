# server.py
import socket

HOST = "localhost"
PORT = 12345

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