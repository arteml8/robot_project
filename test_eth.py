import socket

HOST = "192.168.1.177"  # Match Arduino's IP
PORT = 23

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(b"CMD:DRIVE:0.2,0.0,0.0\n")
    print("Sent command")

    response = s.recv(1024)
    print("Received:", response.decode().strip())