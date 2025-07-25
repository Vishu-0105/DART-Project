import socket

esp32_ip = "192.168.48.175"  # Replace with ESP32's IP
port = 1234

client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((esp32_ip, port))

while True:
    data = client.recv(1024)
    if not data:
        break
    print("Received:", data.decode())

client.close()
