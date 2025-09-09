import socket
import sys
import time
import struct
class Client:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(self):
        message = "send message"
        encoded_message = message.encode('utf-8')
        self.client_socket.sendto(encoded_message, (self.ip, self.port))
        print(f"'{message}' 메시지를 {self.ip}:{self.port} 로 보냈습니다.")
        self.client_socket.settimeout(5)
        data, server_address = self.client_socket.recvfrom(260)
        print(f"서버 {server_address}로부터 {len(data)} 바이트의 데이터를 받았습니다.")
        print(f"받은 데이터(바이트): {data}")
        FORMAT_STRING = '<i12f'
        unpacked_data = struct.unpack(FORMAT_STRING, data)
        print(f"언패킹된 데이터: {unpacked_data}")


BROADCAST_IP = '255.255.255.255'
BROADCAST_PORT = 9999
TIMEOUT = 5 # 5초 타임아웃

broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
broadcast_socket.settimeout(TIMEOUT)

print("네트워크에서 서버를 찾는 중...")

try:
    # 브로드캐스트 메시지 전송
    broadcast_socket.sendto("FIND_SERVER".encode(), (BROADCAST_IP, BROADCAST_PORT))

    # 서버의 응답을 기다림
    data, server_address = broadcast_socket.recvfrom(16)
    response = data.decode()

    if "_" in response:
        server_ip = response.split("_")[1]
        print(f"서버 IP를 찾았습니다: {server_ip}")
    client = Client(server_ip, 9001)
    while True:
        client.send()
        time.sleep(2)

except socket.timeout:
    print("타임아웃: 서버를 찾지 못했습니다.")
except Exception as e:
    print(f"오류 발생: {e}")
finally:
    pass
    #client_socket.close()