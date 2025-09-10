# udp_client.py
import socket
import struct

TARGET_HOST = '127.0.0.1'
TARGET_PORT = 9999
message = "파이썬 UDP 클라이언트가 메시지를 보냅니다."

try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    encoded_message = message.encode('utf-8')
    client_socket.sendto(encoded_message, (TARGET_HOST, TARGET_PORT))
    print(f"'{message}' 메시지를 {TARGET_HOST}:{TARGET_PORT} 로 보냈습니다.")

    client_socket.settimeout(5)
    data, server_address = client_socket.recvfrom(65535)
    print(f"서버 {server_address}로부터 {len(data)} 바이트의 데이터를 받았습니다.")
    print(f"받은 데이터(바이트): {data}")
    FORMAT_STRING = '<i12f'
    unpacked_data = struct.unpack(FORMAT_STRING, data)
    print(f"언패킹된 데이터: {unpacked_data}")

except socket.timeout:
    print("응답을 받지 못했습니다. 타임아웃 되었습니다.")
except Exception as e:
    print(f"오류가 발생했습니다: {e}")

finally:
    client_socket.close()
