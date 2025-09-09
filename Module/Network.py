import socket
import struct
import threading
import time

class Network:
    def __init__(self, port, broadcast_port):
        self.host = self.get_local_ip()#'127.0.0.1' #localHost
        self.port = port
        self.broadcast_port = broadcast_port
        # UDP 소켓 생성
        # socket.AF_INET: IPv4를 사용
        # socket.SOCK_DGRAM: UDP 통신을 의미
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print(f"setting value: {port}")
        print("Network Initializing Completed.")

    def init_server(self):
        # 소켓을 주소에 바인딩
        self.server_socket.bind((self.host, self.port))

        print(f"UDP 서버가 {self.host}:{self.port} 에서 시작되었습니다.")
        thread1 = threading.Thread(target = self.announce_ip, args=("Broadcaster",))
        thread2 = threading.Thread(target = self.receive, args=("Receiver",))

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()

    def get_local_ip(self):
        """자신의 로컬 IP 주소를 반환하는 함수"""
        time.sleep(2)
        temp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # 가상의 외부 서버에 연결하여 자신의 IP를 가져옴
            temp_socket.connect(('8.8.8.8', 80))
            local_ip = temp_socket.getsockname()[0]
        except Exception:
            # 오류 발생 시 로컬호스트 주소 반환
            local_ip = '127.0.0.1'
        finally:
            temp_socket.close()
        return local_ip
    
    # [TODO] 클라이언트가 모두 연결 됐다면, 소켓을 닫는 로직 만들어야 함. 
    def announce_ip(self, name):
        time.sleep(2)
        # broadcast socket 새로 생성
        broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        broadcast_socket.bind(('', self.broadcast_port))

        print(f"서버가 브로드캐스트 포트 {self.broadcast_port}에서 클라이언트를 기다립니다.")
        
        while True:
            try:
                data, client_address = broadcast_socket.recvfrom(16)
                print(f"클라이언트 {client_address}로부터 브로드캐스트 메시지를 받았습니다: {data.decode()}")

                if data.decode() == "FIND_SERVER":
                    # 클라이언트에게 자신의 IP 주소와 포트 정보 응답
                    response_message = f"_{self.get_local_ip()}"
                    broadcast_socket.sendto(response_message.encode(), client_address)
                    print(f"클라이언트 {client_address}에게 나의 IP를 보냈습니다.")

            except Exception as e:
                print(f"오류 발생: {e}")

    def receive(self, name):
        while True:
            # 클라이언트로부터 데이터 수신
            data, client_address = self.server_socket.recvfrom(65535)

            # 수신된 데이터와 클라이언트 주소 출력
            print(f"클라이언트 {client_address} 로부터 다음 데이터를 받았습니다: {data.decode('utf-8')}")

            # 응답 메시지 생성
            response = "서버가 메시지를 잘 받았습니다! 데이터를 보냅니다."
            index = 21
            
            x_pos = 251.242
            y_pos = -105.2341
            z_pos = 21.8

            x_axis_x = 235.1
            x_axis_y = 0.235
            x_axis_z = 2.51
            
            y_axis_x = 5.1042
            y_axis_y = 3.1042
            y_axis_z = 2.1642
            
            z_axis_x = 52.251
            z_axis_y = 24.152
            z_axis_z = 831.002

            # '<'는 리틀 엔디안(little-endian)을 의미
            # 'i'는 int (4바이트), 'f'는 float (4바이트)
            format_string = '<i12f'

            # 모든 변수를 튜플로 묶음
            data_tuple = (index, x_pos, y_pos, z_pos,
                          x_axis_x, x_axis_y, x_axis_z,
                          y_axis_x, y_axis_y, y_axis_z,
                          z_axis_x, z_axis_y, z_axis_z)

            # struct.pack을 사용하여 바이너리 패킷으로 묶기
            packed_data = struct.pack(format_string, *data_tuple)

            print(f"포맷 스트링: {format_string}")
            print(f"패킷 크기: {len(packed_data)} 바이트")
            # 출력: 패킷 크기: 52 바이트 (4바이트 + 12 * 4바이트)
            # 클라이언트에게 응답 전송
            self.server_socket.sendto(packed_data, client_address)