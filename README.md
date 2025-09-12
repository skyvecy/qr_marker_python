# Title
QR 마커 카메라 R&D 프로젝트

# Introduction
- OpenCV의 ArUco Marker 인식 기능을 활용해 R&D를 진행한다.
- Sterolabs 사의 [Zed X Camera](https://www.stereolabs.com/en-kr/store/products/zed-x-stereo-camera)를 사용한다.
- Nvidia Jetson 환경에서 개발을 진행한다.

# Environment
## Software
- Python Version: 3.10.12 
- OS: Linux(Obuntu 22.04.5 LTS) / L4T(Linux for Tegra)(R36.4.4)
- Jetpack: 6.2.1
### Pacakages
#### External
- pyzed(5.0)
- numpy(1.26.4)
#### Internal
- math
- dataclasses
- cv2(4.8.0)
    - 시스템 패키지 관리자(apt)를 통해 jetPack에 의해 사전 설치 됨.
## Hardware
- Camera: [Zed X](https://www.stereolabs.com/en-kr/store/products/zed-x-stereo-camera)
- Device: [NVIDIA Jetson Orin Nano Developer Kit](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/)
- CPU Architecture: ARM64

# QR 마커 서버 동작 환경

[QR 서버 버전(동영상)](https://drive.google.com/file/d/1ZapBdnMDqIu2dd6qPZs1M8eKOyYbKVMQ/view?usp=sharing)
- Broadcast를 활용한 로컬 IP 찾기 로직 적용
   - **9999**번 포트로 동작
- UDP 서버 동작
   - 9001번 포트로 동작
   - 클라이언트가 데이터 요청을 하면 해당 데이터 셋으로 데이터 전송
      - 16byte 미만 값을 보내야 한다.
      - 바이트 오더: 리틀 엔디안
      - 데이터 셋(byte[]): 52바이트 * 5 = 260바이트
         - 4바이트: 데이터 존재 유무(0, 1)
            - 만약 데이터가 존재하지 않으면, 아래 데이터들은 모두 0 값으로 전달
         - 12바이트: 마커 위치
         - 36바이트: X Axis 값(12), Y Axis 값(12), Z Axis 값(12)
[파이썬 클라이언트(동영상)](https://drive.google.com/file/d/1Qcn1sj8csFSdYuoJL9s6zLIP6NwAup-X/view?usp=sharing)
- Broadcast를 활용한 로컬 IP 찾기 로직
- 9999번 포트로 동작
- 16바이트 미만의 문자열(send message)을 서버로 전달  후 서버 데이터 셋 byte[] 형태로 전달 받음

# Refereneces
## Zed Camera
[Zed SDK Download](https://www.stereolabs.com/en-kr/developers/release)
- Nvidia Jetson 페이지에서 **ZED SDK for JetPack 6.1 and 6.2 (L4T 36.4) 5.0 (Jetson Orin, CUDA 12.6)**를 찾아, 다운로드 하면 됨.
[Zed SDK Docs](https://www.stereolabs.com/docs)
[Zed 카메라와 Nvidia Jetson 디바이스와 물리적인 연결 방법](https://www.stereolabs.com/docs/embedded/zed-link/mono-jetson-orin-nano-devkit-setup)
## Development
[ArUco Marker Generator](https://chev.me/arucogen/)
## Linux
[Setting](https://drive.google.com/drive/folders/17sdCqwlKatiwnEhSNNZFgLtwJwxvsXKn?usp=sharing)
