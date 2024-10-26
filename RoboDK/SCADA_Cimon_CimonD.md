### CimonD

![image](https://github.com/user-attachments/assets/0723bcce-de97-4ae2-86d1-b74caca8ff41)

```
장점
- 파워포인트로 박스(틀) 만들어서 복붙가능함
- 빠른 개발 가능
```

### 데이터베이스

- 데이터베이스에 변수를 선언해 놓으면 이걸로 프로그램 안에서 상태를 선언하거나 뭐 변수를 출력하거나 할 수 있음

### I/O 디바이스

![image](https://github.com/user-attachments/assets/214ccc05-9a06-4744-94c7-a725315c649d)

- Modbus TCP Slave를 선언하면 여기서 TCP 통신으로 화면 업데이트를 할 수 있음
- 파이썬 Modbus TCP 통신을 활용해서 IP랑 port 설정해서 데이터 줄 수 있음

### 예시 코드(모드버스 음수 표현, 소수 표현)

```python
# Type help("robodk.robolink") or help("robodk.robomath") for more information
# Press F5 to run the script
# Documentation: https://robodk.com/doc/en/RoboDK-API.html
# Reference:     https://robodk.com/doc/en/PythonAPI/robodk.html
# Note: It is not required to keep a copy of this file, your Python script is saved with your RDK project
from pymodbus.client import ModbusTcpClient
# You can also use the new version of the API:
from robodk import robolink    # RoboDK API
from robodk import robomath    # Robot toolbox
RDK = robolink.Robolink()

# Forward and backwards compatible use of the RoboDK API:
# Remove these 2 lines to follow python programming guidelines
from robodk import *      # RoboDK API
from robolink import *    # Robot toolbox
# Link to RoboDK
# RDK = Robolink()

RB1 = RDK.Item('RB1', itemtype=2)

def move_robot(robot_object, pose):
    robot_object.MoveJ(pose)
    read_coils_and_registers(RB1, 100)

def read_coils_and_registers(robot_object, std_register=99):
    # Modbus TCP 클라이언트 생성
    client = ModbusTcpClient('127.0.0.1', port=502)
    # 서버에 접속
    connection = client.connect()

    if connection:
        a = robot_object.Joints().tolist()

        print(a)

        
        for i in range(0,6):
            b = int(a[i] * 10)
            if a[i] < 0:
                b += 32768*2
            client.write_register(std_register + i, b, 1)

        # 연결 종료
        client.close()
    else:
        print("Unable to connect to the Modbus server.")

# 클라이언트 코드 실행
if __name__ == "__main__":
    p0 = [47.581198, 26.726422, 58.524246, -85.303997, -101.092563, 3.989284]
    move_robot(RB1, p0)

```

```
- RoboDK에 로봇 관절 각도를 명령하고, 그 각도를 읽어서 CimonD에 보여주는 예시 코드
- 모드버스는 소수를 보낼 수 없음 -> 10을 곱해서 뻥튀기 한 다음 CimonD에서 10나눠서
보여주면 됨
- 모드버스는 음수를 보낼 수 없음 -> 비트 연산을 고려해서 보수를 더해준 다음 값을
전송하면 됨 (Int16의 경우 32768 * 2를 더해서 보내면 됨)
```
