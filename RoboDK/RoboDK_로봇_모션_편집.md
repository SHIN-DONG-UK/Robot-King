# RoboDK란?

### 로봇 시뮬레이션 툴이다

- 다양한 로봇 라이브러리와 API를 지원
- 여러 종류의 로봇을 단일 플랫폼에서 실행할 수 있음

### 예시) 로봇 레이아웃
![image](https://github.com/user-attachments/assets/5ed0a914-a0ba-49eb-8af5-1433bc72fb53)


### 예시) 상자 이동
![image](https://github.com/user-attachments/assets/7825710d-6530-40b7-9be4-08626772a723)

### 로봇 불러오기

- 온라인 라이브러리를 활용하여 RoboDK에서 지원하는 로봇을 불러올 수 있음

![image](https://github.com/user-attachments/assets/37181c77-b42e-4fea-837a-fedf75d54f1f)

### 조작법

![image](https://github.com/user-attachments/assets/01323a1c-c950-4865-a6b2-c36a13636118)

### 객체명과 좌표계

![image](https://github.com/user-attachments/assets/1fddbfc3-049a-4353-88e8-527ee99776e2)

- 위 처럼 객체를 생성하면 객체에 대한 좌표계가 같이 생성되고, 이 좌표계를 기준으로 목표 지점이 정의됨

### 타겟 좌표 지정하기

1. 수동으로 설정

![image](https://github.com/user-attachments/assets/65bef9a2-0bf1-49e6-ad74-304265678a34)

1. 로봇을 옮겨놓고 현재 위치를 지정할 수 있음

![image](https://github.com/user-attachments/assets/d885fcce-0e77-4021-955c-f375f1f9ae4d)

### 프로그램 생성

![image](https://github.com/user-attachments/assets/a1d52187-2039-459a-b236-3e67813d582a)

```
- 프로그램 생성 버튼을 눌러 특정 모션으로 동작하는 프로그램을
만들 수 있음
- 모션 명령어는 MoveJ, MoveL, MoveC를 지원함

MoveJ
- 로봇을 조인트 공간에서 이동시키는 명령어
- 로봇이 관절 궤적을 따라 부드럽게 이동함

MoveL
- 로봇을 직교 공간에서 직선 경로를 따라 이동시키는 명령어
- 로봇의 끝단(End Effector)이 직선 궤적을 그리며 이동함
- 경로의 정확성이 중요한 경우에 사용됨
- 경로를 직선으로 유지하는 것이 특징임 -> 로봇이 특정 작업을 수행
할 때, 끝단이 직선으로 움직여야 하는 경우(접착제 도포, 용접, 절단)
유용함

MoveC
- 로봇이 원호(Circular) 경로를 따라 이동하는 명령어
- 로봇의 끝단(End Effector)이 두 지점을 거쳐 원호 형태로 이동하
도록 제어
```
