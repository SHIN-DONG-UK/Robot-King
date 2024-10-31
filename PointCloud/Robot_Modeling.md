### URDF(Unified Robot Description Format)

- 로봇의 물리적 모델을 정의하기 위한 XML 기반의 파일 형식
- 로봇의 관절, 링크, 모터, 센서 등의 구조와 특성을 기술
- 로봇 시뮬레이션, 시각화 및 제어를 위한 로봇 모델링에 활용

![1](https://github.com/user-attachments/assets/a7625de9-10cc-4fa1-abe5-592dff1d924c)

### URDF 파일 구조

- 기본적으로 robot. link, joint 요소로 구성
- robot 태크
    - 로봇의 최상위 요소, 이름을 정의
- link 태그
    - 로봇의 각 부품(링크)을 정의하는 요소
    - visual, collision, inertia 서브 태그 사용 가능
    - 시각적, 물리적 특성 정의
- joint 태그
    - 링크 간의 관계를 설정
    - 회전이나 슬라이딩 등의 운동을 정의
    - 종류 : fixed, revolute, prismatic, continuous, floating, planar

![2](https://github.com/user-attachments/assets/0ab05fa5-2539-4f1a-93b1-64a102ef348b)

### XML Macro

- ROS 환경에서 URDF를 좀 더 간편하게 작성할 수 있도록 돕는 매크로 언어
- 사실상 URDF의 매크로 버전
- URDF로 컴파일되기 전에 매크로를 사용하여 구성 요소들을 동적으로 생성
- 복잡한 파일을 단순화함

![3](https://github.com/user-attachments/assets/ab9356c6-969c-46de-ad50-664c0dbe637a)

### Xacro 사용의 필요성

- 기본적인 URDF는 XML로 작성
- 복잡한 로봇을 정의할 때 많은 중복된 코드가 생길 수 있음
    - ex) 여러 개의 링크라 조인트가 유사한 속성(모양, 크기, 위치 등)을 가질 때
- 반복해서 쓰면 코드가 길어지고 관리하기 어려워짐
- Xacro를 사용하면
    - 코드 중복을 제거
    - 유연한 매개변수화
        - ex) 링크 크기나 조인트의 위치를 변수로 정의
    - 반복 작업 감소

### Xacro 주요 기능

- 변수 사용
    - 파라미터화된 URDF 모델을 만들 때 유용
    - 변수는 로봇의 특정 부분의 크기나 모양, 물리적 속성 등을 정의
    - 나중에 여러 곳에서 재사용 가능

![4](https://github.com/user-attachments/assets/accb94cb-83ec-4679-af65-34c3f803a07a)

- 매크로 정의 및 호출
    - 특정 기능이나 동작을 여러 번 정의해야 할 때 유용
    - Xacro에서는 매크로를 정의하고 이를 필요할 때마다 호출할 수 있음
- 조건문
    - 단순한 XML을 넘어서는 조건문과 반복문도 지원

![5](https://github.com/user-attachments/assets/8b8332c3-a678-4117-ace7-365d2f8df4f0)

- 반복문

![6](https://github.com/user-attachments/assets/d6688c87-720d-4b2f-aca7-b079aad06c05)

### link 태그

- 로봇의 각 부품을 정의하는 요소
- 각각의 링크는 로봇의 고정된 부분을 나타내며, 이는 로봇의 바디나 팔, 다리 등의 구성요소가 됨
- 하위 태그
    - visual : 링크의 시각적 표현을 정의
        - geometry : 링크의 모양을 정의(ex : box, cylinder, sphere, mesh 등)
        - material : 링크의 색상 또는 텍스처
    - Collision : 충돌 감지를 위한 물리적 표현을 정의. 주로 시뮬레이션 충돌을 감지하는데 사용
        - geometry : 시각적 표현과 유사하지만 충돌 감지를 위해 사용
    - inertial : 링크의 질량과 관성 모멘트를 정의
        - mass : 링크의 질량을 정의
        - inertia : 링크의 관성 모멘트를 정의

### <joint> 태그

- 로봇의 두 링크 간의 연결을 정의
- 로봇의 움직임을 나타내는 중요한 요소
- 속성
    - name : 조인트의 이름 정의
    - type : 조인트의 종류를 지정(fixed, revolute, prismatic, continuos 등)
- 하위 태그
    - parent : 연결된 부모 링크를 지정
    - child : 연결된 자식 링크를 저장
    - origin : 부모 링크와 자식 링크 간의 상대적 위치와 회전을 정의
    - axis : 조인트의 회전 또는 이동 방향을 정의
    - limit : 조인트의 운동 범위와 제한(최대 힘, 속도, 회전각, 이동거리 등)을 설정

### 링크와 조인트의 관계

- 링크와 조인트는 로봇의 구조를 정의하는 기본적인 구성 요소
- 링크는 로봇의 구성 요소(부품)를 나타내며, 이들 간의 관계를 조인트가 정의
- 조인트는 두 링크를 연결하고, 그 연결된 링크 간의 상대적인 움직임을 설정

![7](https://github.com/user-attachments/assets/b3e1a84f-47cd-4cff-a540-592e56f60790)

### RealSense D435i 설치

https://github.com/IntelRealSense/realsense-ros

위 방법 따라하기

### URDF 파일 만들고 rviz에서 동작해보기

![8](https://github.com/user-attachments/assets/c8453b67-4d30-4434-a0af-31b8dd3dacbd)

```
1. pkg 생성
ros2 pkg create --build-type ament_python urdf_exmaple

2. 패키지 내 urdf파일 생성 -> urdf 정의하기
<로봇>
	base link 정의
	
	joint1 정의
	
	link 정의
	
	joint2 정의
	
	...
</로봇>
3. rviz 폴더 생성
rviz 파일 이 폴더에 저장해놓기

4. launch 폴더 생성
launch.py 작성

5. 실행
ros2 launch urdf_example launch.py
```

### 2DOF_ROBOT.xacro

```
<robot name="two_link_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:property name="link_radius" value="0.1" />
    <xacro:property name="link_length" value="1.0" />
    
    <link name="base_link">
    </link>

    <joint name="joint1" type="revolute">
        <parent link="base_link" />
        <child link="second_link" />
        <origin xyz="0 0 0.5" rpy="0 0 0" />
        <axis xyz="1 0 0" />
        <limit lower="-1.57" upper="1.57" effort="1000" velocity="1.0" />
    </joint>

    <link name="second_link">
        <visual>
            <origin xyz="0 0 ${link_length/2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link_radius}" length="${link_length}" />
            </geometry>
            <material name="blue">
                <color rgba="0.0 0.0 1.0 1.0" />
            </material>
        </visual>
    </link>

    <joint name="joint2" type="revolute">
        <parent link="second_link" />
        <child link="third_link" />
        <origin xyz="0 0 1" rpy="0 0 0" />
        <axis xyz="1 0 0" />
        <limit lower="-1.57" upper="1.57" effort="1000" velocity="1.0" />
    </joint>

    <link name="third_link">
        <visual>
            <origin xyz="0 0 ${link_length/2}" rpy="0 0 0" />
            <geometry>
                <cylinder radius="${link_radius}" length="${link_length}" />
            </geometry>
            <material name="green">
                <color rgba="0.0 1.0 0.0 1.0" />
            </material>
        </visual>
    </link>
</robot>
```

### launch.py

```python
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    rviz_config_file = '/home/donguk/point_ws/src/urdf_practice/rviz/urdf_practice.rviz'

    xacro_file = '/home/donguk/point_ws/src/urdf_practice/urdf/two_link_robot.xacro'

    doc = xacro.process_file(xacro_file)
    robot_description = doc.toxml()

    return LaunchDescription([
        # robot_state_publisher node
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # joint_state_publisher_gui node
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),

        # Rviz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )

    ])
```
