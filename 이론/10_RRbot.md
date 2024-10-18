# RRbot

### RRbot이 뭐임?

RRBot은 "Revolute-Revolute Bot"의 약자로, 주로 **ROS(로봇 운영 체제)** 환경에서 사용되는 2자유도(2-DOF) 로봇 팔 모델입니다. 이 모델은 시뮬레이션 및 제어 알고리즘을 테스트하고 배우는 데 자주 사용됩니다.

RRBot은 두 개의 회전 관절을 가지고 있으며, 각각의 관절이 1자유도를 제공하여 **2자유도**를 구현합니다. 첫 번째 관절은 로봇의 기본 부분과 연결된 회전축을, 두 번째 관절은 첫 번째 링크와 연결된 회전축을 제공합니다. 각 관절의 회전은 **revolute joint**로 표현되며, 이것이 RRBot의 이름에 반영되어 있습니다.

주로 **Gazebo** 시뮬레이터와 함께 사용되며, URDF(Unified Robot Description Format)를 사용해 모델링됩니다. 초보자들이 ROS에서 로봇 시뮬레이션과 제어를 학습할 때 많이 활용하는 간단한 로봇입니다.

**주요 특징:**

- **2 DOF(자유도)**: 두 개의 회전 관절을 통해 두 방향으로 움직일 수 있는 로봇 팔.
- **ROS와 Gazebo**: ROS 환경에서 Gazebo 시뮬레이터를 통해 물리적 시뮬레이션이 가능.
- **학습 및 연구용**: 제어 알고리즘, 역기구학, 경로 계획 등의 학습에 사용됨.

RRBot은 간단한 구조 덕분에 로봇 공학과 제어 이론을 배우기 좋은 모델입니다.

### RRbot을 어캐 조정함?

- 메시지 타입

RRbot은 Float64MultiArray 형식을 받게 되어 있음

그리고 이걸 MultiArrayLayout과 함께 데이터를 보내게 되어 있음

이거 파악하면 topic publish하는 파이썬 파일 만들어서 실행하면 조정 가능함

```yaml
std_msgs/Float64MultiArray:
  layout:
    dim:
      - label: ""
        size: 0
        stride: 0
    data_offset: 0
  data: [joint1_position, joint2_position]
```

- publisher

이제 이 메시지 형식을 publish하는 파이썬 파일 만들면 끝 ㅋ

어캐 만들어?

클래스 만들어

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
```

```python
class RRBotJointControl(Node):
    def __init__(self):
        super().__init__('rrbot_joint_control')

        # Publisher to send joint positions using Float64MultiArray
        self.publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        
        # Timer to periodically send joint commands
        self.timer = self.create_timer(1.0, self.send_joint_command)
```

생성자 이렇게 만들어

publisher 만들어 → 누구한테 pub할지 적어줘야 함!!

timer만들고 콜백함수(일정 시간마다 호출할 함수) 가리켜

- 콜백 함수

```python
    def send_joint_command(self):
        # Create a Float64MultiArray message
        joint_positions = Float64MultiArray()

        # Create layout with dimensions
        layout = MultiArrayLayout()
        # If you want to specify dimensions, you can add MultiArrayDimension instances
        # For example: layout.dim = [MultiArrayDimension(label='positions', size=2, stride=2)]
        layout.dim = []  # Empty dimensions as per your request
        layout.data_offset = 0
        
        # Assign the layout to the message
        joint_positions.layout = layout
        
        # Assign the data representing joint positions
        joint_positions.data = [0.0, 0.0]  # Replace with desired values for joint1 and joint2

        # Publish the message
        self.publisher.publish(joint_positions)
        self.get_logger().info(f'Published joint positions: {joint_positions.data}')
```

매 초마다 호출할 함수 만들어

이 안에 뭐 있어야 돼? 메시지 틀을 만들어야겠죠

틀 만들고 그 메시지에 뭐 넣을지 적어야 겠쬬

쉽죠?

- main 함수

```python
def main(args=None):
    rclpy.init(args=args)
    
    rrbot_joint_control = RRBotJointControl()
    
    # Spin the node to keep it alive until manually shut down
    rclpy.spin(rrbot_joint_control)
    
    # Shutdown ROS 2 when done
    rrbot_joint_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

메인 함수에 클래스 만들어서 spin으로 실행하면 됨 ㅅㄱㅇ
