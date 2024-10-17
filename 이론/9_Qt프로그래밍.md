# 1. Qt가 뭘까?

Qt는 크로스 플랫폼 애플리케이션 개발 프레임워크로, 주로 C++로 개발되어 있으며 다양한 운영체제(Windows, macOS, Linux, Android, iOS 등)에서 실행 가능한 그래픽 사용자 인터페이스(GUI) 애플리케이션을 개발하는 데 사용됩니다. 하지만 Qt는 단순히 GUI만을 위한 것이 아니라, 네트워크 통신, 데이터베이스 관리, 스레딩 등 다양한 기능을 포함하고 있어 다목적 개발에도 적합합니다.

### 주요 특징:

1. **크로스 플랫폼**: 하나의 코드 베이스로 여러 운영체제에서 동일한 애플리케이션을 실행할 수 있습니다. 운영체제에 맞게 코드를 수정하지 않아도 Qt가 각 플랫폼에 맞게 변환해줍니다.
2. **위젯 기반 GUI**: 다양한 UI 요소(버튼, 라벨, 텍스트 박스 등)를 쉽게 추가하고 구성할 수 있습니다. Qt 디자이너를 통해 시각적으로 GUI를 설계할 수 있고, 코드와의 통합도 원활합니다.
3. **신호와 슬롯**: Qt의 이벤트 처리 메커니즘으로, 객체 간의 비동기식 통신을 쉽게 구현할 수 있습니다. 예를 들어, 버튼을 클릭했을 때 특정 함수가 실행되도록 연결할 수 있습니다.
4. **QML**: Qt는 QML이라는 JavaScript 기반의 선언형 언어도 지원합니다. 주로 빠르고 직관적인 UI 디자인을 위해 사용되며, 애니메이션이나 동적인 UI 요소를 쉽게 구성할 수 있습니다. C++ 로직과도 통합이 가능하여 고성능 애플리케이션을 개발할 수 있습니다.
5. **모듈화**: Qt는 다양한 모듈로 구성되어 있어, 필요한 기능만 선택해서 사용할 수 있습니다. 예를 들어, QtCore(기본 기능), QtGui(그래픽 기능), QtNetwork(네트워크 기능) 등 다양한 모듈이 제공됩니다.

### Qt 사용 예시:

- **데스크톱 애플리케이션**: 크로스 플랫폼 GUI 앱(예: 텍스트 편집기, 이미지 뷰어).
- **임베디드 시스템**: 임베디드 장치의 UI 및 제어 시스템 개발.
- **모바일 애플리케이션**: Android 및 iOS용 앱 개발.

Qt는 또한 강력한 커뮤니티와 상업적 지원을 제공하여 다양한 산업군에서 널리 사용되고 있습니다.

# 2. 예시

아래 창들은 ChatGPT를 활용해서 쉽게 구현할 수 있다

### 1. 간단한 출력

![image (3)](https://github.com/user-attachments/assets/099233c2-3247-4600-b900-ffc4c4245f20)

### 2. 체크박스

![image (4)](https://github.com/user-attachments/assets/dc8cc898-2eff-4d63-95f6-958cdd8ffe88)

### 3. 세로 레이아웃

![image (5)](https://github.com/user-attachments/assets/93713489-bac9-4f4d-802c-c858488b30d3)

### 4. 가로 레이아웃

![image (6)](https://github.com/user-attachments/assets/dfd482d4-0992-46b0-90ad-46d35c026f9a)

### 5. 다이얼 조그

![image (7)](https://github.com/user-attachments/assets/a172c694-16ab-4f25-b69d-2a3b11c55af7)

### 6. 계산기

![image (8)](https://github.com/user-attachments/assets/0814aeb3-1b90-4316-bad3-379aef3b594a)

### 7. 계산기 + 다이얼 조그

![image (9)](https://github.com/user-attachments/assets/d07c3a11-b1b6-4303-95bb-e38c74a8d090)

### 8. 이미지

![image (10)](https://github.com/user-attachments/assets/06ce0223-c67b-413c-9f10-d0b1c0d05e0d)

### 9. 그림판

![image (11)](https://github.com/user-attachments/assets/5608d347-fbef-4837-bf00-4bb29a89259b)

# 3. Qt 디자이너

![image (12)](https://github.com/user-attachments/assets/3ca212fe-fb6e-4972-8455-075fae9e9639)

- 이걸로 그냥 드래그 앤 드롭으로 GUI를 만들 수 있음!
