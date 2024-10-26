## RGBD 카메라

### 개요

- RGB(Red, Green,  Blue) 카메라와 Depth(깊이) 센서를 결합한 장치
- 색상 정보를 포함한 2D 이미지를 캡처하면서 동시에 각 픽셀의 깊이 정보를 수집함
- Microsoft의 Kinect, Intel의 RealSense, ASUS의 Xtion Pro 등이 있음

![1](https://github.com/user-attachments/assets/191b5800-6aba-4099-9e4b-8438e3b9301f)

![2](https://github.com/user-attachments/assets/c71f4856-7b8f-45e4-be66-ee0f90b82b88)

### 주요기능

- 컬러 이미지 캡처 (RGB) : 일반 카메라처럼 컬러 이미지를 캡처
- 깊이 정보 수집 (Depth) : 각 픽셀의 거리를 측정하여 깊이 정보를 수집
- 동기화된 데이터 : RGB 이미지와 깊이 정보가 동기화되어 있음

### 활용분야

- 로봇 공학 : 로봇이 주변 환경을 인식하고 상호작용하는 데 사용됨
- 증강 현실(AR) 및 가상 현실(VR) : 현실 세계와 가상 객체를 정밀하게 결합하기 위해 사용
- 3D 모델링 및 스캐닝 : 물체나 환경의 3D 모델을 만들 때 사용
- 제스처 인식 : 사용자의 손 동작을 인식하여 제어할 수 있게 함
- 안전 및 보안 : 얼굴 인식 및 사용자 인증 시스템에 활용

### Intel RealSense SDK

- 다양한 프로그래밍 언어(C++, Python, C# 등)를 지원하는 라이브러리와 API를 제공
- 데이터 스트림 처리 : 컬러 이미지, 깊이 이미지, 적외선 이미지, 3D 포인트 클라우드 등의 데이터 스트림을 처리하고 활용할 수 있는 기능을 제공함
- 모듈화된 아키텍처 : 여러 모듈로 구성되어 있어 필요한 기능만 선택적으로 사용할 수 있음

(예 : 얼굴 인식 모듈, 제스처 인식 모듈, 모션 추적 모듈 등)

![3](https://github.com/user-attachments/assets/248bf1c3-878f-4da3-9159-b64926f929b0)

![4](https://github.com/user-attachments/assets/33ed8cce-cfce-4a06-ae52-ab6734330f1c)

### Intel RealSense Viewer

- Intel RealSense Viewer 어플리케이션에서 카메라를 세팅할 수 있음

![5](https://github.com/user-attachments/assets/107b798b-ff5e-4d18-9027-1bfe8f1518b2)

## Open CV

### 개요

- Open Source Computer Vision Library
- 컴퓨터 비전 및 이미지 처리 작업을 위해 개발된 오픈 소스 라이브러리
- C++, Python, Java 및 MATLAB 등 다양한 프로그래밍 언어를 지원
- 이미지 및 비디오 처리와 관련된 많은 기능을 제공

### 대표적인 기능

1. **이미지 및 영상 처리 (Image and Video Processing)**

- OpenCV는 다양한 필터, 변환, 조작 기능을 통해 이미지를 효율적으로 처리할 수 있음
- 예를 들어, 이미지의 색상 변환, 필터링(예: Gaussian, Median 필터), 엣지 검출(예: Canny Edge Detection), 히스토그램 평활화 등이 포함
- 이러한 기능은 컴퓨터 비전의 전처리 과정에서 많이 사용됨

2. **객체 검출 및 인식 (Object Detection and Recognition)**

- 객체 검출과 인식은 OpenCV의 주요 기능 중 하나로, 얼굴 인식, 물체 추적, 차량 검출 등을 포함
- Haar Cascade Classifier와 같은 미리 학습된 모델을 사용하여 얼굴이나 눈을 인식할 수 있고, 딥러닝과의 결합을 통해 더욱 정교한 객체 검출 및 인식을 구현할 수 있음

3. **영상 분석 (Video Analysis)**

- OpenCV는 모션 감지, 배경 차감, 추적 등과 같은 영상 분석 기능을 제공
- 예를 들어, 배경 차감 알고리즘을 사용하여 고정된 카메라에서 움직이는 물체를 검출하거나, Optical Flow 알고리즘을 통해 물체의 이동 경로를 추적할 수 있음
- 이는 보안, 스포츠 분석, 자율 주행 등 여러 응용 분야에서 유용하게 사용됨

![6](https://github.com/user-attachments/assets/1072c037-9a56-4aa7-818f-791832f998f0)

---

## API 활용하기

### RealSense Python 패키지 설치

```bash
pip install pyrealsense2
```

- 설치 시 특정 파이썬 버전이 필요할 경우, 다음과 같이 가상환경을 구축하여 특정 파이썬버전 환경을 구축할 수 있음

```bash
py -3.11 -m venv myenv
```

### Pyrealsense2 설치 확인

- 100개의 프레임을 캡처 후 실시간으로 데이터를 수집하고 각 프레임의 메타데이터를 출력하는 예제

```python
import pyrealsense2 as rs
pipe = rs.pipeline()
profile = pipe.start()
try:
	for i in range(0, 100):
		frames = pipe.wait_for_frames()
		for f in frames:
			print(f.profile)
finally:
	pipe.stop()
```

![7](https://github.com/user-attachments/assets/545df25d-53ec-4fed-8fd7-889546abb665)

### 특정 컬러 검출하기

```bash
pip install opencv-python
```

1. 라이브러리

![8](https://github.com/user-attachments/assets/c5c885a9-88c0-4dd4-9f60-b201ac1cf68e)

1. 스트림 및 색 영역 설정

![9](https://github.com/user-attachments/assets/75776efc-02ac-4e5a-8b64-d642578775fe)

1. BGR → HSV 색상영역을 활용

![10](https://github.com/user-attachments/assets/0eaf0ac5-c7a7-42c2-9144-976d95690b5c)

### RGB 색 공간

- **구성**: RGB 색 공간은 Red(빨강), Green(초록), Blue(파랑) 세 가지 색상의 조합으로 색을 표현합니다. 각 색 성분은 0에서 255 범위의 값으로 나타나며, 세 값의 조합으로 하나의 색상을 표현합니다.
- **특징**: RGB는 대부분의 디지털 화면에서 사용되는 색 공간으로, 빛의 삼원색을 기반으로 한 가산혼합 방식을 사용합니다.
- **단점**: RGB 색 공간은 색상(Hue), 채도(Saturation), 명도(Value) 정보를 명확히 구분하지 못하기 때문에, 색상의 변화를 분석하거나 특정 색을 탐지하는 작업에서는 비효율적일 수 있습니다.

### HSV 색 공간

- **구성**: HSV 색 공간은 Hue(색상), Saturation(채도), Value(명도)로 구성됩니다.
    - **Hue (색상)**: 색상의 종류를 나타내며, 각도(0-360°)로 표현됩니다. 예를 들어, 빨강은 0°, 초록은 120°, 파랑은 240°로 나타납니다.
    - **Saturation (채도)**: 색의 선명도를 나타내며, 0에서 100% 범위로 표현됩니다. 0%에 가까울수록 회색에 가깝고, 100%는 가장 선명한 색을 나타냅니다.
    - **Value (명도)**: 색의 밝기를 나타내며, 0에서 100% 범위로 표현됩니다. 0%에 가까울수록 검은색에 가까워지고, 100%는 가장 밝은 상태를 의미합니다.
- **특징**: HSV는 색상, 채도, 명도가 구분되어 있어 색상 탐지나 필터링에 유리합니다. 예를 들어, 색상(Hue) 범위만으로 특정 색을 쉽게 추출할 수 있어, 이미지 처리나 컴퓨터 비전 작업에서 자주 사용됩니다.

### RGB와 HSV의 비교

| 색 공간 | 구성 요소 | 주요 용도 및 특징 |
| --- | --- | --- |
| **RGB** | Red, Green, Blue | 화면 표시용 색상 공간. 가산 혼합 방식으로 색상을 구성. 일반적인 디지털 이미지 표현에 사용됨. |
| **HSV** | Hue, Saturation, Value | 색상과 채도, 밝기를 구분해 표현. 색상 탐지와 이미지 필터링 작업에 유리하며, 사람이 직관적으로 색을 이해하기 쉬움. |

HSV는 특히 색상 분석이 중요한 컴퓨터 비전 및 이미지 처리 작업에 적합하며, RGB는 일반적인 디지털 장치에서 색을 표현하는 데 널리 사용됩니다.

### 빨간색 HSV 범위

```python
# 빨간색 범위 정의 (HSV)
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])
```
