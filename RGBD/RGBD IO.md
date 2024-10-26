### 빨간색, 초록색, 파란색 박스 인식해보기

- 피부색 기반 손 인식의 기본적인 동작을 구현하는 과정

```
1. RealSense 카메라 설정 : pyrealsense2 라이브러리를 사용하여 RealSense 카메라를 초기화
2. 컬러 스트림 설정
3. 프레임 가져오기 : 카메라로부터 컬러 프레임을 받아온다
4. 색 범위 설정 : HSV 컬러 스페이스에서 색에 해당하는 범위를 설정
5. 마스크 생성 : 설정한 색 범위를 기반으로 마스크를 생성
6. 노이즈 제거 : 마스크에 모폴로지 연산과 블러를 적용하여 노이즈를 제거
7. 윤곽선 검출 : 마스크에서 윤곽선을 검출
```

### 1. RealSense 카메라 설정

```python
import pyrealsense2 as rs
import numpy as np
import cv2

# Realsense 카메라 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.eanble_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 스트리밍 시작
pipeline.start(config)
```

### 2. 프레임 가져오기

```python
# 프레임 수신
frames = pipline.wait_for_frames()
color_frame = frames.get_color_frame()
depth_frame = frames.get_depth_frame()
# Color 프레임과 Depth 프레임을 numpy 배열로 변환
color_image = np.asanyarray(color_frame.get_data())
depth_image = np.asanyarray(depth_frame.get_data())
# BGR에서 HSV로 변환
hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
```

### 3. 색 범위 설정

```python
# 색상 범위 설정
# 빨간색
lower_red1 = np.array([0, 120, 70])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 120, 70])
upper_red2 = np.array([180, 255, 255])
# 초록색
lower_green = np.array([40, 40, 40])
upper_green = np.array([70, 255, 255])
# 파란색
lower_blue = np.array([100, 150, 0])
upper_blue = np.array([140, 255, 255])
```

### 4. 마스크 생성

```python
mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
mask_red = mask_red1 + mask_red2

mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
```

### 5. 노이즈 제거

- 모폴로지 연산으로 작은 노이즈 제거

```python
kernel = np.ones((5, 5), np.uint8)
mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)
```

![1](https://github.com/user-attachments/assets/f782370f-6ce8-46d4-9920-2dd378317808)

### 6. 윤곽선 검출

- cv2.findContours 함수는 이미지에서 윤곽선을 검출하는 데 사용
- 이진화된 흑백이미지를 입력받아 윤곽선 정보를 반환
- cv2.findContours
    
  ![2](https://github.com/user-attachments/assets/90eafd00-7489-40b4-ba99-6db92872af10)
    
    - image : 이진화된 입력 이미지
        - 일반적으로 cv2.inRange나 cv2.threshold 같은 함수로 만든 마스크 이미지가 사용됨
    - mode : 윤곽선 검색 모드
        - 윤곽선을 찾는 방법을 결정
        
      ![3](https://github.com/user-attachments/assets/45b26788-ee90-4173-bec7-8e77982cd1b3)
        
    - method : 윤곽선 근사화 방법
        - 윤곽선을 어떻게 표현할지 결정
        
      ![3](https://github.com/user-attachments/assets/fe4a649f-890b-475b-b743-616795ae16a2)
        
- 반환값
    - Contours : 검출된 윤곽선들의 리스트. 각 윤곽선은 점들의 배열로 표현
    - Hierarchy : 윤곽선의 계층 구조를 나타내는 배열. mode가 cv2.RETR_TREE일 때 유용

```python
# 컨투어(윤곽선) 검출
contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# 빨간색 상자 검출
for contour in contours_red:
    area = cv2.contourArea(contour)
    if area > 500:  # 최소 크기 필터
        # 외곽선 그리기
        cv2.drawContours(color_image, [contour], -1, (0, 255, 0), 2)

        # 중심점 계산
        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2

        # 중심점 그리기
        cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

        # 중심점의 depth 정보 가져오기
        depth_value = depth_frame.get_distance(cx, cy)
        print(f"Red Box detected - Center: (x: {cx}, y: {cy}), Depth: {depth_value:.3f} meters")
        print("1")  # 빨간 상자 검출

# 초록색 상자 검출
for contour in contours_green:
    area = cv2.contourArea(contour)
    if area > 500:  # 최소 크기 필터
        # 외곽선 그리기
        cv2.drawContours(color_image, [contour], -1, (255, 0, 0), 2)

        # 중심점 계산
        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2

        # 중심점 그리기
        cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1)

        # 중심점의 depth 정보 가져오기
        depth_value = depth_frame.get_distance(cx, cy)
        print(f"Green Box detected - Center: (x: {cx}, y: {cy}), Depth: {depth_value:.3f} meters")
        print("2")  # 초록 상자 검출

# 파란색 상자 검출
for contour in contours_blue:
    area = cv2.contourArea(contour)
    if area > 500:  # 최소 크기 필터
        # 외곽선 그리기
        cv2.drawContours(color_image, [contour], -1, (0, 255, 255), 2)

        # 중심점 계산
        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2

        # 중심점 그리기
        cv2.circle(color_image, (cx, cy), 5, (255, 0, 0), -1)

        # 중심점의 depth 정보 가져오기
        depth_value = depth_frame.get_distance(cx, cy)
        print(f"Blue Box detected - Center: (x: {cx}, y: {cy}), Depth: {depth_value:.3f} meters")
        print("3")  # 파란 상자 검출

# 결과 이미지 표시
cv2.imshow('Box Detection', color_image)
```

![5](https://github.com/user-attachments/assets/5efe5244-4d0a-4abc-a01d-f8b153756d35)

### 7. Depth 정보 취득

- 해당하는 객체를 윤곽선 처리하여 중심점의 깊이 정보를 취득
- 단위는 미터(m)로 출력됨

```python
if contours:
	# 가장 큰 윤곽을 찾는다
	C = max(contours, key=cv2.contourArea)
	x, y, w, h = cv2.boundingRect(C)
	
	# 상자 중심의 깊이 정보 가져오기
	depth = depth_frame.get_distance(x + w//2, y + h//2)
	
	# 상자를 이미지에 그리기
	cv2.rectangle(color_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
	cv2.putText(color_image, f"Depth: {depth:.2f} meters", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
```
