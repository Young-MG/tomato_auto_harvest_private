from ultralytics import YOLO
import cv2
from ultralytics.yolo.utils.plotting import Annotator
import time
import numpy as np
import pyrealsense2 as rs
import logging


# 로그 출력 비활성화
logging.disable(logging.INFO)
model = YOLO('tomato_v4_best.pt')


# Realsense 카메라 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# Realsense 카메라 시작
pipeline.start(config)

while True:
    # 프레임 가져오기
    frames = pipeline.wait_for_frames()
    
    # 컬러 프레임 가져오기
    color_frame = frames.get_color_frame()
    color_image = np.asarray(color_frame.get_data())

    # 뎁스 프레임 가져오기
    depth_frame = frames.get_depth_frame()
    depth_image = np.asarray(depth_frame.get_data())



    img = [color_image]
    results = model.predict(img)


    for r in results:
        
        annotator = Annotator(color_image)
        boxes = r.boxes
        for box in boxes:
            b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
            c = box.cls
            annotator.box_label(b, model.names[int(c)])

            t_np = b.cpu().detach().numpy()
            tomato_color_x=((t_np[0]+t_np[2])/2)
            tomato_color_y=((t_np[1]+t_np[3])/2)
            cla=int(c)

            print("Color_X:",tomato_color_x,"Color_Y:",tomato_color_y,cla)

            # 깊이 정보 가져오기
            depth = depth_frame.get_distance(int(tomato_color_x), int(tomato_color_y))
            print("Depth:", depth)
            
            # 검출된 픽셀값에 대한 뎁스 카메라에서의 좌표값을 출력
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
            depth_pixel = [int(tomato_color_x), int(tomato_color_y)]
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth)
            print("Depth Point:", depth_point)
            print("")

            # 깊이 이미지에 검출된 픽셀값에 대한 원을 그림
            cv2.circle(depth_image, (int(tomato_color_x), int(tomato_color_y)), 5, (0, 0, 255), -1)



    time.sleep(0.01)
    frame = annotator.result()
    # 컬러 이미지 출력
    cv2.imshow("Color Image", color_image)

    # 뎁스 이미지 출력
    depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 뎁스 이미지 시각화를 위한 스케일 조정
    cv2.imshow("Depth Image", depth_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 리소스 해제
pipeline.stop()
cv2.destroyAllWindows()