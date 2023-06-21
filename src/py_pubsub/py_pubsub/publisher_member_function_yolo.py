from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2

# from open_manipulator_msgs.msg import open_manipulator_moving_state

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'joint_msg', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.moving_state = self.create_subscription(
            # String,
            # 'state_msg',
            # self.state_callback,
            # 10)
        # self.state_msg_subscription

    def timer_callback(self):
        msg = String()
        msg.data = '{}'.format(depth_point2)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    # def state_callback(self,msg):
        # self.robot_state = msg.open_manipulator_moving_state

def main(args=None):

    global depth_point
    global depth_point2

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    # Realsense 카메라 초기화
    pipeline = rs.pipeline()
    config = rs.config()
    
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    # Realsense 카메라 시작
    pipeline.start(config)

    align = rs.align(rs.stream.color)

    model = YOLO('/home/tomato_v6_best.pt')  # load an official detection model

    dx = []
    dy = []
    dz = []

    Detect = False

    depth_point2 = []

    try:    
        while True:
            while (Detect == False):
                depth_pixel = [0, 0]
                # 프레임 가져오기
                frames = pipeline.wait_for_frames()
                aligned_frames = align.process(frames)

                # 컬러 프레임 가져오기
                color_frame = aligned_frames.get_color_frame()
                color_image = np.asanyarray(color_frame.get_data())

                # 뎁스 프레임 가져오기
                depth_frame = aligned_frames.get_depth_frame()

                ####################################################
                # 토마토 색상 검출및 중점 픽셀값 / 뎁스 좌표값
                ####################################################
                red_tomato_list = []

                h, w, _ = color_image.shape

                color_image[:h, :int(w/4), :] = 0
                color_image[:h, (w - int(w/4)):, :] = 0
                color_image1 = color_image.copy()

                result = model.predict(color_image, conf = 0.5)[0]

                if result:
                    num_list = result.boxes.cls.tolist()

                    for idx, each in enumerate(num_list):
                        if model.names[int(each)] == "red_tomato":
                            red_tomato_list.append((result.boxes.xyxy.tolist()[idx],result.boxes.conf.tolist()[idx]))

                    # variable for search near red_tomato from center frame
                    min_distance = w
                    min_index = 0

                    # near red_tomato from center frame
                    # calculation distance from center frame to detected frame and compare distance
                    for idx2, each2 in enumerate(red_tomato_list):
                        if abs(each2[0][0]-(w/2)) < min_distance or abs(each2[0][2]-(w/2)) < min_distance:
                            if abs(each2[0][0]-(w/2)) > abs(each2[0][2]-(w/2)):
                                min_distance = abs(each2[0][2]-(w/2))
                            else:
                                min_distance = abs(each2[0][0]-(w/2))
                            min_index = idx2

                    if red_tomato_list:

                        Detect = True
                        
                        x1, y1, x2, y2 = map(int, red_tomato_list[min_index][0])

                        color_image1 = cv2.rectangle(color_image1, (x1,y1),(x2,y2), (0,255,0),2,cv2.LINE_8)

                        cx = int((x1+x2)/2)
                        cy = int((y1+y2)/2)

                        # depth = depth_frame.get_distance(cx,cy)
                        # depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                        # depth_pixel = [cx, cy]

                        color_image1 = cv2.line(color_image1, (int(w/2),0), (int(w/2),h),(255,0,0),2, cv2.LINE_8)
                        color_image1 = cv2.line(color_image1, (0,int(h/2)), (w,int(h/2)),(255,0,0),2, cv2.LINE_8)
                        color_image1 = cv2.line(color_image1, (cx-8,cy), (cx+8,cy),(0,0,0),2, cv2.LINE_8)
                        color_image1 = cv2.line(color_image1, (cx,cy-8), (cx,cy+8),(0,0,0),2, cv2.LINE_8)
                        color_image1 = cv2.circle(color_image1, (cx,cy),2,(255,255,255))

                        # depth_point2 = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth)

                        # x - 0.025, y - 0.05 (m)
                        depth_point2[0] + 0.0025
                        depth_point2[1] + 0.005

                        dx = []
                        dy = []
                        dz = []

                        for idx_x in range (cx - int((x2-x1)/4), cx + int((x2-x1)/4), 1):
                            for idx_y in range (cy - int((y2-y1)/4), cy + int((y2-y1)/4), 1):
                                depth = depth_frame.get_distance(int(idx_x),int(idx_y))
                                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                                depth_pixel = [idx_x, idx_y]
                                depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth)

                                if depth_point[2] >= 0.2 and depth_point[2] <=0.6:
                                    dx.append(depth_point[0])
                                    dy.append(depth_point[1])
                                    dz.append(depth_point[2])

                        if dx and dy:
                            depth_point2 = [sum(dx)/len(dx), sum(dy)/len(dy), sum(dz)/len(dz)]
                        else:
                            depth_point2 = [0,0,0]

                        print(depth_point2)
                        print("-------------")

                        # rclpy.spin_once(minimal_publisher)
                    else:
                            depth_point2 = [0.0,0.0,0.0]
                    cv2.imshow("test",color_image1)

                    ####################################################
                    # 종료 조건: q 키를 누르면 종료
                    if cv2.waitKey(1) == ord('q') or cv2.waitKey(1) == 27:
                        break
                print("--------a-------")
                rclpy.spin_once(minimal_publisher)

                time.sleep(1)

                if Detect == True:
                    Detect = False

                if cv2.waitKey(1) == 27:
                    break
    finally:
        # 리소스 해제
        if Detect == False:
            pipeline.stop()
        cv2.destroyAllWindows()
 
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
