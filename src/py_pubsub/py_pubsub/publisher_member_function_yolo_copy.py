from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

count = 0

depth_point = []
depth_list = []

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'joint_msg', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = '{}'.format(depth_point)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):

    global depth_point

    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    # Realsense 카메라 초기화
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    # Realsense 카메라 시작
    pipeline.start(config)

    model = YOLO('/home/tomato_v5_best.pt')  # load an official detection model

    try:
        while True:
            depth_pixel = [0, 0]
            # 프레임 가져오기
            frames = pipeline.wait_for_frames()
            # 컬러 프레임 가져오기
            color_frame = frames.get_color_frame()
            color_image = np.asanyarray(color_frame.get_data())
            # 뎁스 프레임 가져오기
            depth_frame = frames.get_depth_frame()
            ####################################################
            # 토마토 색상 검출및 중점 픽셀값 / 뎁스 좌표값
            ####################################################
            #color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

            red_tomato_list = []

            h, w, _ = color_image.shape
            color_image[:h, :int(w/4), :] = 0
            color_image[:h, (w - int(w/4)):, :] = 0

            result = model.predict(color_image)[0]

            # count = 0

            if result:
                num_list = result.boxes.cls.tolist()

                for idx, each in enumerate(num_list):
                    # if model.names[int(each)] == "red_tomato":
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

                color_image1 = color_image.copy()

                if red_tomato_list:
                    x1, y1, x2, y2 = map(int, red_tomato_list[min_index][0])

                    color_image1 = cv2.rectangle(color_image1, (x1,y1),(x2,y2), (0,255,0),2,cv2.LINE_8)
                    
                    cx = int((x1+x2)/2)
                    cy = int((y1+y2)/2)
                    
                    for idx_x in w:
                        for idx_y in h:
                            depth = depth_frame.get_distance(idx_x,idx_y)

                            if depth != 0 and depth < 0.5:
                                depth_list.append(depth_frame.get_distance(idx_x,idx_y))
                    
                    depth_mean = np.mean(depth_list)
                    
                    depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                    depth_pixel = [int(cx), int(cy)]
                    depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth_mean)
                    
                    color_image1 = cv2.line(color_image1, (int(w/2),0), (int(w/2),h),(255,0,0),2, cv2.LINE_8)

                    # depth_point.append(count)
                    # count += 1

                    # if count == 0:
                    rclpy.spin_once(minimal_publisher)
                    count += 1

                cv2.imshow("test2",result.plot())
                cv2.imshow("test",color_image1)
                
            ####################################################
            # 종료 조건: q 키를 누르면 종료
            if cv2.waitKey(1) == ord('q'):
                break
    finally:
        # 리소스 해제
        pipeline.stop()
        cv2.destroyAllWindows()

    
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
