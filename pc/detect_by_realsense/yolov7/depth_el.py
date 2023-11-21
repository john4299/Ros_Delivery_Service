import cv2
import pyrealsense2 as rs
import numpy as np
first_depth = 0

class Depth_Camera():

    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.align = rs.align(rs.stream.color)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 6)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 6)
        self.toggle = 0  # 초기값을 0으로 설정

    def execute(self):
        try:
            self.pipeline.start(self.config)
        except:
            print("카메라 연결 안됨")
            return

        try:
            while True:
                with open('/home/boma/Desktop/test.txt', 'r') as f:
                    number = f.read()
                    number = int(number)

                frames = self.pipeline.wait_for_frames()
                aligned_frames = self.align.process(frames)
                depth_frame = aligned_frames.get_depth_frame()
                x, y = 320, 240
                depth_el = round(depth_frame.get_distance(x, y), 2)
                print("엘리베이터와의 거리: ", depth_el, "m")

                first_depth = depth_el # 초기값을 변수에 저장
                if abs(depth_el - first_depth) > 0.25 : # 거리가 초기값보다 25 미만이면
                    with open('/home/boma/Desktop/test.txt', 'w') as f:
                        f.write('6')
                    print("엘리베이터 열림")

                color_image = np.asanyarray(aligned_frames.get_color_frame().get_data())
                cv2.imshow('RealSense', cv2.circle(color_image, (x, y), 2, (0, 0, 255), -1))

                # if cv2.waitKey(1) & 0xFF == ord('q'):
                #     break

                if number == 7:
                    with open('/home/boma/Desktop/test.txt', 'w') as f:
                        f.write('0')
                    break

        finally:
            self.pipeline.stop()

if __name__ == "__main__":
    depth_camera = Depth_Camera()
    depth_camera.execute()