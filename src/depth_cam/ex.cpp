#include <librealsense2/rs.hpp>
#include <iostream>

int main()
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // 파이프라인 시작
    pipe.start(cfg);

    while (true)
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // 물체의 좌표 설정
        int targetX = 320;  // 물체의 X 좌표
        int targetY = 240;  // 물체의 Y 좌표

        // 좌표의 깊이 값 확인
        float depth_meters = depth_frame.get_distance(targetX, targetY);

        // 좌표 출력
        std::cout << "물체의 좌표: X=" << targetX << ", Y=" << targetY << ", Depth=" << depth_meters << "m" << std::endl;
    }

    return 0;
}
