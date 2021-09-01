#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>
#include <vector>
#include <asio.hpp>
#include <chrono>

int main(int argc, char **argv) {
  std::vector<std::string> test_files{"../test/test720_pad.mp4","../test/test480_pad.mp4","../test/test240_pad.mp4"};
  for (int j = 0; j < test_files.size(); ++j) {
    std::cout << test_files[j] << std::endl;

    cv::VideoCapture cap(test_files[j]);
    cv::Mat img;
    cap >> img;

    std::vector<uchar> buf;
    {
    // IMDECODE
    std::chrono::time_point<std::chrono::high_resolution_clock> begin1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
      cv::imencode(".jpg", img, buf, std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 30});
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> curr1 = std::chrono::high_resolution_clock::now();
    int imencode_time = std::chrono::duration_cast<std::chrono::milliseconds>(curr1-begin1).count();
    std::cout << "cv::imencode() jpg " << imencode_time << " ms, 10000 reps\n";

    // IMDECODE
    std::chrono::time_point<std::chrono::high_resolution_clock> begin2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
      img = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> curr2 = std::chrono::high_resolution_clock::now();
    int imdecode_time = std::chrono::duration_cast<std::chrono::milliseconds>(curr2-begin2).count();
    std::cout << "cv::imdecode() jpg " << imdecode_time << " ms, 10000 reps\n";
    }

    {
    // IMDECODE
    std::chrono::time_point<std::chrono::high_resolution_clock> begin1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
      cv::imencode(".bmp", img, buf, std::vector<int>{});
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> curr1 = std::chrono::high_resolution_clock::now();
    int imencode_time = std::chrono::duration_cast<std::chrono::milliseconds>(curr1-begin1).count();
    std::cout << "cv::imencode() bmp " << imencode_time << " ms, 10000 reps\n";

    // IMDECODE
    std::chrono::time_point<std::chrono::high_resolution_clock> begin2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
      img = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> curr2 = std::chrono::high_resolution_clock::now();
    int imdecode_time = std::chrono::duration_cast<std::chrono::milliseconds>(curr2-begin2).count();
    std::cout << "cv::imdecode() bmp " << imdecode_time << " ms, 10000 reps\n";
    }
  }

  return 0;
}
