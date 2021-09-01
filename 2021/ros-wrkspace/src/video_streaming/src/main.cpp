#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <string.h>
#include <csignal>
#include <vector>
#include <asio.hpp>

void SigintHandler(int sig) {
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "video_streaming");
  ros::NodeHandle nh;

  std::string server_addr;
  std::string server_port;
  ros::param::get("server_addr", server_addr);
  ros::param::get("server_port", server_port);
  
  signal(SIGINT, SigintHandler);

  cv::VideoCapture cap("../test/test.mp4");
  //cv::VideoCapture cap(0);
  //cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280); // (2560, 720), (1280, 480), (640, 240)
  //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  if (!cap.isOpened()) {
    std::cerr << "ERROR! Unable to open camera\n";
    ros::shutdown();
    return -1;
  }

  cv::Mat test_img;
  cap.read(test_img);
  const int img_rows = test_img.rows;
  const int img_cols = test_img.cols;
  const int img_type = test_img.type();


  asio::io_context io_context;
  asio::ip::tcp::resolver resolver(io_context);
  asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(server_addr, server_port);

  asio::ip::tcp::socket socket(io_context);
  socket.set_option(asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{ 500 });
  asio::error_code ec;
  asio::connect(socket, endpoints);

  for (;;) {
    asio::error_code ignored_error;
    asio::write(socket, asio::buffer(&img_rows, 4), ignored_error);
    asio::write(socket, asio::buffer(&img_cols, 4), ignored_error);
    asio::write(socket, asio::buffer(&img_type, 4), ignored_error);

    std::vector<char> buf;
    asio::error_code error;
    size_t len = socket.read_some(asio::buffer(buf), error);
  }

  ros::spin();
  return 0;
}
