//#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>
#include <vector>
#include <asio.hpp>


int main(int argc, char **argv) {

  //ros::init(argc, argv, "video_streaming");
  //ros::NodeHandle nh;

  std::string ground_station_addr = "6.tcp.ngrok.io";
  std::string ground_station_port = "17104";
  //ros::param::get("ground_station_addr", ground_station_addr);
  //ros::param::get("ground_station_port", ground_station_port);
  
  //signal(SIGINT, SigintHandler);

  cv::VideoCapture cap("../test/test240_pad.mp4");
  //cv::VideoCapture cap(0);
  //cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280); // (2560, 720), (1280, 480), (640, 240)
  //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
  if (!cap.isOpened()) {
    std::cerr << "ERROR! Unable to open camera\n";
    //ros::shutdown();
    return -1;
  }

  cv::Mat img;
  cap >> img;
  int img_metadata[3];
  img_metadata[0] = img.rows;
  img_metadata[1] = img.cols;
  img_metadata[2] = img.type();
  int img_size = img.rows * img.cols;

  std::cout << img_metadata[0] << " rows, " 
            << img_metadata[1] << " cols, " 
            << img_metadata[2] << " type" << std::endl;
  std::cout << "Image size: " << img_size << std::endl;


  asio::io_context io_context;
  asio::ip::tcp::resolver resolver(io_context);
  asio::ip::tcp::resolver::results_type endpoints = resolver.resolve(ground_station_addr, ground_station_port);

  asio::ip::tcp::socket socket(io_context);
  while (true) {
    try {
      asio::connect(socket, endpoints);
    } catch (const asio::system_error& ex) {
      std::cerr << "ERROR! Unable to connect to " << ground_station_addr << ':' << ground_station_port << std::endl;
      //ros::Duration(0.2).sleep();
      continue;
    }
    break;
  }

  std::cout << "Connected!" << std::endl;
  //socket.set_option(asio::detail::socket_option::integer<SOL_SOCKET, SO_RCVTIMEO>{ 500 });

  while (true) {
    //asio::error_code ignored_error;
    asio::write(socket, asio::buffer(img_metadata, 12));

    std::vector<char> buf(5);
    size_t len = asio::read(socket, asio::buffer(buf, 3));
    std::string s(buf.begin(), buf.end());
    std::cout << s << std::endl;
    break;
    if (s == "ACK") {
      break;
    }
  }

  //cv::namedWindow("rpi_send", 1);
  for (int i = 0; i < 1000; ++i) {
    cap >> img;
    asio::write(socket, asio::buffer(img.data, img_size*img.elemSize()));

    //cv::imshow("rpi_send", img);
    //cv::waitKey(1);
  }

  socket.close();

  //ros::spin();
  return 0;
}
