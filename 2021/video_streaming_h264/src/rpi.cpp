//#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>
#include <vector>
#include <asio.hpp>
#include "h264.h"

int main(int argc, char **argv) {
  AVPacket *pkt;

  //ros::init(argc, argv, "video_streaming");
  //ros::NodeHandle nh;

  std::string ground_station_addr = "127.0.0.1";
  //std::string ground_station_addr = "6.tcp.ngrok.io";
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

  H264 encoder(img.cols, img.rows, img.cols, img.rows, true);
  //H264 decoder(img.cols, img.rows, img.cols, img.rows);
  // allocate packet to retrive encoded frame
  pkt = av_packet_alloc();

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

  // encoding loop
  int ret;

  struct {
      uint32_t size_data;
      uint32_t size_side_data;
      uint32_t side_data_type_size;
      uint32_t flags;
      uint32_t stream_index;
      uint32_t pos;
      uint32_t duration;
  } packetinfo;

  //cv::namedWindow("rpi_send", 1);
  for (int i = 0; i < 200; ++i) {
    if (i == 200)
      encoder.flushEncode();
    else
      cap >> img;

    encoder.encode(img);
    pkt = av_packet_alloc();
    while ((ret = encoder.get_packet(pkt)) >= 0) {
      packetinfo.size_data = pkt->size;
      packetinfo.size_side_data = pkt->side_data->size;
      packetinfo.side_data_type_size = sizeof(pkt->side_data->type);
      packetinfo.flags = pkt->flags;
      packetinfo.stream_index = pkt->stream_index;
      packetinfo.pos = pkt->pos;
      packetinfo.duration = pkt->duration;

      asio::write(socket, asio::buffer((void*)&packetinfo, 28));
      asio::write(socket, asio::buffer((void*)&pkt->side_data->type, sizeof(pkt->side_data->type)));
      asio::write(socket, asio::buffer(pkt->data, pkt->size));
      asio::write(socket, asio::buffer(pkt->side_data->data, pkt->side_data->size));


      std::cout << "===========================\n" << "send packet:\n";
      std::cout << "pkt_size:" << pkt->size << "\n";
      std::cout << "pkt_side_data_elems:" << pkt->side_data_elems << "\n";
      std::cout << "pkt_flags:" << pkt->flags << "\n";
      std::cout << "===========================\n" << std::endl;
      //decoder.decode(pkt);
      av_packet_free(&pkt);
      pkt = av_packet_alloc();
    }
    if (ret == AVERROR_EOF) {
      std::cerr << "fail to avcodec_receive_packet: ret=" << ret << "\n";
      break;
    }
    

    //std::vector<uchar> buf;
    //cv::imencode(".jpg", img, buf, std::vector<int>{cv::IMWRITE_JPEG_QUALITY, 30});
    //std::cout << buf.size() << std::endl;

    //int header[1];
    //header[0] = buf.size();
    //asio::write(socket, asio::buffer(header, 4));
    //asio::write(socket, asio::buffer(buf, img_size));
    //asio::write(socket, asio::buffer(img.data, img_size));

    //cv::imshow("rpi_send", img);
    //cv::waitKey(1);
  }

  socket.close();

  //ros::spin();
  return 0;
}
