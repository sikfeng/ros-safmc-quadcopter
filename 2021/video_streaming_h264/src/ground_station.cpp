#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>
#include <vector>
#include <asio.hpp>
#include <chrono>
#include "h264.h"

extern "C" {
    #include <libavutil/frame.h>
    #include <libavutil/mem.h>
    #include <libavcodec/avcodec.h>
}

int main(int argc, char **argv) {
    //std::string ground_station_port = "4097";
  int ground_station_port = 17104;

  int img_metadata[3];

    struct {
        uint32_t size_data;
        uint32_t size_side_data;
        uint32_t side_data_type_size;
        uint32_t flags;
        uint32_t stream_index;
        uint32_t pos;
        uint32_t duration;
        uint8_t end;
    } packetinfo;

  asio::io_context io_context;
  asio::ip::tcp::acceptor acceptor(io_context, asio::ip::tcp::endpoint(asio::ip::tcp::v4(), ground_station_port));
  asio::ip::tcp::socket socket(io_context);

  // start reading for connections
  while (true) {
      asio::error_code ec;
      socket.close(ec);
      //socket.open(asio::ip::tcp::v4(), ec);
      acceptor.accept(socket);
      try {
        asio::mutable_buffer mut_buf = asio::buffer(img_metadata);
        size_t len = asio::read(socket, asio::buffer(mut_buf, 12));
        std::cout << len << " bytes read\n";
        memcpy(&img_metadata, (char*)mut_buf.data(), asio::buffer_size(mut_buf));
        std::cout << img_metadata[0] << " rows, " 
  	        << img_metadata[1] << " cols, " 
  		<< img_metadata[2] << " type" << std::endl;
      } catch (const asio::system_error& ex) {
        continue;
      }
    
    // send ACK to start video transmission
    asio::write(socket, asio::buffer("ACK", 3));
    break;
  }

  int width = img_metadata[1], height = img_metadata[0];
  int img_size = img_metadata[0] * img_metadata[1];
  std::cout << "Image size: " << img_size << std::endl;
  cv::Mat img = cv::Mat::zeros(img_metadata[0], img_metadata[1], img_metadata[2]);
  cv::VideoWriter writer("video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, img.size(), true);
  cv::namedWindow("ground_station_recv", 1);

  //make img continuous
  if (!img.isContinuous()) {
    img = img.clone();
  }

  H264 decoder(width, height, width,height, false);
  AVPacket *pkt;
  AVPacketSideDataType type;
  uint8_t buf[img_size*2];
  uint8_t side_data[1000];

  int ret, cont = 1, i = 0;

  std::chrono::time_point<std::chrono::high_resolution_clock> begin = std::chrono::high_resolution_clock::now();
  while (cont) {
    try {
      //size_t len = asio::read(socket, asio::buffer(img.data, img_size));
      pkt = av_packet_alloc();
      size_t len = asio::read(socket, asio::buffer((void*)&packetinfo, 29));
      size_t len1 = asio::read(socket, asio::buffer((void*)&type, packetinfo.side_data_type_size));
      size_t len2 = asio::read(socket, asio::buffer(buf, packetinfo.size_data));
      size_t len3 = asio::read(socket, asio::buffer(side_data, packetinfo.size_side_data));

      std::cout << "recv: " << len << "bytes, " << len1 << "bytes, " << len2 << "bytes, " << len3 << "bytes" <<std::endl;
      av_packet_from_data(pkt, buf, static_cast<int>(packetinfo.size_data));
      av_packet_add_side_data(pkt, type, side_data, packetinfo.size_side_data);
      pkt->duration = packetinfo.duration;
      pkt->pos = packetinfo.pos;
      pkt->stream_index = static_cast<int>(packetinfo.stream_index);
      pkt->flags = static_cast<int>(packetinfo.flags);

      if (packetinfo.end == 1) {
          cont = 0;
      }

      std::cout << "===========================\n" << "recv packet:\n";
      std::cout << "pkt_size:" << pkt->size << "\n";
      std::cout << "pkt_side_data_elems:" << pkt->side_data_elems << "\n";
      std::cout << "pkt_flags:" << pkt->flags << "\n";
      std::cout << "===========================\n" << std::endl;

      decoder.decode(pkt);
      cv::waitKey(10);
      while ((ret = decoder.get_frame(&img)) >= 0) {
          i++;
          cv::imshow("ground_station_recv", img);
          cv::waitKey(10);
          writer << img;
      }
      if (ret == AVERROR_EOF) {
        std::cerr << "fail to avcodec_receive_frame: ret=" << ret << "\n";
        break;
      }
    } catch (const asio::system_error& ex) {
      std::cout << "Error when receiving frame " << i << std::endl;
      std::cout << ex.what() << std::endl;
      continue;
    }
    std::chrono::time_point<std::chrono::high_resolution_clock> curr = std::chrono::high_resolution_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(curr-begin).count();
    if (i % 100 == 0) {
      std::cout << (double)i / ((double)diff * std::chrono::milliseconds::period::num / std::chrono::milliseconds::period::den) << " FPS\n";
    }
  }

  writer.release();
  socket.close();

  //ros::spin();
  return 0;
}
