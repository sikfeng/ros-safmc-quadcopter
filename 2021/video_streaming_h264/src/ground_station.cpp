#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>
#include <vector>
#include <asio.hpp>
#include <chrono>

extern "C" {
    #include <libavutil/frame.h>
    #include <libavutil/mem.h>
    #include <libavcodec/avcodec.h>
}

int main(int argc, char **argv) {

  //std::string ground_station_port = "4097";
  int ground_station_port = 4097;

  int img_metadata[3];

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


  int img_size = img_metadata[0] * img_metadata[1];
  std::cout << "Image size: " << img_size << std::endl;
  cv::Mat img = cv::Mat::zeros(img_metadata[0], img_metadata[1], img_metadata[2]);
  cv::VideoWriter writer("video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, img.size(), true);
  cv::namedWindow("ground_station_recv", 1);

  //make img continuous
  if (!img.isContinuous()) {
    img = img.clone();
  }

  std::chrono::time_point<std::chrono::high_resolution_clock> begin = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 1000; ++i) {
    try {
      //size_t len = asio::read(socket, asio::buffer(img.data, img_size));

      std::vector<uchar> buf(img_size);
      int header[1];
      size_t len = asio::read(socket, asio::buffer(header, 4));
      size_t len1 = asio::read(socket, asio::buffer(buf, header[0]));
      img = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
      cv::imshow("ground_station_recv", img);
      cv::waitKey(1);
      writer << img;
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
