//#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <csignal>
#include <vector>
#include <asio.hpp>

extern "C" {
  #include <libavformat/avformat.h>
  #include <libavcodec/avcodec.h>
  #include <libavutil/avutil.h>
  #include <libavutil/pixdesc.h>
  #include <libswscale/swscale.h>
  #include <libavutil/imgutils.h>
}

int main(int argc, char **argv) {
  AVCodecID codec_id = AV_CODEC_ID_H264;
  AVCodec *codec;
  AVCodecContext *c= NULL;
  int i, ret, x, y, got_output;
  AVFrame *frame;
  AVPacket *pkt;
  AVDictionary* dict = NULL;

  /* find the mpeg1 video encoder */
  codec = avcodec_find_encoder(codec_id);
  if (!codec) {
      fprintf(stderr, "Codec not found\n");
      exit(1);
  }

  c = avcodec_alloc_context3(codec);
  if (!c) {
      fprintf(stderr, "Could not allocate video codec context\n");
      exit(1);
  }

  //ros::init(argc, argv, "video_streaming");
  //ros::NodeHandle nh;

  //std::string ground_station_addr = "127.0.0.1";
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

  /* put sample parameters */
  c->bit_rate = 400000;
  /* resolution must be a multiple of two */
  c->width = img_metadata[1];
  c->height = img_metadata[0];
  /* frames per second */
  c->time_base = (AVRational){1,25};
  /* emit one intra frame every ten frames
  * check frame pict_type before passing frame
  * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
  * then gop_size is ignored and the output of encoder
  * will always be I frame irrespective to gop_size
  */
  c->gop_size = 10;
  c->max_b_frames = 1;
  c->pix_fmt = AV_PIX_FMT_YUV420P;

  printf("fmt: %d\n fmt_vidcodec: %d\n", AV_PIX_FMT_BGR24, codec->pix_fmts[0]);

  if (codec_id == AV_CODEC_ID_H264)
      av_dict_set(&dict, "preset", "fast", 0);

  /* open it */
  if (avcodec_open2(c, codec, &dict) < 0) {
      fprintf(stderr, "Could not open codec\n");
      exit(1);
  } 

  // initialize sample scaler
  SwsContext* swsctx = sws_getContext(
      img_metadata[1], img_metadata[0], AV_PIX_FMT_BGR24,
      c->width, c->height, c->pix_fmt,
      SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (!swsctx) {
      std::cerr << "fail to sws_getContext";
      return 2;
  }

  // allocate frame buffer for encoding
  frame = av_frame_alloc();
  frame->width = c->width;
  frame->height = c->height;
  frame->format = static_cast<int>(c->pix_fmt);
  ret = av_frame_get_buffer(frame, 32);
  if (ret < 0) {
      std::cerr << "fail to av_frame_get_buffer: ret=" << ret;
      return 2;
  }

  // allocate packet to retrive encoded frame
  pkt = av_packet_alloc();

  std::cout<< "vcodec:  " << codec->name << "\n"
      << "size:    " << c->width << 'x' << c->height << "\n"
      << "fps:     " << av_q2d(c->framerate) << "\n"
      << "pixfmt:  " << av_get_pix_fmt_name(c->pix_fmt) << "\n"
      << std::flush;

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
  int64_t frame_pts = 0;
  unsigned nb_frames = 0;

  //cv::namedWindow("rpi_send", 1);
  for (int i = 0; i < 1000; ++i) {
    cap >> img;

    // convert cv::Mat(OpenCV) to AVFrame(FFmpeg)
    const int stride[4] = { static_cast<int>(img.step[0]) };
    sws_scale(swsctx, &img.data, stride, 0, img.rows, frame->data, frame->linesize);
    frame->pts = frame_pts++;
    // encode video frame
    ret = avcodec_send_frame(c, frame);
    if (ret < 0) {
      std::cerr << "fail to avcodec_send_frame: ret=" << ret << "\n";
      break;
    }
    ret = avcodec_receive_packet(c, pkt);
    if (ret != 0) {
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
