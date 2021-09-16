//
// Created by orange on 14/9/21.
//

#include <iostream>
#include "h264.h"

H264::H264(int frame_width, int frame_height, int encode_width, int encode_height, bool encode) {
    this->frame_height = frame_height;
    this->frame_width = frame_width;
    /* find the mpeg1 video encoder */
    if (encode) {
      this->codec = avcodec_find_encoder(this->codec_id);
    } else {
      this->codec = avcodec_find_decoder(this->codec_id);
    }
    if (!this->codec) {
        fprintf(stderr, "Codec not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(this->codec);
    if (!this->c) {
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }

    /* put sample parameters */
    this->c->bit_rate = 400000;
    /* resolution must be a multiple of two */
    this->c->width = encode_width;
    this->c->height = encode_height;
    /* frames per second */
    //c->time_base = (AVRational){1,25};
    /* emit one intra frame every ten frames
    * check frame pict_type before passing frame
    * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
    * then gop_size is ignored and the output of encoder
    * will always be I frame irrespective to gop_size
    */
    this->c->gop_size = 10;
    this->c->max_b_frames = 1;
    this->c->pix_fmt = AV_PIX_FMT_YUV420P;
    this->c->time_base = av_inv_q(this->dst_fps);
    this->c->framerate = this->dst_fps;

    av_dict_set(&this->dict, "preset", "fast", 0);

    /* open it */
    if (avcodec_open2(this->c, this->codec, &this->dict) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }

    // initialize sample scaler
    swsctx_mat_to_frame = sws_getContext(
            frame_width, frame_height, AV_PIX_FMT_BGR24,
            this->c->width, this->c->height, this->c->pix_fmt,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
    swsctx_frame_to_mat = sws_getContext(this->c->width, this->c->height, this->c->pix_fmt,
            frame_width, frame_height, AV_PIX_FMT_BGR24,
            SWS_BILINEAR,NULL, NULL, NULL);

    if (!this->swsctx_mat_to_frame) {
        std::cerr << "fail to sws_getContext mat to frame";
        throw std::runtime_error("fail to sws_getContext mat to frame");
    }

    if (!this->swsctx_frame_to_mat) {
        std::cerr << "fail to sws_getContext frame to mat";
        throw std::runtime_error("fail to sws_getContext frame to mat");
    }

    // allocate frame buffer for encoding
    this->frame = av_frame_alloc();
    if (encode) {
        this->frame->width = this->c->width;
        this->frame->height = this->c->height;
        this->frame->format = static_cast<int>(this->c->pix_fmt);
        this->ret = av_frame_get_buffer(this->frame, 32);
        if (this->ret < 0) {
            std::cerr << "fail to av_frame_get_buffer: ret=" << this->ret;
            throw std::runtime_error("fail to av_frame_get_buffer");
        }
    }

    this->frame_converted = av_frame_alloc();
    av_image_alloc(this->frame_converted->data, this->frame_converted->linesize, frame_width, frame_height, AV_PIX_FMT_BGR24, 32);
    //av_image_fill_arrays(this->frame_converted->data, this->frame_converted->linesize, NULL, AV_PIX_FMT_BGR24, frame_width, frame_height, 32);

    std::cout<< "vcodec:  " << this->codec->name << "\n"
             << "size:    " << this->c->width << 'x' << this->c->height << "\n"
             << "fps:     " << av_q2d(this->c->framerate) << "\n"
             << "pixfmt:  " << av_get_pix_fmt_name(this->c->pix_fmt) << "\n"
             << std::flush;
}

void H264::encode(const cv::Mat &image) {
    // convert cv::Mat(OpenCV) to AVFrame(FFmpeg)
    const int stride[4] = { static_cast<int>(image.step[0]) };
    sws_scale(this->swsctx_mat_to_frame, &image.data, stride, 0, image.rows, this->frame->data, this->frame->linesize);
    this->frame->pts = this->frame_pts++;
    // encode video frame
    this->ret = avcodec_send_frame(this->c, this->frame);
    if (this->ret < 0) {
        std::cerr << "fail to avcodec_send_frame: ret=" << ret << "\n";
        throw std::runtime_error("fail to avcodec send frame");
    }
}

void H264::decode(const AVPacket *packet) {
    this->ret = avcodec_send_packet(this->c, packet);
    if (ret < 0) {
        std::cerr << "fail to avcodec_send_packet: ret=" << ret << "\n";
        throw std::runtime_error("fail to avcodec send packet");
    }
}

int H264::get_packet(AVPacket *packet) {
    int out = avcodec_receive_packet(this->c, packet);
    packet->duration = 1;
    return out;
}

int H264::get_frame(cv::Mat *image) {
    int out = avcodec_receive_frame(this->c, this->frame);
    std::cout << "avcodec_receive_frame: " << out << std::endl;
    if (out != 0) {
      return out;
    }
    this->frame_converted->data[0] = (uint8_t *)image->data;
    sws_scale(this->swsctx_frame_to_mat, this->frame->data, this->frame->linesize, 0, this->c->height, this->frame_converted->data, this->frame_converted->linesize);
    return out;
}

void H264::flushEncode() {
    this->ret = avcodec_send_frame(this->c, NULL);
    if (this->ret < 0) {
        std::cerr << "fail to avcodec_send_frame: ret=" << this->ret << "\n";
        throw std::runtime_error("fail to avcodec send frame");
    }
}
