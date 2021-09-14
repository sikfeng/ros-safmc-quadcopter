//
// Created by orange on 14/9/21.
//

#include <iostream>
#include "h264.h"

H264::H264(int frame_width, int frame_height, int encode_width, int encode_height, bool encode) {
    this->frame_height = frame_height;
    this->frame_width = frame_width;
    /* find the mpeg1 video encoder */
    if (encode) codec = avcodec_find_encoder(codec_id);
    else codec = avcodec_find_decoder(codec_id);
    if (!codec) {
        fprintf(stderr, "Codec not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(codec);
    if (!c) {
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }

    /* put sample parameters */
    c->bit_rate = 400000;
    /* resolution must be a multiple of two */
    c->width = encode_width;
    c->height = encode_height;
    /* frames per second */
    //c->time_base = (AVRational){1,25};
    /* emit one intra frame every ten frames
    * check frame pict_type before passing frame
    * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
    * then gop_size is ignored and the output of encoder
    * will always be I frame irrespective to gop_size
    */
    c->gop_size = 10;
    c->max_b_frames = 1;
    c->pix_fmt = AV_PIX_FMT_YUV420P;
    c->time_base = av_inv_q(dst_fps);
    c->framerate = dst_fps;

    av_dict_set(&dict, "preset", "fast", 0);

    /* open it */
    if (avcodec_open2(c, codec, &dict) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }

    // initialize sample scaler
    swsctx_mat_to_frame = sws_getContext(
            frame_width, frame_height, AV_PIX_FMT_BGR24,
            c->width, c->height, c->pix_fmt,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
    swsctx_frame_to_mat = sws_getContext(c->width, c->height, c->pix_fmt,
            frame_width, frame_height, AV_PIX_FMT_BGR24,
            SWS_BILINEAR,NULL, NULL, NULL);

    if (!swsctx_mat_to_frame) {
        std::cerr << "fail to sws_getContext mat to frame";
        throw std::runtime_error("fail to sws_getContext mat to frame");
    }

    if (!swsctx_frame_to_mat) {
        std::cerr << "fail to sws_getContext frame to mat";
        throw std::runtime_error("fail to sws_getContext frame to mat");
    }

    // allocate frame buffer for encoding
    frame = av_frame_alloc();
    if (encode) {
        frame->width = c->width;
        frame->height = c->height;
        frame->format = static_cast<int>(c->pix_fmt);
        ret = av_frame_get_buffer(frame, 32);
        if (ret < 0) {
            std::cerr << "fail to av_frame_get_buffer: ret=" << ret;
            throw std::runtime_error("fail to av_frame_get_buffer");
        }
    }

    frame_converted = av_frame_alloc();
    av_image_alloc(frame_converted->data, frame_converted->linesize, frame_width, frame_height, AV_PIX_FMT_BGR24, 32);
    av_image_fill_arrays(frame_converted->data, frame_converted->linesize, NULL, AV_PIX_FMT_BGR24, frame_width, frame_height, 32);

    std::cout<< "vcodec:  " << codec->name << "\n"
             << "size:    " << c->width << 'x' << c->height << "\n"
             << "fps:     " << av_q2d(c->framerate) << "\n"
             << "pixfmt:  " << av_get_pix_fmt_name(c->pix_fmt) << "\n"
             << std::flush;
}

void H264::encode(cv::Mat image) {
    // convert cv::Mat(OpenCV) to AVFrame(FFmpeg)
    const int stride[4] = { static_cast<int>(image.step[0]) };
    sws_scale(swsctx_mat_to_frame, &image.data, stride, 0, image.rows, frame->data, frame->linesize);
    frame->pts = frame_pts++;
    // encode video frame
    ret = avcodec_send_frame(c, frame);
    if (ret < 0) {
        std::cerr << "fail to avcodec_send_frame: ret=" << ret << "\n";
        throw std::runtime_error("fail to avcodec send frame");
    }
}

void H264::decode(AVPacket *packet) {
    ret = avcodec_send_packet(c, packet);
    if (ret < 0) {
        std::cerr << "fail to avcodec_send_packet: ret=" << ret << "\n";
        throw std::runtime_error("fail to avcodec send packet");
    }
}

int H264::get_packet(AVPacket *packet) {
    int out = avcodec_receive_packet(c, packet);
    packet->duration = 1;
    return out;
}

int H264::get_frame(cv::Mat *image) {
    int out = avcodec_receive_frame(c, frame);
    sws_scale(swsctx_frame_to_mat, frame->data, frame->linesize, 0, c->height, frame_converted->data, frame_converted->linesize);
    cv::Mat mat(frame_height, frame_width, CV_8UC3, frame_converted->data[0], frame_converted->linesize[0]);
    image = &mat;
    return out;
}

void H264::flushEncode() {
    ret = avcodec_send_frame(c, NULL);
    if (ret < 0) {
        std::cerr << "fail to avcodec_send_frame: ret=" << ret << "\n";
        throw std::runtime_error("fail to avcodec send frame");
    }
}
