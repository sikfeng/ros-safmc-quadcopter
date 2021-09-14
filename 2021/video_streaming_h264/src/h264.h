//
// Created by orange on 14/9/21.
//

#ifndef VIDEO_STREAMING_H264_H
#define VIDEO_STREAMING_H264_H

#include <opencv2/core/mat.hpp>

extern "C" {
    #include <libavformat/avformat.h>
    #include <libavcodec/avcodec.h>
    #include <libavutil/avutil.h>
    #include <libavutil/pixdesc.h>
    #include <libswscale/swscale.h>
    #include <libavutil/imgutils.h>
}

class H264 {
private:
    AVCodecID codec_id = AV_CODEC_ID_H264;
    AVCodec *codec;
    AVCodecContext *c= nullptr;
    int ret;
    AVFrame *frame;
    AVDictionary* dict = nullptr;

    SwsContext* swsctx_mat_to_frame = nullptr;
    SwsContext* swsctx_frame_to_mat = nullptr;

    int64_t frame_pts = 0;
    unsigned nb_frames = 0;

    int frame_height, frame_width;

public:
    H264(int frame_width, int frame_height, int encode_width, int encode_height);
    void encode(cv::Mat image);
    void decode(AVPacket *packet);

    int get_packet(AVPacket *packet);
    int get_frame(cv::Mat image);

    void flushEncode();
};

#endif //VIDEO_STREAMING_H264_H
