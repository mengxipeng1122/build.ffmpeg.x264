

#pragma once 


extern "C" {
    #include <libavutil/avassert.h>
    #include <libavutil/channel_layout.h>
    #include <libavutil/opt.h>
    #include <libavutil/mathematics.h>
    #include <libavutil/timestamp.h>
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
    #include <libswresample/swresample.h>
}

#define SCALE_FLAGS SWS_BICUBIC

// a wrapper around a single output AVStream
typedef struct OutputStream {
    AVStream *st;
    AVCodecContext *enc;

    /* pts of the next frame that will be generated */
    int64_t next_pts;
    int samples_count;

    AVFrame *frame;
    AVFrame *tmp_frame;

    AVPacket *tmp_pkt;

    float t, tincr, tincr2;

    struct SwsContext *sws_ctx;
    struct SwrContext *swr_ctx;
} OutputStream;


#if 0
struct AviWriter
{
    protected:
        int outVideoWidth_;
        int outVideoHeight_;
        OutputStream video_st_;
        OutputStream audio_st_;
        const AVOutputFormat *fmt_;
        char *filename_;
        AVFormatContext *oc_;
        const AVCodec *audio_codec_, *video_codec_;
        bool have_video_   , have_audio_   ;
        bool encode_video_ , encode_audio_ ;
        AVDictionary *opt_ ;

    public:
        AviWriter(const char* filename, int videoWidth, int videoHeight);
        int addFileVideoFrame(unsigned char* buffer, unsigned int sz);
         ~AviWriter();
};
#else
        int init_writeavi(const char*, int width, int height);
        int frame_writeavi(unsigned char*, unsigned  int);
        void deinit_writeavi();
#endif

