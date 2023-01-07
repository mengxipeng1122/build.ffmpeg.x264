


/**
 * @file
 * libavformat API example.
 *
 * Output a media file in any supported libavformat format. The default
 * codecs are used.
 * @example muxing.c
 */

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "utils.h"
#include "aviwriter.hpp"

#define STREAM_FRAME_RATE 60 
#define STREAM_PIX_FMT    AV_PIX_FMT_RGB24 // AV_PIX_FMT_YUV420P /* default pix_fmt */
#define STREAM_SRC_FMT    AV_PIX_FMT_RGB565LE /* src pix_fmt */


static void log_packet(const AVFormatContext *fmt_ctx, const AVPacket *pkt)
{
    AVRational *time_base = &fmt_ctx->streams[pkt->stream_index]->time_base;

    LOG_INFOS("pts:%s pts_time:%s dts:%s dts_time:%s duration:%s duration_time:%s stream_index:%d",
           av_ts2str(pkt->pts), av_ts2timestr(pkt->pts, time_base),
           av_ts2str(pkt->dts), av_ts2timestr(pkt->dts, time_base),
           av_ts2str(pkt->duration), av_ts2timestr(pkt->duration, time_base),
           pkt->stream_index);
}

static int write_frame(AVFormatContext *fmt_ctx, AVCodecContext *c, AVStream *st, AVFrame *frame, AVPacket *pkt)
{
    int ret;

    // send the frame to the encoder
    ret = avcodec_send_frame(c, frame);
    if (ret < 0) {
        LOG_ERRS("Error sending a frame to the encoder: %s", av_err2str(ret));
    }

    while (ret >= 0) {
        ret = avcodec_receive_packet(c, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            break;
        else if (ret < 0) {
            LOG_ERRS("Error encoding a frame: %s", av_err2str(ret));
        }

        /* rescale output packet timestamp values from codec to stream timebase */
        av_packet_rescale_ts(pkt, c->time_base, st->time_base);
        pkt->stream_index = st->index;

        /* Write the compressed frame to the media file. */
        //log_packet(fmt_ctx, pkt);
        ret = av_interleaved_write_frame(fmt_ctx, pkt);
        /* pkt is now blank (av_interleaved_write_frame() takes ownership of
         * its contents and resets pkt), so that no unreferencing is necessary.
         * This would be different if one used av_write_frame(). */
        if (ret < 0) {
            LOG_ERRS("Error while writing output packet: %s", av_err2str(ret));
        }
    }

    return ret == AVERROR_EOF ? 1 : 0;
}

/* Add an output stream. */
static void add_stream(OutputStream *ost, AVFormatContext *oc,
                       const AVCodec **codec,
                       enum AVCodecID codec_id)
{
    AVCodecContext *c;
    int i;

    /* find the encoder */
    *codec = avcodec_find_encoder(codec_id);
    if (!(*codec)) {
        LOG_ERRS( "Could not find encoder for '%s'", avcodec_get_name(codec_id));
    }

    ost->tmp_pkt = av_packet_alloc();
    if (!ost->tmp_pkt) {
        LOG_ERRS( "Could not allocate AVPacket");
    }

    ost->st = avformat_new_stream(oc, NULL);
    if (!ost->st) {
        LOG_ERRS("Could not allocate stream");
    }
    ost->st->id = oc->nb_streams-1;
    c = avcodec_alloc_context3(*codec);
    if (!c) {
        LOG_ERRS("Could not alloc an encoding context");
    }
    ost->enc = c;

    /* Some formats want stream headers to be separate. */
    if (oc->oformat->flags & AVFMT_GLOBALHEADER)
        c->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

}

/**************************************************************/
/* audio output */
static AVFrame *alloc_audio_frame(enum AVSampleFormat sample_fmt,
                                  uint64_t channel_layout,
                                  int sample_rate, int nb_samples)
{
    AVFrame *frame = av_frame_alloc();
    int ret;

    if (!frame) {
        LOG_ERRS("Error allocating an audio frame");
    }

    frame->format = sample_fmt;
    frame->channel_layout = channel_layout;
    frame->sample_rate = sample_rate;
    frame->nb_samples = nb_samples;

    if (nb_samples) {
        ret = av_frame_get_buffer(frame, 0);
        if (ret < 0) {
            LOG_ERRS("Error allocating an audio buffer");
        }
    }

    return frame;
}

static void open_audio(AVFormatContext *oc, const AVCodec *codec,
                       OutputStream *ost, AVDictionary *opt_arg)
{
    AVCodecContext *c;
    int nb_samples;
    int ret;
    AVDictionary *opt = NULL;

    c = ost->enc;

    /* open it */
    //av_dict_copy(&opt, opt_arg, 0);
    ret = avcodec_open2(c, codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        LOG_ERRS("Could not open audio codec: %s", av_err2str(ret));
    }

    /* init signal generator */
    ost->t     = 0;
    ost->tincr = 2 * M_PI * 110.0 / c->sample_rate;
    /* increment frequency by 110 Hz per second */
    ost->tincr2 = 2 * M_PI * 110.0 / c->sample_rate / c->sample_rate;

    //if (c->codec->capabilities & AV_CODEC_CAP_VARIABLE_FRAME_SIZE)
    //    nb_samples = 10000;
    //else
    //    nb_samples = c->frame_size;
    nb_samples = 534;

    ost->frame     = alloc_audio_frame(c->sample_fmt, c->channel_layout,
                                       c->sample_rate, nb_samples);
    ost->tmp_frame = alloc_audio_frame(AV_SAMPLE_FMT_S16, c->channel_layout,
                                       c->sample_rate, nb_samples);

    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        LOG_ERRS("Could not copy the stream parameters");
    }

    /* create resampler context */
    ost->swr_ctx = swr_alloc();
    if (!ost->swr_ctx) {
        LOG_ERRS("Could not allocate resampler context");
    }

    /* set options */
    av_opt_set_int       (ost->swr_ctx, "in_channel_count",   c->channels,       0);
    av_opt_set_int       (ost->swr_ctx, "in_sample_rate",     c->sample_rate,    0);
    av_opt_set_sample_fmt(ost->swr_ctx, "in_sample_fmt",      AV_SAMPLE_FMT_S16, 0);
    av_opt_set_int       (ost->swr_ctx, "out_channel_count",  c->channels,       0);
    av_opt_set_int       (ost->swr_ctx, "out_sample_rate",    c->sample_rate,    0);
    av_opt_set_sample_fmt(ost->swr_ctx, "out_sample_fmt",     c->sample_fmt,     0);

    /* initialize the resampling context */
    if ((ret = swr_init(ost->swr_ctx)) < 0) {
        LOG_ERRS("Failed to initialize the resampling context");
    }
}

/* Prepare a 16 bit dummy audio frame of 'frame_size' samples and
 * 'nb_channels' channels. */
static AVFrame *get_audio_frame(OutputStream *ost)
{
    AVFrame *frame = ost->tmp_frame;
    int j, i, v;
    int16_t *q = (int16_t*)frame->data[0];


    for (j = 0; j <frame->nb_samples; j++) {
        v = (int)(sin(ost->t) * 10000);
        for (i = 0; i < ost->enc->channels; i++)
            *q++ = v;
        ost->t     += ost->tincr;
        ost->tincr += ost->tincr2;
    }

    frame->pts = ost->next_pts;
    ost->next_pts  += frame->nb_samples;

    return frame;
}

/*
 * encode one audio frame and send it to the muxer
 * return 1 when encoding is finished, 0 otherwise
 */
static int write_audio_frame(AVFormatContext *oc, OutputStream *ost)
{
    AVCodecContext *c;
    AVFrame *frame;
    int ret;
    int dst_nb_samples;

    c = ost->enc;

    frame = get_audio_frame(ost);

    if (frame) {
        /* convert samples from native format to destination codec format, using the resampler */
        /* compute destination number of samples */
        dst_nb_samples = av_rescale_rnd(swr_get_delay(ost->swr_ctx, c->sample_rate) + frame->nb_samples,
                                        c->sample_rate, c->sample_rate, AV_ROUND_UP);
        av_assert0(dst_nb_samples == frame->nb_samples);

        /* when we pass a frame to the encoder, it may keep a reference to it
         * internally;
         * make sure we do not overwrite it here
         */
        ret = av_frame_make_writable(ost->frame);
        if (ret < 0) {
            LOG_ERRS("Error while make writable");
        }

        /* convert to destination format */
        ret = swr_convert(ost->swr_ctx,
                          ost->frame->data, dst_nb_samples,
                          (const uint8_t **)frame->data, frame->nb_samples);
        if (ret < 0) {
            LOG_ERRS("Error while converting");
        }
        frame = ost->frame;

        frame->pts = av_rescale_q(ost->samples_count, (AVRational){1, c->sample_rate}, c->time_base);
        ost->samples_count += dst_nb_samples;
    }

    return write_frame(oc, c, ost->st, frame, ost->tmp_pkt);
}

/**************************************************************/
/* video output */

static AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
{
    AVFrame *picture;
    int ret;

    picture = av_frame_alloc();
    if (!picture)
        return NULL;

    picture->format = pix_fmt;
    picture->width  = width;
    picture->height = height;

    /* allocate the buffers for the frame data */
    ret = av_frame_get_buffer(picture, 0);
    if (ret < 0) {
        LOG_ERRS("Could not allocate frame data.");
    }

    return picture;
}

static void open_video(AVFormatContext *oc, const AVCodec *codec,
                       OutputStream *ost, AVDictionary *opt_arg)
{
    int ret;
    AVCodecContext *c = ost->enc;
    AVDictionary *opt = NULL;

    av_dict_copy(&opt, opt_arg, 0);

    av_dict_set(&opt, "preset", "slow", 0);
    av_dict_set(&opt, "crf", "22", 0);
    //av_dict_set(&opt, "qscale:v", "3", NULL);

    /* open the codec */
    ret = avcodec_open2(c, codec, &opt);
    av_dict_free(&opt);
    if (ret < 0) {
        LOG_ERRS("Could not open video codec: %s", av_err2str(ret));
    }

    /* allocate and init a re-usable frame */
    ost->frame = alloc_picture(c->pix_fmt, c->width, c->height);
    if (!ost->frame) {
        LOG_ERRS("Could not allocate video frame");
    }

    /* If the output format is not STREAM_SRC_FMT, then a temporary STREAM_SRC_FMT
     * picture is needed too. It is then converted to the required
     * output format. */
    ost->tmp_frame = NULL;
    if (c->pix_fmt != STREAM_SRC_FMT) {
        ost->tmp_frame = alloc_picture(STREAM_SRC_FMT, c->width, c->height);
        if (!ost->tmp_frame) {
            LOG_ERRS("Could not allocate temporary picture");
        }
    }

    /* copy the stream parameters to the muxer */
    ret = avcodec_parameters_from_context(ost->st->codecpar, c);
    if (ret < 0) {
        LOG_ERRS("Could not copy the stream parameters");
    }
}

/* Prepare a dummy image. */
static void fill_yuv_image(AVFrame *pict, int frame_index,
                           int width, int height)
{
    int x, y, i;

    i = frame_index;

    /* Y */
    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
            pict->data[0][y * pict->linesize[0] + x] = x + y + i * 3;

    /* Cb and Cr */
    for (y = 0; y < height / 2; y++) {
        for (x = 0; x < width / 2; x++) {
            pict->data[1][y * pict->linesize[1] + x] = 128 + y + i * 2;
            pict->data[2][y * pict->linesize[2] + x] = 64 + x + i * 5;
        }
    }
}

static AVFrame *get_video_frame(OutputStream *ost)
{
    AVCodecContext *c = ost->enc;

//    /* check if we want to generate more frames */
//    if (av_compare_ts(ost->next_pts, c->time_base,
//                      STREAM_DURATION, (AVRational){ 1, 1 }) > 0)
//        return NULL;

    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(ost->frame) < 0) {
        LOG_ERRS("make writable faild ");
    }

    if (c->pix_fmt != STREAM_SRC_FMT) {
        /* as we only generate a STREAM_SRC_FMT picture, we must convert it
         * to the codec pixel format if needed */
        if (!ost->sws_ctx) {
            ost->sws_ctx = sws_getContext(c->width, c->height,
                                          STREAM_SRC_FMT,
                                          c->width, c->height,
                                          c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ost->sws_ctx) {
                LOG_ERRS( "Could not initialize the conversion context");
            }
        }
        fill_yuv_image(ost->tmp_frame, ost->next_pts, c->width, c->height);
        sws_scale(ost->sws_ctx, (const uint8_t * const *) ost->tmp_frame->data,
                  ost->tmp_frame->linesize, 0, c->height, ost->frame->data,
                  ost->frame->linesize);
    } else {
        fill_yuv_image(ost->frame, ost->next_pts, c->width, c->height);
    }

    ost->frame->pts = ost->next_pts++;

    return ost->frame;
}

/*
 * encode one video frame and send it to the muxer
 * return 1 when encoding is finished, 0 otherwise
 */
static int write_video_frame(AVFormatContext *oc, OutputStream *ost)
{
    return write_frame(oc, ost->enc, ost->st, get_video_frame(ost), ost->tmp_pkt);
}

static void close_stream(AVFormatContext *oc, OutputStream *ost)
{
    avcodec_free_context(&ost->enc);
    av_frame_free(&ost->frame);
    av_frame_free(&ost->tmp_frame);
    av_packet_free(&ost->tmp_pkt);
    sws_freeContext(ost->sws_ctx);
    swr_free(&ost->swr_ctx);
}


#if 0
/**************************************************************/
/* media file output */
int init_writeavi()


//     while (encode_video || encode_audio) {
//         /* select the stream to encode */
//         if (encode_video &&
//             (!encode_audio || av_compare_ts(video_st.next_pts, video_st.enc->time_base,
//                                             audio_st.next_pts, audio_st.enc->time_base) <= 0)) {
//             encode_video = !write_video_frame(oc, &video_st);
//         } else {
//             encode_audio = !write_audio_frame(oc, &audio_st);
//         }
//     }
// 

static AVFrame *get_file_video_frame(OutputStream *ost, char* fn)
{
    AVCodecContext *c = ost->enc;

//    /* check if we want to generate more frames */
//    if (av_compare_ts(ost->next_pts, c->time_base,
//                      STREAM_DURATION, (AVRational){ 1, 1 }) > 0)
//        return NULL;

    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(ost->frame) < 0) {
        LOG_ERRS("make writable failed ");
    }

    if (c->pix_fmt != STREAM_SRC_FMT) {
        /* as we only generate a YUV420P picture, we must convert it
         * to the codec pixel format if needed */
        if (!ost->sws_ctx) {
            ost->sws_ctx = sws_getContext(c->width, c->height,
                                          STREAM_SRC_FMT,
                                          c->width, c->height,
                                          c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ost->sws_ctx) {
                LOG_ERRS("Could not initialize the conversion context");
            }
        }
        /* Y */
        // for (y = 0; y < height; y++)
        //     for (x = 0; x < width; x++)
        //         pict->data[0][y * pict->linesize[0] + x] = x + y + i * 3;
        AVFrame *pict = ost->tmp_frame;
        FILE* fp = fopen(fn,"rb");
        if(fp)
        {
            AVCodecContext *c = ost->enc;
            int height = c->height;
            int y;
            int linesz= pict->linesize[0];
            for (y = 0; y < height; y++)
            {
                int read = fread(&(pict->data[0][y * pict->linesize[0]]), 1, linesz, fp);
                if(read != linesz) LOG_ERRS(" read failed with file %s %d/%d", fn, read, linesz);
            }
            fclose(fp);
        }
        else{
            LOG_ERRS(" can not open file %s", fn);
        }
        //fill_yuv_image(ost->tmp_frame, ost->next_pts, c->width, c->height);
        sws_scale(ost->sws_ctx, (const uint8_t * const *) ost->tmp_frame->data,
                  ost->tmp_frame->linesize, 0, c->height, ost->frame->data,
                  ost->frame->linesize);
    } else {
        fill_yuv_image(ost->frame, ost->next_pts, c->width, c->height);
    }

    ost->frame->pts = ost->next_pts++;

    return ost->frame;
}

/* Prepare a 16 bit dummy audio frame of 'frame_size' samples and
 * 'nb_channels' channels. */
static AVFrame *get_file_audio_frame(OutputStream *ost, char* fn)
{
    AVFrame *frame = ost->tmp_frame;
    int j, i, v;
    int16_t *q = (int16_t*)frame->data[0];

    FILE* fp = fopen(fn,"rb");
    if(fp)
    {
        int read = fread(q, 4, frame->nb_samples, fp);
        if(read != frame->nb_samples) LOG_ERRS(" read failed with file %s %d/%d", fn, read, frame->nb_samples);
        fclose(fp);
    }


    frame->pts = ost->next_pts;
    ost->next_pts  += frame->nb_samples;

    return frame;
}

//aurgments
unsigned char* add_file_video_frame_buffer_;
unsigned int  add_file_video_frame_size_;
int frida_add_file_video_frame(){
    return add_file_video_frame(add_file_video_frame_buffer_, add_file_video_frame_size_);
}

#endif

#if 0

int AviWriter::addFileVideoFrame(unsigned char* buffer, unsigned int sz)
{
    int ret;
    OutputStream *ost = &video_st_;
    AVCodecContext *c = ost->enc;
    
    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(ost->frame) < 0) {
        LOG_ERRS(" make writable failed");
    }
    
    if (c->pix_fmt != STREAM_SRC_FMT) {
        /* as we only generate a YUV420P picture, we must convert it
         * to the codec pixel format if needed */
        if (!ost->sws_ctx) {
            ost->sws_ctx = sws_getContext(c->width, c->height,
                                          STREAM_SRC_FMT,
                                          c->width, c->height,
                                          c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ost->sws_ctx) {
                LOG_ERRS( "Could not initialize the conversion context");
            }
        }
        AVFrame *pict = ost->tmp_frame;
        {
            AVCodecContext *c = ost->enc;
            int height = c->height;
            int y;
            int linesz= pict->linesize[0];
            memcpy(pict->data[0], buffer, sz);
        }
        sws_scale(ost->sws_ctx, (const uint8_t * const *) ost->tmp_frame->data,
                  ost->tmp_frame->linesize, 0, c->height, ost->frame->data,
                  ost->frame->linesize);
    }
    else{
        LOG_ERRS("  c->pix not right ");
    }
    
    ost->frame->pts = ost->next_pts++;
    ret = write_frame(oc_, video_st_.enc, video_st_.st, ost->frame, video_st_.tmp_pkt);
    return ret;
}

AviWriter::AviWriter(const char* filename, int videoWidth, int videoHeight)
{
    LOG_INFOS(" ");
    memset(&video_st_, 0, sizeof(video_st_));
    LOG_INFOS(" ");
    memset(&audio_st_, 0, sizeof(audio_st_));
    LOG_INFOS(" ");
    filename_ = (char*)filename;
    outVideoWidth_ = videoWidth;
    outVideoHeight_ = videoHeight;
    /* allocate the output media context */
    avformat_alloc_output_context2(&oc_, NULL, "apng", filename_);
    if (!oc_) {
        LOG_INFOS("Could not deduce output format from file extension: using MPEG.");
        avformat_alloc_output_context2(&oc_, NULL, "mpeg", filename_);
    }
    if (!oc_){
        LOG_ERRS(" alloc oc failed ");
    }
    fmt_ = oc_->oformat;
    //fmt->video_codec=AV_CODEC_ID_H264;

    /* Add the audio and video streams using the default format codecs
     * and initialize the codecs. */
    if (fmt_->video_codec != AV_CODEC_ID_NONE) {
        add_stream(&video_st_, oc_, &video_codec_, fmt_->video_codec);
        {
            OutputStream *ost = &video_st_;
            AVCodecContext *c = video_st_.enc;
            c->codec_id = fmt_->video_codec;

            c->bit_rate = 400000;
            /* Resolution must be a multiple of two. */
            c->width    = outVideoWidth_;
            c->height   = outVideoHeight_;
            /* timebase: This is the fundamental unit of time (in seconds) in terms
             * of which frame timestamps are represented. For fixed-fps content,
             * timebase should be 1/framerate and timestamp increments should be
             * identical to 1. */
            ost->st->time_base = (AVRational){ 1, STREAM_FRAME_RATE };
            c->time_base       = ost->st->time_base;

            c->gop_size      = 12; /* emit one intra frame every twelve frames at most */
            c->pix_fmt       = STREAM_PIX_FMT;
            if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
                /* just for testing, we also add B-frames */
                c->max_b_frames = 2;
            }
            if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
                /* Needed to avoid using macroblocks in which some coeffs overflow.
                 * This does not happen with normal video, it just happens here as
                 * the motion of the chroma plane does not match the luma plane. */
                c->mb_decision = 2;
            }
        }
        have_video_   = true;
        encode_video_ = true;
    }
    if (fmt_->audio_codec != AV_CODEC_ID_NONE) {
        add_stream(&audio_st_, oc_, &audio_codec_, fmt_->audio_codec);
        {
            const AVCodec **codec=&audio_codec_;
            OutputStream *ost = &audio_st_;
            AVCodecContext *c = audio_st_.enc;
            c->sample_fmt  = (*codec)->sample_fmts ?  (*codec)->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
            c->bit_rate    = 64000;
            c->sample_rate = 44100;
            if ((*codec)->supported_samplerates) {
                c->sample_rate = (*codec)->supported_samplerates[0];
                for (int i = 0; (*codec)->supported_samplerates[i]; i++) {
                    if ((*codec)->supported_samplerates[i] == 44100)
                        c->sample_rate = 44100;
                }
            }
            c->channels        = av_get_channel_layout_nb_channels(c->channel_layout);
            c->channel_layout = AV_CH_LAYOUT_STEREO;
            if ((*codec)->channel_layouts) {
                c->channel_layout = (*codec)->channel_layouts[0];
                for (int i = 0; (*codec)->channel_layouts[i]; i++) {
                    if ((*codec)->channel_layouts[i] == AV_CH_LAYOUT_STEREO)
                        c->channel_layout = AV_CH_LAYOUT_STEREO;
                }
            }
            c->channels        = av_get_channel_layout_nb_channels(c->channel_layout);
            ost->st->time_base = (AVRational){ 1, c->sample_rate };
        }
        have_audio_   = false;
        encode_audio_ = false;
    }

    /* Now that all the parameters are set, we can open the audio and
     * video codecs and allocate the necessary encode buffers. */
    if (have_video_)
        open_video(oc_, video_codec_, &video_st_, opt_);

    if (have_audio_)
        open_audio(oc_, audio_codec_, &audio_st_, opt_);

    av_dump_format(oc_, 0, filename_, 1);

    int ret;
    /* open the output file, if needed */
    if (!(fmt_->flags & AVFMT_NOFILE)) {
        ret = avio_open(&oc_->pb, filename_, AVIO_FLAG_WRITE);
        if (ret < 0) {
            LOG_ERRS("Could not open '%s': %s", filename_, av_err2str(ret));
        }
    }

    /* Write the stream header, if any. */
    ret = avformat_write_header(oc_, &opt_);
    if (ret < 0) {
        LOG_ERRS("Error occurred when opening output file: %s", av_err2str(ret));
    }
    return;
}

AviWriter::~AviWriter(){
    /* Write the trailer, if any. The trailer must be written before you
     * close the CodecContexts open when you wrote the header; otherwise
     * av_write_trailer() may try to use memory that was freed on
     * av_codec_close(). */
    av_write_trailer(oc_);

    /* Close each codec. */
    if (have_video_) close_stream(oc_, &video_st_);
    if (have_audio_) close_stream(oc_, &audio_st_);

    if (!(fmt_->flags & AVFMT_NOFILE))
        /* Close the output file. */
        avio_closep(&oc_->pb);

    /* free the stream */
    avformat_free_context(oc_);
}

#else
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

int frame_writeavi(unsigned char* buffer, unsigned int sz)
{
    int ret;
    OutputStream *ost = &video_st_;
    AVCodecContext *c = ost->enc;
    
    /* when we pass a frame to the encoder, it may keep a reference to it
     * internally; make sure we do not overwrite it here */
    if (av_frame_make_writable(ost->frame) < 0) {
        LOG_ERRS(" make writable failed");
    }
    
    if (c->pix_fmt != STREAM_SRC_FMT) {
        /* as we only generate a YUV420P picture, we must convert it
         * to the codec pixel format if needed */
        if (!ost->sws_ctx) {
            ost->sws_ctx = sws_getContext(c->width, c->height,
                                          STREAM_SRC_FMT,
                                          c->width, c->height,
                                          c->pix_fmt,
                                          SCALE_FLAGS, NULL, NULL, NULL);
            if (!ost->sws_ctx) {
                LOG_ERRS( "Could not initialize the conversion context");
            }
        }
        AVFrame *pict = ost->tmp_frame;
        {
            AVCodecContext *c = ost->enc;
            int height = c->height;
            int y;
            int linesz= pict->linesize[0];
            memcpy(pict->data[0], buffer, sz);
        }
        sws_scale(ost->sws_ctx, (const uint8_t * const *) ost->tmp_frame->data,
                  ost->tmp_frame->linesize, 0, c->height, ost->frame->data,
                  ost->frame->linesize);
    }
    else{
        LOG_ERRS("  c->pix not right ");
    }
    
    ost->frame->pts = ost->next_pts++;
    ret = write_frame(oc_, video_st_.enc, video_st_.st, ost->frame, video_st_.tmp_pkt);
    return ret;
}

int init_writeavi(const char* filename, int videoWidth, int videoHeight)
{
    LOG_INFOS(" ");
    memset(&video_st_, 0, sizeof(video_st_));
    LOG_INFOS(" ");
    memset(&audio_st_, 0, sizeof(audio_st_));
    LOG_INFOS(" ");
    filename_ = (char*)filename;
    outVideoWidth_ = videoWidth;
    outVideoHeight_ = videoHeight;
    /* allocate the output media context */
    avformat_alloc_output_context2(&oc_, NULL, "apng", filename_);
    if (!oc_) {
        LOG_INFOS("Could not deduce output format from file extension: using MPEG.");
        avformat_alloc_output_context2(&oc_, NULL, "mpeg", filename_);
    }
    if (!oc_){
        LOG_ERRS(" alloc oc failed ");
    }
    fmt_ = oc_->oformat;
    //fmt->video_codec=AV_CODEC_ID_H264;

    /* Add the audio and video streams using the default format codecs
     * and initialize the codecs. */
    if (fmt_->video_codec != AV_CODEC_ID_NONE) {
        add_stream(&video_st_, oc_, &video_codec_, fmt_->video_codec);
        {
            OutputStream *ost = &video_st_;
            AVCodecContext *c = video_st_.enc;
            c->codec_id = fmt_->video_codec;

            c->bit_rate = 400000;
            /* Resolution must be a multiple of two. */
            c->width    = outVideoWidth_;
            c->height   = outVideoHeight_;
            /* timebase: This is the fundamental unit of time (in seconds) in terms
             * of which frame timestamps are represented. For fixed-fps content,
             * timebase should be 1/framerate and timestamp increments should be
             * identical to 1. */
            ost->st->time_base = (AVRational){ 1, STREAM_FRAME_RATE };
            c->time_base       = ost->st->time_base;

            c->gop_size      = 12; /* emit one intra frame every twelve frames at most */
            c->pix_fmt       = STREAM_PIX_FMT;
            if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
                /* just for testing, we also add B-frames */
                c->max_b_frames = 2;
            }
            if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
                /* Needed to avoid using macroblocks in which some coeffs overflow.
                 * This does not happen with normal video, it just happens here as
                 * the motion of the chroma plane does not match the luma plane. */
                c->mb_decision = 2;
            }
        }
        have_video_   = true;
        encode_video_ = true;
    }
    if (fmt_->audio_codec != AV_CODEC_ID_NONE) {
        add_stream(&audio_st_, oc_, &audio_codec_, fmt_->audio_codec);
        {
            const AVCodec **codec=&audio_codec_;
            OutputStream *ost = &audio_st_;
            AVCodecContext *c = audio_st_.enc;
            c->sample_fmt  = (*codec)->sample_fmts ?  (*codec)->sample_fmts[0] : AV_SAMPLE_FMT_FLTP;
            c->bit_rate    = 64000;
            c->sample_rate = 44100;
            if ((*codec)->supported_samplerates) {
                c->sample_rate = (*codec)->supported_samplerates[0];
                for (int i = 0; (*codec)->supported_samplerates[i]; i++) {
                    if ((*codec)->supported_samplerates[i] == 44100)
                        c->sample_rate = 44100;
                }
            }
            c->channels        = av_get_channel_layout_nb_channels(c->channel_layout);
            c->channel_layout = AV_CH_LAYOUT_STEREO;
            if ((*codec)->channel_layouts) {
                c->channel_layout = (*codec)->channel_layouts[0];
                for (int i = 0; (*codec)->channel_layouts[i]; i++) {
                    if ((*codec)->channel_layouts[i] == AV_CH_LAYOUT_STEREO)
                        c->channel_layout = AV_CH_LAYOUT_STEREO;
                }
            }
            c->channels        = av_get_channel_layout_nb_channels(c->channel_layout);
            ost->st->time_base = (AVRational){ 1, c->sample_rate };
        }
        have_audio_   = false;
        encode_audio_ = false;
    }

    /* Now that all the parameters are set, we can open the audio and
     * video codecs and allocate the necessary encode buffers. */
    if (have_video_)
        open_video(oc_, video_codec_, &video_st_, opt_);

    if (have_audio_)
        open_audio(oc_, audio_codec_, &audio_st_, opt_);

    av_dump_format(oc_, 0, filename_, 1);

    int ret;
    /* open the output file, if needed */
    if (!(fmt_->flags & AVFMT_NOFILE)) {
        ret = avio_open(&oc_->pb, filename_, AVIO_FLAG_WRITE);
        if (ret < 0) {
            LOG_ERRS("Could not open '%s': %s", filename_, av_err2str(ret));
        }
    }

    /* Write the stream header, if any. */
    ret = avformat_write_header(oc_, &opt_);
    if (ret < 0) {
        LOG_ERRS("Error occurred when opening output file: %s", av_err2str(ret));
    }
    return 0;
}

void deinit_writeavi()
{
    /* Write the trailer, if any. The trailer must be written before you
     * close the CodecContexts open when you wrote the header; otherwise
     * av_write_trailer() may try to use memory that was freed on
     * av_codec_close(). */
    av_write_trailer(oc_);

    /* Close each codec. */
    if (have_video_) close_stream(oc_, &video_st_);
    if (have_audio_) close_stream(oc_, &audio_st_);

    if (!(fmt_->flags & AVFMT_NOFILE))
        /* Close the output file. */
        avio_closep(&oc_->pb);

    /* free the stream */
    avformat_free_context(oc_);
}

#endif
