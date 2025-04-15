/*
 * Copyright (C) 2006-2021  Music Technology Group - Universitat Pompeu Fabra
 *
 * This file is part of Essentia
 *
 * Essentia is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Affero General Public License as published by the Free
 * Software Foundation (FSF), either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the Affero GNU General Public License
 * version 3 along with this program.  If not, see http://www.gnu.org/licenses/
 */

#include "audiocontext.h"
#include <iostream> // for warning cout

using namespace std;
using namespace essentia;

AudioContext::AudioContext()
  : _isOpen(false), _avStream(nullptr), _muxCtx(nullptr), _codecCtx(nullptr),
    _inputBufSize(0), _buffer(nullptr), _convertCtxAv(nullptr) {
  av_log_set_level(AV_LOG_VERBOSE);

  if (sizeof(float) != av_get_bytes_per_sample(AV_SAMPLE_FMT_FLT)) {
    throw EssentiaException("Unsupported float size");
  }
}

int AudioContext::create(const std::string& filename,
                         const std::string& format,
                         int nChannels, int sampleRate, int bitrate) {
  if (_muxCtx != nullptr) close();

  _filename = filename;

  const AVOutputFormat* av_output_format = av_guess_format(format.c_str(), nullptr, nullptr);
  if (!av_output_format) {
    throw EssentiaException("Could not find a suitable output format for \"", filename, "\"");
  }

  if (format != av_output_format->name) {
    E_WARNING("Essentia is using a different format than the one supplied. Format used is " << av_output_format->name);
  }

  if (avformat_alloc_output_context2(&_muxCtx, nullptr, format.c_str(), filename.c_str()) < 0) {
    throw EssentiaException("Could not allocate the format context");
  }

  _avStream = avformat_new_stream(_muxCtx, nullptr);
  if (!_avStream) throw EssentiaException("Could not allocate stream");

  const AVCodec* audioCodec = avcodec_find_encoder(av_output_format->audio_codec);
  if (!audioCodec) throw EssentiaException("Codec for ", format, " files not found or not supported");

  _codecCtx = avcodec_alloc_context3(audioCodec);
  if (!_codecCtx) throw EssentiaException("Could not allocate codec context");

  _codecCtx->codec_id       = av_output_format->audio_codec;
  _codecCtx->codec_type     = AVMEDIA_TYPE_AUDIO;
  _codecCtx->bit_rate       = bitrate;
  _codecCtx->sample_rate    = sampleRate;
  _codecCtx->channels       = nChannels;
  _codecCtx->channel_layout = av_get_default_channel_layout(nChannels);
  _codecCtx->sample_fmt     = audioCodec->sample_fmts ? audioCodec->sample_fmts[0] : AV_SAMPLE_FMT_FLT;

  for (const enum AVSampleFormat* p = audioCodec->sample_fmts; p && *p != AV_SAMPLE_FMT_NONE; ++p) {
    if (*p == _codecCtx->sample_fmt) break;
  }

  if (avcodec_open2(_codecCtx, audioCodec, nullptr) < 0) {
    throw EssentiaException("Could not open codec");
  }

  if (_codecCtx->frame_size <= 1) {
    throw EssentiaException("Do not know how to encode given format: ", format);
  }

  _inputBufSize = av_samples_get_buffer_size(nullptr,
                                             _codecCtx->channels,
                                             _codecCtx->frame_size,
                                             AV_SAMPLE_FMT_FLT, 0);
  _buffer = (float*)av_malloc(_inputBufSize);

  E_DEBUG(EAlgorithm, "AudioContext: using sample format conversion from libswresample");
  _convertCtxAv = swr_alloc_set_opts(nullptr,
                                     _codecCtx->channel_layout,
                                     _codecCtx->sample_fmt,
                                     _codecCtx->sample_rate,
                                     _codecCtx->channel_layout,
                                     AV_SAMPLE_FMT_FLT,
                                     _codecCtx->sample_rate,
                                     0, nullptr);

  if (!_convertCtxAv || swr_init(_convertCtxAv) < 0) {
    throw EssentiaException("AudioLoader: Could not initialize swresample context");
  }

  return _codecCtx->frame_size;
}

void AudioContext::open() {
  if (_isOpen) return;

  if (!_muxCtx) throw EssentiaException("Trying to open an audio file that has not been created yet or has been closed");

  if (!(_muxCtx->oformat->flags & AVFMT_NOFILE)) {
    if (avio_open(&_muxCtx->pb, _filename.c_str(), AVIO_FLAG_WRITE) < 0) {
      throw EssentiaException("Could not open \"", _filename, "\"");
    }
  }

  if (avformat_write_header(_muxCtx, nullptr) < 0) {
    throw EssentiaException("Error while writing header");
  }

  _isOpen = true;
}

void AudioContext::close() {
  if (!_muxCtx) return;

  if (_isOpen) {
    writeEOF();
    av_write_trailer(_muxCtx);
    if (!(_muxCtx->oformat->flags & AVFMT_NOFILE)) {
      avio_closep(&_muxCtx->pb);
    }
  }

  avcodec_free_context(&_codecCtx);
  avformat_free_context(_muxCtx);

  _muxCtx = nullptr;
  _avStream = nullptr;
  _codecCtx = nullptr;

  if (_buffer) av_freep(&_buffer);

  if (_convertCtxAv) {
    swr_free(&_convertCtxAv);
  }

  _isOpen = false;
}

void AudioContext::write(const vector<StereoSample>& stereoData) {
  if (_codecCtx->channels != 2) {
    throw EssentiaException("Trying to write stereo audio data to an audio file with ", _codecCtx->channels, " channels");
  }

  int dsize = (int)stereoData.size();
  if (dsize > _codecCtx->frame_size) {
    throw EssentiaException("Audio frame size is not sufficient to store samples");
  }

  for (int i = 0; i < dsize; ++i) {
    _buffer[2*i] = (float) stereoData[i].left();
    _buffer[2*i+1] = (float) stereoData[i].right();
  }

  encodePacket(dsize);
}

void AudioContext::write(const vector<AudioSample>& monoData) {
  if (_codecCtx->channels != 1) {
    throw EssentiaException("Trying to write mono audio data to an audio file with ", _codecCtx->channels, " channels");
  }

  int dsize = (int)monoData.size();
  if (dsize > _codecCtx->frame_size) {
    throw EssentiaException("Audio frame size is not sufficient to store samples");
  }

  for (int i = 0; i < dsize; ++i) _buffer[i] = (float)monoData[i];

  encodePacket(dsize);
}

void AudioContext::encodePacket(int size) {
  int inputPlaneSize = av_samples_get_buffer_size(nullptr,
                                                  _codecCtx->channels,
                                                  size,
                                                  AV_SAMPLE_FMT_FLT, 0);
  int outputPlaneSize;
  uint8_t* bufferFmt;

  if (av_samples_alloc(&bufferFmt, &outputPlaneSize,
                       _codecCtx->channels, size,
                       _codecCtx->sample_fmt, 0) < 0) {
    throw EssentiaException("Could not allocate output buffer for sample format conversion");
  }

  if (swr_convert(_convertCtxAv,
                  &bufferFmt,
                  size,
                  (const uint8_t**)&_buffer,
                  size) < size) {
    throw EssentiaException("AudioLoader: Incomplete format conversion");
  }

  AVFrame* frame = av_frame_alloc();
  if (!frame) throw EssentiaException("Error allocating audio frame");

  frame->nb_samples     = size;
  frame->format         = _codecCtx->sample_fmt;
  frame->channel_layout = _codecCtx->channel_layout;
  frame->channels       = _codecCtx->channels;

  avcodec_fill_audio_frame(frame, _codecCtx->channels, _codecCtx->sample_fmt,
                           bufferFmt, outputPlaneSize * _codecCtx->channels, 0);

  AVPacket* pkt = av_packet_alloc();
  int ret = avcodec_send_frame(_codecCtx, frame);
  if (ret < 0) throw EssentiaException("Error sending frame to codec");

  while (ret >= 0) {
    ret = avcodec_receive_packet(_codecCtx, pkt);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
    if (ret < 0) throw EssentiaException("Error receiving packet from codec");

    av_interleaved_write_frame(_muxCtx, pkt);
    av_packet_unref(pkt);
  }

  av_frame_free(&frame);
  av_packet_free(&pkt);
  av_freep(&bufferFmt);
}

void AudioContext::writeEOF() {
  avcodec_send_frame(_codecCtx, nullptr);
  AVPacket* pkt = av_packet_alloc();
  while (true) {
    int ret = avcodec_receive_packet(_codecCtx, pkt);
    if (ret == AVERROR_EOF || ret == AVERROR(EAGAIN)) break;
    if (ret < 0) throw EssentiaException("Error flushing encoder");

    av_interleaved_write_frame(_muxCtx, pkt);
    av_packet_unref(pkt);
  }
  av_packet_free(&pkt);
}
