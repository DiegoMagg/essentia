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

#include "audioloader.h"
#include "algorithmfactory.h"
#include <iomanip>  //  setw()

using namespace std;

namespace essentia {
namespace streaming {

const char* AudioLoader::name = essentia::standard::AudioLoader::name;
const char* AudioLoader::category = essentia::standard::AudioLoader::category;
const char* AudioLoader::description = essentia::standard::AudioLoader::description;


AudioLoader::~AudioLoader() {
    closeAudioFile();

    av_freep(&_buffer);
    av_freep(&_md5Encoded);
    av_frame_free(&_decodedFrame);
}

void AudioLoader::configure() {
    av_log_set_level(AV_LOG_QUIET);
    _computeMD5 = parameter("computeMD5").toBool();
    _selectedStream = parameter("audioStream").toInt();
    reset();
}

void AudioLoader::openAudioFile(const string& filename) {
    E_DEBUG(EAlgorithm, "AudioLoader: opening file: " << filename);

    int errnum;
    if ((errnum = avformat_open_input(&_demuxCtx, filename.c_str(), NULL, NULL)) != 0) {
        char errorstr[128];
        string error = "Unknown error";
        if (av_strerror(errnum, errorstr, 128) == 0) error = errorstr;
        throw EssentiaException("AudioLoader: Could not open file \"", filename, "\", error = ", error);
    }

    if ((errnum = avformat_find_stream_info(_demuxCtx, NULL)) < 0) {
        char errorstr[128];
        string error = "Unknown error";
        if (av_strerror(errnum, errorstr, 128) == 0) error = errorstr;
        avformat_close_input(&_demuxCtx);
        _demuxCtx = nullptr;
        throw EssentiaException("AudioLoader: Could not find stream information, error = ", error);
    }

    _streams.clear();
    for (int i = 0; i < (int)_demuxCtx->nb_streams; ++i) {
        if (_demuxCtx->streams[i]->codecpar->codec_type == AVMEDIA_TYPE_AUDIO) {
            _streams.push_back(i);
        }
    }
    int nAudioStreams = _streams.size();
    if (nAudioStreams == 0) {
        avformat_close_input(&_demuxCtx);
        _demuxCtx = nullptr;
        throw EssentiaException("AudioLoader ERROR: found 0 streams in the file, expecting one or more audio streams");
    }
    if (_selectedStream >= nAudioStreams) {
        avformat_close_input(&_demuxCtx);
        _demuxCtx = nullptr;
        throw EssentiaException("AudioLoader ERROR: 'audioStream' parameter set to ", _selectedStream, ". It should be smaller than the audio streams count, ", nAudioStreams);
    }
    _streamIdx = _streams[_selectedStream];

    // Allocate and setup decoder context
    _audioCtx = avcodec_alloc_context3(nullptr);
    avcodec_parameters_to_context(_audioCtx, _demuxCtx->streams[_streamIdx]->codecpar);

    const AVCodec* _audioCodec = avcodec_find_decoder(_audioCtx->codec_id);
    if (!_audioCodec) {
        throw EssentiaException("AudioLoader: Unsupported codec!");
    }
    if (avcodec_open2(_audioCtx, _audioCodec, NULL) < 0) {
        throw EssentiaException("AudioLoader: Unable to instantiate codec...");
    }

    int64_t layout = av_get_default_channel_layout(_audioCtx->channels);
    _convertCtxAv = swr_alloc_set_opts(nullptr,
                                       layout,
                                       AV_SAMPLE_FMT_FLT,
                                       _audioCtx->sample_rate,
                                       layout,
                                       _audioCtx->sample_fmt,
                                       _audioCtx->sample_rate,
                                       0, nullptr);
    if (!_convertCtxAv || swr_init(_convertCtxAv) < 0) {
        throw EssentiaException("AudioLoader: Could not initialize swresample context");
    }

    av_init_packet(&_packet);
    _decodedFrame = av_frame_alloc();
    if (!_decodedFrame) {
        throw EssentiaException("AudioLoader: Could not allocate audio frame");
    }
    av_md5_init(_md5Encoded);
}

void AudioLoader::closeAudioFile() {
    if (!_demuxCtx) return;
    if (_convertCtxAv) {
        swr_close(_convertCtxAv);
        swr_free(&_convertCtxAv);
    }
    if (_audioCtx) avcodec_free_context(&_audioCtx);
    avformat_close_input(&_demuxCtx);
    av_packet_unref(&_packet);
    _demuxCtx = nullptr;
    _audioCtx = nullptr;
    _streams.clear();
}

AlgorithmStatus AudioLoader::process() {
    if (!parameter("filename").isConfigured()) {
        throw EssentiaException("AudioLoader: process() called without filename configured.");
    }

    // Função auxiliar para converter bytes em hexadecimal
    auto bytesToHex = [](uint8_t* input, int size) {
        std::ostringstream result;
        for (int i = 0; i < size; ++i) {
            result << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(input[i]);
        }
        return result.str();
    };

    do {
        int result = av_read_frame(_demuxCtx, &_packet);
        if (result != 0) {
            if (result != AVERROR_EOF) {
                char errstring[1024];
                av_strerror(result, errstring, sizeof(errstring));
                E_WARNING(std::string("AudioLoader: Error reading frame: ") + errstring);
            }
            shouldStop(true);
            flushPacket();
            closeAudioFile();
            if (_computeMD5) {
                av_md5_final(_md5Encoded, _checksum);
                _md5.push(bytesToHex(_checksum, 16));
            } else {
                _md5.push(std::string());
            }
            return FINISHED;
        }
    } while (_packet.stream_index != _streamIdx);

    if (_computeMD5) av_md5_update(_md5Encoded, _packet.data, _packet.size);
    while (_packet.size > 0) {
        if (!decodePacket()) break;
        copyFFmpegOutput();
    }
    av_packet_unref(&_packet);
    return OK;
}

int AudioLoader::decode_audio_frame(AVCodecContext* audioCtx,
                                    float* output,
                                    int* outputSize,
                                    AVPacket* packet) {
    av_frame_unref(_decodedFrame);
    int gotFrame = 0;
    int ret = avcodec_send_packet(audioCtx, packet);
    if (ret < 0) return ret;
    ret = avcodec_receive_frame(audioCtx, _decodedFrame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
        *outputSize = 0;
        return 0;
    }
    if (ret < 0) return ret;
    gotFrame = 1;
    int inputSamples = _decodedFrame->nb_samples;
    int inputPlaneSize = av_samples_get_buffer_size(NULL, _nChannels, inputSamples, audioCtx->sample_fmt, 1);
    int outputPlaneSize = av_samples_get_buffer_size(NULL, _nChannels, inputSamples, AV_SAMPLE_FMT_FLT, 1);
    if (*outputSize < outputPlaneSize) {
        throw EssentiaException("AudioLoader: Insufficient buffer size for format conversion");
    }
    if (audioCtx->sample_fmt == AV_SAMPLE_FMT_FLT) {
        memcpy(output, _decodedFrame->data[0], inputPlaneSize);
    } else {
        int written = swr_convert(_convertCtxAv,
                                  (uint8_t**)&output,
                                  inputSamples,
                                  (const uint8_t**)_decodedFrame->data,
                                  inputSamples);
        if (written < inputSamples) {
            throw EssentiaException("AudioLoader: Incomplete format conversion");
        }
    }
    *outputSize = outputPlaneSize;
    return inputPlaneSize;
}

void AudioLoader::flushPacket() {
    AVPacket empty;
    av_init_packet(&empty);
    do {
        _dataSize = FFMPEG_BUFFER_SIZE;
        empty.data = NULL;
        empty.size = 0;
        int len = decode_audio_frame(_audioCtx, _buffer, &_dataSize, &empty);
        if (len < 0) {
            char errstring[1204];
            av_strerror(len, errstring, sizeof(errstring));
            E_WARNING(string("AudioLoader: decoding error while flushing packet: ") + errstring);
        }
        copyFFmpegOutput();
    } while (_dataSize > 0);
}

void AudioLoader::copyFFmpegOutput() {
    int nsamples = _dataSize / (av_get_bytes_per_sample(AV_SAMPLE_FMT_FLT) * _nChannels);
    if (nsamples == 0) return;
    bool ok = _audio.acquire(nsamples);
    if (!ok) throw EssentiaException("AudioLoader: could not acquire output for audio");
    vector<StereoSample>& audio = *((vector<StereoSample>*)_audio.getTokens());
    if (_nChannels == 1) {
        for (int i = 0; i < nsamples; ++i) audio[i].left() = _buffer[i];
    } else {
        for (int i = 0; i < nsamples; ++i) {
            audio[i].left() = _buffer[2*i];
            audio[i].right() = _buffer[2*i+1];
        }
    }
    _audio.release(nsamples);
}

void AudioLoader::reset() {
    Algorithm::reset();
    if (!parameter("filename").isConfigured()) return;
    string filename = parameter("filename").toString();
    closeAudioFile();
    openAudioFile(filename);
    pushChannelsSampleRateInfo(_audioCtx->channels, _audioCtx->sample_rate);
    pushCodecInfo(_audioCodec->name, _audioCtx->bit_rate);
}

} // namespace streaming
} // namespace essentia
