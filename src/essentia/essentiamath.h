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

#ifndef ESSENTIA_MATH_H
#define ESSENTIA_MATH_H

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <math.h>
#include <cmath>
#include <vector>
#include <numeric>
#include <limits>
#include <functional>
#include <utility> // for pair
#include <sstream>
#include <algorithm> // for std::sort
#include <deque>
#include "types.h"
#include "utils/tnt/tnt.h"
#include "utils/tnt/tnt2essentiautils.h"

#define M_2PI (2 * M_PI)
#define ALL_NOTES "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#"

namespace essentia {

template <typename T> bool isPowerTwo(T n) {
  return (n & (n-1)) == 0;
}

template <typename T> T log2(T x) {
  return log(x) / M_LN2;
}

template <typename T>
int ilog10(T n) {
  if (n < 0) return ilog10(-n);
  if (n < 10) return 0; // should return -infinite for n == 0
  return 1 + ilog10(n/10);
}

/**
 * Return the next power of two after the given number n.
 * If n is already a power of two, return n.
 */
template <typename T> T nextPowerTwo(T n) {
  n--;
  n |= (n >> 1);
  n |= (n >> 2);
  n |= (n >> 4);
  n |= (n >> 8);
  n |= (n >> 16);
  return ++n;
}

template <> inline long long int nextPowerTwo(long long int n) {
  n--;
  n |= (n >> 1);
  n |= (n >> 2);
  n |= (n >> 4);
  n |= (n >> 8);
  n |= (n >> 16);
  n |= (n >> 32);
  return ++n;
}

/**
 * Returns the L2-norm of an array
 */
template <typename T> T norm(const std::vector<T>& array) {
  if (array.empty()) {
    throw EssentiaException("trying to calculate norm of empty array");
  }

  T sum = (T) 0.0;

  for (uint i=0; i<array.size(); i++) {
    sum += array[i] * array[i];
  }

  return sqrt(sum);
}

/**
 * Returns the sum of squared values of an array
 */
template <typename T> T sumSquare(const std::vector<T> array) {
  T sum = 0.0;
  for (size_t i = 0; i < array.size(); ++i) {
    sum += array[i] * array[i];
  }
  return sum;
}

/**
 * returns the sum of an array, unrolled version.
 */
template <typename T> T sum(const std::vector<T>& array, int start, int end) {
  T sum = 0.0;
  int i = start;

  for (; i<end-8; i+=8) {
    sum += array[i];
    sum += array[i+1];
    sum += array[i+2];
    sum += array[i+3];
    sum += array[i+4];
    sum += array[i+5];
    sum += array[i+6];
    sum += array[i+7];
  }

  // do the rest of the loop
  for (; i<end; i++) {
    sum += array[i];
  }

  return sum;
}

/**
 * returns the mean of an array, unrolled version.
 */
template <typename T> T mean(const std::vector<T>& array, int start, int end) {
  return sum(array, start, end) / (end - start);
}

/**
 * returns the sum of an array.
 */
template <typename T> T sum(const std::vector<T>& array) {
  if (array.empty()) return 0;
  return sum(array, 0, array.size());
}

/**
 * returns the mean of an array.
 */
template <typename T> T mean(const std::vector<T>& array) {
  if (array.empty())
    throw EssentiaException("trying to calculate mean of empty array");
  return mean(array, 0, array.size());
}

/**
 * returns the mean of an array of TNT::Array2D*
 */
template <typename T>
  TNT::Array2D<T> meanMatrix(const std::vector<TNT::Array2D<T>* >& array) {
  if (array.empty())
    throw EssentiaException("trying to calculate mean of empty array");
  //return mean(array, 0, array.size());
  TNT::Array2D<T> mean(array[0]->dim1(), array[0]->dim2());
  matinit(mean);
  for (int i = 0; i < (int)array.size(); i++) {
    mean += *array[i];
  }
  mean /= (Real)array.size();
  return mean;
}

/**
 * returns the mean of an array of TNT::Array2D
 */
template <typename T>
  TNT::Array2D<T> meanMatrix(const std::vector<TNT::Array2D<T> >& array) {
  if (array.empty())
    throw EssentiaException("trying to calculate mean of empty array");
  //return mean(array, 0, array.size());
  TNT::Array2D<T> mean(array[0].dim1(), array[0].dim2());
  matinit(mean);
  for (int i = 0; i < (int)array.size(); i++) {
    mean += array[i];
  }
  mean /= (Real)array.size();
  return mean;
}

// returns the mean of frames
template <typename T>
std::vector<T> meanFrames(const std::vector<std::vector<T> >& frames, int beginIdx=0, int endIdx=-1) {
  if (frames.empty()) {
    throw EssentiaException("trying to calculate mean of empty array of frames");
  }

  if (endIdx == -1) endIdx = (int)frames.size();
  uint vsize = frames[0].size();

  std::vector<T> result(vsize, (T)0.0);
  typename std::vector<std::vector<T> >::const_iterator it = frames.begin() + beginIdx;
  typename std::vector<std::vector<T> >::const_iterator end = frames.begin() + endIdx;
  for (; it!=end; ++it) {
	typename std::vector<T>::const_iterator itFrame = it->begin();
	typename std::vector<T>::const_iterator endFrame = it->end();
	typename std::vector<T>::iterator itResult = result.begin();
    for (; itFrame != endFrame; ++itFrame, ++itResult) {
      *itResult += *itFrame;
    }
  }
  for (uint j=0; j<vsize; j++) result[j] /= (endIdx - beginIdx);

  return result;
}

// returns the median of frames
template <typename T>
std::vector<T> medianFrames(const std::vector<std::vector<T> >& frames, int beginIdx=0, int endIdx=-1) {
  if (frames.empty()) {
    throw EssentiaException("trying to calculate mean of empty array of frames");
  }

  if (endIdx == -1) endIdx = (int)frames.size();

  uint vsize = frames[0].size();
  uint fsize = endIdx - beginIdx;

  std::vector<T> result(vsize, (T)0.0);
  std::vector<T> temp;
  temp.reserve(fsize);

  for (uint i=0; i<vsize; ++i) {
    typename std::vector<std::vector<T> >::const_iterator it = frames.begin() + beginIdx;
    typename std::vector<std::vector<T> >::const_iterator end = frames.begin() + endIdx;

    temp.clear();
    for (; it!=end; ++it) {
      temp.push_back((*it)[i]);
    }
    std::sort(temp.begin(), temp.end());

    // array size is an odd number
    if (fsize % 2 == 0.0) {
      result[i] = (temp[uint(fsize/2 - 1)] + temp[uint(fsize/2)]) / 2;
    }
    // array size is an even number
    else {
      result[i] = temp[uint(fsize/2)];
    }
  }
  return result;
}


// returns the variance of frames
template <typename T>
std::vector<T> varianceFrames(const std::vector<std::vector<T> >& frames) {
  if (frames.empty()) {
    throw EssentiaException("trying to calculate variance of empty array of frames");
  }

  uint nframes = frames.size();
  uint vsize = frames[0].size();

  std::vector<T> m = meanFrames(frames);

  std::vector<T> result(vsize, (T)0.0);
  T diff;
  for (uint i=0; i<nframes; i++) {
    for (uint j=0; j<vsize; j++) {
      diff = frames[i][j] - m[j];
      result[j] += diff*diff;
    }
  }
  for (uint j=0; j<vsize; j++) result[j] /= nframes;

  return result;
}


// returns the sum of frames 
template <typename T>
std::vector<T> sumFrames(const std::vector<std::vector<T> >& frames) {
  if (frames.empty()) {
    throw EssentiaException("sumFrames: trying to calculate sum of empty input frames");
  }
  size_t nframes = frames.size();
  size_t vsize = frames[0].size();
  std::vector<T> result(vsize, (T)0);
  for (size_t j=0; j<vsize; j++) {
    for (size_t i=0; i<nframes; i++) {
      result[j] += frames[i][j];
    }
  }
  return result;
}


template <typename T>
std::vector<T> skewnessFrames(const std::vector<std::vector<T> >& frames) {
  if (frames.empty()) {
    throw EssentiaException("trying to calculate skewness of empty array of frames");
  }

  uint nframes = frames.size();
  uint vsize = frames[0].size();

  std::vector<T> m = meanFrames(frames);

  std::vector<T> result(vsize, (T)0.0);
  std::vector<T> m3(vsize, (T)0.0);
  std::vector<T> m2(vsize, (T)0.0);
  T diff;
  for (uint i=0; i<nframes; i++) {
    for (uint j=0; j<vsize; j++) {
      diff = frames[i][j] - m[j];
      m2[j] += diff*diff;
      m3[j] += diff*diff*diff;
    }
  }
  for (uint j=0; j<vsize; j++) {
    m2[j] /= nframes;
    m3[j] /= nframes;
    if (m2[j] == (T)0.) result[j] = (T)0.;
    else result[j] = m3[j] / pow(m2[j], (T)1.5);
  }

  return result;
}

template <typename T>
std::vector<T> kurtosisFrames(const std::vector<std::vector<T> >& frames) {
  if (frames.empty()) {
    throw EssentiaException("trying to calculate kurtosis of empty array of frames");
  }

  uint nframes = frames.size();
  uint vsize = frames[0].size();

  std::vector<T> m = meanFrames(frames);

  std::vector<T> result(vsize, (T)0.0);
  std::vector<T> m2(vsize, (T)0.0);
  std::vector<T> m4(vsize, (T)0.0);
  T diff;
  for (uint i=0; i<nframes; i++) {
    for (uint j=0; j<vsize; j++) {
      diff = frames[i][j] - m[j];
      m2[j] += diff*diff;
      m4[j] += diff*diff*diff*diff;
    }
  }
  for (uint j=0; j<vsize; j++) {
    m2[j] /= nframes;
    m4[j] /= nframes;
    if (m2[j] == (T)0.) result[j] = (T)(-3.);
    else result[j] = m4[j] / (m2[j]*m2[j]) - 3;
  }

  return result;
}


// returns the median of an array
template <typename T> T median(const std::vector<T>& array) {
  if (array.empty())
    throw EssentiaException("trying to calculate median of empty array");

  // median has sense only on sorted array
  std::vector<T> sorted_array = array;
  std::sort(sorted_array.begin(), sorted_array.end());

  uint size = sorted_array.size();

  // array size is an odd number
  if (size % 2 == 0.0) {
    return (sorted_array[uint(size/2 - 1)] + sorted_array[uint(size/2)]) / 2;
  }
  // array size is an even number
  else {
    return sorted_array[uint(size/2)];
  }
}

// returns the absolute value of each element of the array
template <typename T>
void rectify(std::vector<T>& array) {
  for (int i=0; i<(int)array.size(); i++) {
    array[i] = fabs(array[i]);
  }
}

// returns the sum of the squared array = the energy of the array
template <typename T> T energy(const std::vector<T>& array) {
  if (array.empty())
    throw EssentiaException("trying to calculate energy of empty array");

  return inner_product(array.begin(), array.end(), array.begin(), (T)0.0);
}

// returns the instantaneous power of an array
template <typename T> T instantPower(const std::vector<T>& array) {
  return energy(array) / array.size();
}

// silence_cutoff_dB = -60
// silence_cutoff = 10 ^ (silence_cutoff_dB / 10)
// silence cutoff has been set to -90 dB. The rationale behind it is that in
// principle we won't have absolute silence anywhere except for those few seconds
// left between one song and the next one on a CD, all the other frames will
// never have complete digital silence as any equipment will produce some kind
// of noise and even when music is rendered to file it is normally dithered
// first. For this reason we set the silence cutoff as what should be silence
// on a 16bit pcm file, that is (16bit - 1bit)*6.02 which yields -90.3 dB thus
// aproximately -90
#define SILENCE_CUTOFF 1e-10
#define DB_SILENCE_CUTOFF -100
#define LOG_SILENCE_CUTOFF -23.025850929940457

// returns true if the signal average energy is below a cutoff value, here -90dB
template <typename T> bool isSilent(const std::vector<T>& array) {
  return instantPower(array) < SILENCE_CUTOFF;
}

// returns the variance of an array of TNT::Array2D<T> elements
template <typename T>
  TNT::Array2D<T> varianceMatrix(const std::vector<TNT::Array2D<T> >& array, const TNT::Array2D<T> & mean) {
  if (array.empty())
    throw EssentiaException("trying to calculate variance of empty array");

  TNT::Array2D<T> variance(array[0].dim1(), array[0].dim2());
  matinit(variance);

  for (int i=0; i<(int)array.size(); i++) {
    TNT::Array2D<T> temp = array[i] - mean;
    variance += temp * temp;
  }

  return variance / (T)array.size();
}
// returns the variance of an array of TNT::Array2D<T>* elements
template <typename T>
  TNT::Array2D<T> varianceMatrix(const std::vector<TNT::Array2D<T>* >& array, const TNT::Array2D<T> & mean) {
  if (array.empty())
    throw EssentiaException("trying to calculate variance of empty array");

  TNT::Array2D<T> variance(array[0]->dim1(), array[0]->dim2());
  matinit(variance);

  for (int i=0; i<(int)array.size(); i++) {
    TNT::Array2D<T> temp = *array[i] - mean;
    variance += temp * temp;
  }

  return variance / (T)array.size();
}

// returns the variance of an array
template <typename T> T variance(const std::vector<T>& array, const T mean) {
  if (array.empty())
    throw EssentiaException("trying to calculate variance of empty array");

  T variance = (T) 0.0;

  for (uint i=0; i<array.size(); i++) {
    T temp = array[i] - mean;
    variance += temp * temp;
  }

  return variance / array.size();
}

// returns the skewness of an array
template <typename T> T skewness(const std::vector<T>& array, const T mean) {
  if (array.empty())
    throw EssentiaException("trying to calculate skewness of empty array");

  const int n = (int)array.size();
  T m2 = (T)0.0, m3 = (T)0.0;

  for (int i=0; i<n; i++) {
    T temp = array[i] - mean;
    m2 += temp * temp;
    m3 += temp * temp * temp;
  }

  m2 /= n; m3 /= n;

  T result;
  //if (std::isnan(result) || std::isinf(result)) return 0;
  if (m2 == (T)0.) result = (T)0.;
  else result = m3 / pow(m2, (T)1.5);

  return result;
}

// returns the kurtosis of an array
template <typename T> T kurtosis(const std::vector<T>& array, const T mean) {
  if (array.empty())
    throw EssentiaException("trying to calculate kurtosis of empty array");

  const int n = (int)array.size();
  T m2 = (T)0.0, m4 = (T)0.0;

  for (int i=0; i<n; i++) {
    T temp = array[i] - mean;
    m2 += temp * temp;
    m4 += temp * temp * temp * temp;
  }

  m2 /= n; m4 /= n;

  T result;
  //if (std::isnan(result) || std::isinf(result)) return 0;
  if (m2 == (T)0.) result = (T)(-3.);
  else result = m4 / (m2*m2) - 3;

  return result;
}


// returns the standard deviation of an array
template <typename T> T stddev(const std::vector<T>& array, const T mean) {
  if (array.empty())
    throw EssentiaException("trying to calculate stddev of empty array");

  return (T)sqrt(variance(array, mean));
}

// round a value to the nearest integer value
template <typename T> T round(const T value) {
  return (T)std::floor(value + (T)0.5);
}


inline Real lin2db(Real value) {
  return value < SILENCE_CUTOFF ? DB_SILENCE_CUTOFF : (Real)10.0 * log10(value);
}

inline Real lin2db(Real value, Real silenceCutoff, Real dbSilenceCutoff) {
  return value < silenceCutoff ? dbSilenceCutoff : (Real)10.0 * log10(value);
}

inline Real db2lin(Real value) {
  return pow((Real)10.0, value/(Real)10.0);
}

inline Real pow2db(Real power) {
  return lin2db(power);
}

inline Real pow2db(Real power, Real silenceCutoff, Real dbSilenceCutoff) {
  return lin2db(power, silenceCutoff, dbSilenceCutoff);  
}

inline Real db2pow(Real power) {
  return db2lin(power);
}

inline Real amp2db(Real amplitude) {
  return Real(2.0)*lin2db(amplitude);
}

inline Real amp2db(Real amplitude, Real silenceCutoff, Real dbSilenceCutoff) {
  return Real(2.0)*lin2db(amplitude, silenceCutoff, dbSilenceCutoff);  
}

inline Real db2amp(Real amplitude) {
  return db2lin(0.5*amplitude);
}

inline Real linear(Real input) {
  return input;
}

inline Real lin2log(Real value) {
  return value < SILENCE_CUTOFF ? LOG_SILENCE_CUTOFF : log(value);
}

inline Real lin2log(Real input, Real silenceCutoff, Real logSilenceCutoff) {
  return input < silenceCutoff ? logSilenceCutoff : log(input);
}


#ifdef OS_WIN32
// The following function hz2bark needs the function asinh,
// which is not included in ANSI math.h and thus does not
// "compile in windows". I copied the function asinh from boost
// http://www.boost.org/boost/math/special_functions/asinh.hpp
// Joachim, 25. June 2007
template<typename T>
inline T asinh(const T x)
{
    using ::std::abs;
    using ::std::sqrt;
    using ::std::log;
    using ::std::numeric_limits;

    T const one = static_cast<T>(1);
    T const two = static_cast<T>(2);

    static T const taylor_2_bound = sqrt(numeric_limits<T>::epsilon());
    static T const taylor_n_bound = sqrt(taylor_2_bound);
    static T const upper_taylor_2_bound = one/taylor_2_bound;
    static T const upper_taylor_n_bound = one/taylor_n_bound;

    if (x >= +taylor_n_bound) {
       if (x > upper_taylor_n_bound) {
          if (x > upper_taylor_2_bound) {
             // approximation by laurent series in 1/x at 0+ order from -1 to 0
             return( log( x * two) );
          }
          else {
             // approximation by laurent series in 1/x at 0+ order from -1 to 1
             return( log( x*two + (one/(x*two)) ) );
          }
       }
       else {
          return( log( x + sqrt(x*x+one) ) );
       }
    }
    else if (x <= -taylor_n_bound) {
       return(-asinh(-x));
    }
    else {
       // approximation by taylor series in x at 0 up to order 2
       T result = x;

       if (abs(x) >= taylor_2_bound)
       {
          T x3 = x*x*x;

          // approximation by taylor series in x at 0 up to order 4
          result -= x3/static_cast<T>(6);
       }
       return(result);
    }
}
#endif

/**
 * Converts a given frequency into its Bark value.
 * This formula is taken from:
 *  H. Traunmüller (1990) "Analytical expressions for the tonotopic sensory scale" J. Acoust. Soc. Am. 88: 97-100.
 * and has been independently verified to be the one that best matches the band
 * frequencies defined by Zwicker in 1961.
 * @param f the input frequency, in Hz
 */
inline Real hz2bark(Real f) {
  Real b = ((26.81*f)/(1960 + f)) - 0.53;

  if (b < 2) b += 0.15*(2-b);
  if (b > 20.1) b += 0.22*(b - 20.1);

  return b;
}

/**
 * Converts a Bark value into its corresponding frequency.
 * This formula is deduced from:
 *  H. Traunmüller (1990) "Analytical expressions for the tonotopic sensory scale" J. Acoust. Soc. Am. 88: 97-100.
 * and has been independently verified to be the one that best matches the band
 * frequencies defined by Zwicker in 1961.
 * @param z the critical band rate, in Bark
 */
inline Real bark2hz(Real z) {
  // note: these conditions have been deduced by inverting the ones from hz2bark
  if (z < 2) z = (z - 0.3) / 0.85;
  if (z > 20.1) z = (z - 4.422) / 1.22;

  // this part comes from Traunmüller's paper (could have deduced it also by myself... ;-) )
  return 1960.0 * (z + 0.53) / (26.28 - z);
}

inline Real barkCriticalBandwidth(Real z) {
  return 52548.0 / (z*z - 52.56 * z + 690.39);
}


inline Real mel2hz(Real mel) {
  return 700.0 * (exp(mel/1127.01048) - 1.0);
}

inline Real mel102hz(Real mel) {
  return 700.0 * (pow(10.0, mel/2595.0) - 1.0);
}

// Convert Mel to Hz based on Slaney's formula in MATLAB Auditory Toolbox
inline Real mel2hzSlaney(Real mel) {
  const Real minLogHz = 1000.0;
  const Real linSlope = 3 / 200.;
  const Real minLogMel = minLogHz * linSlope;

  if (mel < minLogMel) {
    // Linear part: 0 - 1000 Hz.
    return mel / linSlope;
  }
  else {
    // Log-scale part: >= 1000 Hz.
    const Real logStep = log(6.4) / 27.0;
    return minLogHz * exp((mel - minLogMel) * logStep);
  }
}

inline Real hz2mel(Real hz) {
  return 1127.01048 * log(hz/700.0 + 1.0);
}

inline Real hz2mel10(Real hz) {
  return 2595.0 * log10(hz/700.0 + 1.0);
}

// Convert Hz to Mel based on Slaney's formula in MATLAB Auditory Toolbox
inline Real hz2melSlaney(Real hz) {
  const Real minLogHz = 1000.0;
  const Real linSlope = 3 / 200.;

  if (hz < minLogHz) {
    // Linear part: 0 - 1000 Hz.
    return hz * linSlope;
  }
  else {
    // Log-scale part: >= 1000 Hz.
    const Real minLogMel = minLogHz * linSlope;
    const Real logStep = log(6.4) / 27.0;
    return minLogMel + log(hz/minLogHz) / logStep;
  }
}

inline Real hz2hz(Real hz){
  return hz;
}

inline Real cents2hz(Real cents, Real referenceFrequency) {
  return referenceFrequency * powf(2.0, cents / 1200.0);
}

inline Real hz2cents(Real hz, Real referenceFrequency) {
  return 1200 * log2(hz / referenceFrequency);
}

inline int hz2midi(Real hz, Real tuningFrequency) {
  return 69 + (int) round(log2(hz / tuningFrequency) * 12);
}

inline Real midi2hz(int midiNoteNumber, Real tuningFrequency) {
  return tuningFrequency * powf(2, (midiNoteNumber - 69) / 12.0);
}

inline std::string note2root(std::string note) {
    return note.substr(0, note.size()-1);
}

inline int note2octave(std::string note) {
    char octaveChar = note.back();
    return octaveChar - '0';
}

inline std::string midi2note(int midiNoteNumber) {
  std::string NOTES[] = {ALL_NOTES};
  int nNotes = *(&NOTES + 1) - NOTES;
  int CIdx = 3;
  int diffCIdx = nNotes - CIdx;
  int noteIdx = midiNoteNumber - 69;
  int idx = abs(noteIdx) % nNotes;
  int octave = (CIdx + 1) + floor(float(noteIdx + diffCIdx) / nNotes);
  if (noteIdx < 0) {
    idx = abs(idx - nNotes) % nNotes;
  }
  std::string closest_note = NOTES[idx] + std::to_string(octave);
  return closest_note;
}

inline int note2midi(std::string note) {
  //const std::vector<std::string> ALL_NOTES { "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#" };
  std::string NOTES[] = {ALL_NOTES};
  int octave = note2octave(note);
  std::string root = note2root(note);
  int nNotes = *(&NOTES + 1) - NOTES;
  //int nNotes = NOTES.size();
  int CIdx = 3;

  int noteIdx = floor((octave - (CIdx + 1)) * nNotes);
  int idx = 0;
  for (int i = 0; i < nNotes; i++) {
    if (NOTES[i] == root) {
      idx = i;
      if (idx >= CIdx) {
        idx = idx - nNotes;
      }
      break;
    }
  }
  int midiNote = noteIdx + 69 + idx;
  return midiNote;
}

inline std::string hz2note(Real hz, Real tuningFrequency) {
    int midiNoteNumber = hz2midi(hz, tuningFrequency);
    return midi2note(midiNoteNumber);
}

inline int note2hz(std::string note, Real tuningFrequency) {
    int midiNoteNumber = note2midi(note);
    return midi2hz(midiNoteNumber, tuningFrequency);
}

inline int db2velocity (Real decibels, Real hearingThreshold) {
  int velocity = 0;
  if (decibels > hearingThreshold) {
    velocity = (int)((hearingThreshold - decibels) * 127 / hearingThreshold);  // decibels should be negative
  }
  return velocity;
}

inline Real velocity2db(int velocity, Real hearingThreshold) {
  return -(hearingThreshold * velocity / 127 -hearingThreshold);
}

inline int argmin(const std::vector<Real>& input) {
  if (input.empty())
    throw EssentiaException("trying to get argmin of empty array");
  return std::min_element(input.begin(), input.end()) - input.begin();
}

inline int argmax(const std::vector<Real>& input) {
  if (input.empty())
    throw EssentiaException("trying to get argmax of empty array");
  return std::max_element(input.begin(), input.end()) - input.begin();
}

// normalize a vector so its largest value gets mapped to 1
// if zero, the vector isn't touched
template <typename T> void normalize(std::vector<T>& array) {
  if (array.empty()) return;

  T maxElement = *std::max_element(array.begin(), array.end());

  if (maxElement != (T) 0.0) {
    for (uint i=0; i<array.size(); i++) {
      array[i] /= maxElement;
    }
  }
}

// normalize to the max(abs(array))
template <typename T> void normalizeAbs(std::vector<T>& array) {
  if (array.empty()) return;
  std::vector<T> absArray = array;
  rectify(absArray);
  T maxElement = *std::max_element(absArray.begin(), absArray.end());

  if (maxElement != (T) 0.0) {
    for (uint i=0; i<array.size(); i++) {
      array[i] /= maxElement;
    }
  }
}

// normalize to the max(abs(array)) with a headroom value
template <typename T> void normalizeAbs(std::vector<T>& array, T headroom) {
  if (array.empty()) return;
  std::vector<T> absArray = array;
  rectify(absArray);
  T maxElement = *std::max_element(absArray.begin(), absArray.end());

  if (maxElement != (T) 0.0) {
    for (uint i=0; i<array.size(); i++) {
      array[i] /= (maxElement + headroom);
    }
  }
}

// normalize a vector so it's sum is equal to 1. the vector is not touched if
// it contains negative elements or the sum is zero
template <typename T> void normalizeSum(std::vector<T>& array) {
  if (array.empty()) return;

  //T sumElements = std::accumulate(array.begin(), array.end(), (T) 0.0);
  T sumElements = (T) 0.;
  for (size_t i=0; i<array.size(); ++i) {
    if (array[i] < 0) return;
    sumElements += array[i];
  }

  if (sumElements != (T) 0.0) {
    for (size_t i=0; i<array.size(); ++i) {
      array[i] /= sumElements;
    }
  }
}

// returns the difference and approximate derivative vector of a vector
// derivative(x), for a vector x, is [x(1)-x(0)  x(2)-x(1) ... x(n-1)-x(n-2)]
template <typename T>
std::vector<T> derivative(const std::vector<T>& array) {
  if (array.size() < 2) {
     throw EssentiaException("trying to calculate approximate derivative of empty or single-element array");
  }

  std::vector<T> result(array.size()-1, (T)0.0);
  for (int i=0; i<(int)result.size(); i++) {
    result[i] = array[i+1] - array[i];
  }
  return result;
}

template<typename T, typename U, typename Comparator=std::greater<T> >
class PairCompare {
  Comparator _cmp;
  public:
    bool operator () (const std::pair<T,U>& p1, const std::pair<T,U>& p2) const {
      if (_cmp(p1.first, p2.first)) return true;
      if (_cmp(p2.first, p1.first)) return false;
      return _cmp(p1.second, p2.second);
    }
};

// sorts two vectors by the cmp function. If the first elements of the pairs
// are equal, then it sorts by using cmp on the second value of the pair
template <typename T, typename U, typename Comparator>
void sortpair(std::vector<T>& v1, std::vector<U>& v2) {
  if (v1.size() != v2.size()) {
    throw EssentiaException("Cannot sort vectors of different size");
  }
  int size = v1.size();
  std::vector<std::pair<T, U> > tmp(size);
  for (int i=0; i<size; i++)
    tmp[i] = std::make_pair(v1[i], v2[i]);
  std::sort(tmp.begin(), tmp.end(), PairCompare<T, U, Comparator>());
  for (int i=0; i<size; i++) {
    v1[i] = tmp[i].first;
    v2[i] = tmp[i].second;
  }
}


// returns whether a number is a denormal number or not
inline bool isDenormal(const float& x) {
  return std::fpclassify(x) == FP_SUBNORMAL;

  // old version: works only for i386 and float
  //const int& xbits = reinterpret_cast<const int&>(x);
  //const int absMantissa = xbits & 0x007FFFFF;
  //const int biasedExponent = xbits & 0x7F800000;
  //return (biasedExponent == 0 && absMantissa != 0);
}

// should always return a positive value, even when a/b is negative
template <typename T> T fmod(T a, T b) {
  T q = floor(a/b);
  return a - q*b;
}

// returns the principal phase argument between [-PI,PI]
template <typename T> T princarg(T y) {
  T x = essentia::fmod(y + M_PI, M_2PI);
  //if (x < 0) x += M_2PI; // should be useless with our implementation of fmod
  return x - M_PI;
}

/**
 * Given a set of values, computes the associated histogram. This method is
 * designed to work the same way as in Matlab/Octave. It is based on the
 * algorithms used in Octave rather than Matlab. The result structures
 * (n_arr, x_arr) have to be allocated before calling this function.
 * @param array the input array, containing the data values
 * @param n the number of elements of this array
 * @param n_array the array where the distribution will be written
 * @param x_array the array that will contain the centers of each bin
 * @param n_bins the number of desired bins for the distribution
 */
template <typename T>
void hist(const T* array, uint n, int* n_array, T* x_array, uint n_bins) {
  T miny = *std::min_element(array, array+n);
  T maxy = *std::max_element(array, array+n);

  // x contains the center of the bins
  for (uint i=0; i<n_bins; i++) {
    x_array[i] = (0.5 + i)*(maxy - miny)/n_bins + miny;
  }

  // cutoff contains the boundaries between the bins
  std::vector<T> cutoff(n_bins - 1);
  for (uint i=0; i<n_bins-1; i++) {
    cutoff[i] = (x_array[i] + x_array[i+1]) / 2.0;
  }

  // algo: either build the cumulated histogram by 2-level
  //       for-loops, and then build the diff, or first
  //       sort the distribution and do it directly.
  // 1st version: O(n^2) time, O(1) space
  // 2nd version: O(n·log(n)) time, O(n) space

  // implementation of 2nd version
  std::vector<T> dist(array, array+n);
  std::sort(dist.begin(), dist.end());
  uint current_cutoff_idx = 0;
  T current_cutoff = cutoff[0];
  for (uint i=0; i<n_bins; i++) n_array[i] = 0;

  for (uint i=0; i<n; i++) {
    while (dist[i] > current_cutoff) {
      // last case; skip the rest and fill in the last bin
      if (current_cutoff_idx == n_bins-2) {
    n_array[n_bins-1] = n-i; // fill in the last bin with what's left
    i = n; // to jump out of the 2nd loop (the 'for' one)
    n_array[n_bins-2]--; // to compensate for the last one that will be added before jumping out of the loop
    break;
      }
      current_cutoff_idx++;
      current_cutoff = cutoff[current_cutoff_idx];
    }
    n_array[current_cutoff_idx]++;
  }
}


/**
 * returns in output the number of occurence of each value in the input vector
 */
template <typename T>
void bincount(const std::vector<T>& input, std::vector<T>& output) {
  output.clear();
  output.resize( (int) ( std::max<Real>( input[argmax(input)], 0.) + 0.5 ) + 1);
  uint index = 0;
  for (uint i=0; i< input.size(); i++) {
    index = int(std::max<Real>(input[i],0) + 0.5);
    if (index < output.size() ) {
      output[index] += 1.;
    }
  }
}


/**
 * Transpose the given matrix. This function throws an exception if all the rows
 * do not have the same size.
 */
template <typename T>
std::vector<std::vector<T> > transpose(const std::vector<std::vector<T> >& m) {
  if (m.empty()) return std::vector<std::vector<T> >();

  int nrows = m.size();
  int ncols = m[0].size();
  for (int i=1; i<nrows; i++) {
    if ((int)m[i].size() != ncols) {
      std::ostringstream ss;
      ss <<"Trying to transpose a non rectangular matrix. Expecting dim2 = " << ncols
         << " but got " << m[i].size() << ". Cannot transpose!";
      throw EssentiaException(ss.str());
    }
  }

  std::vector<std::vector<T> > result(ncols, std::vector<Real>(nrows));
  for (int i=0; i<nrows; i++) {
    for (int j=0; j<ncols; j++) {
      result[j][i] = m[i][j];
    }
  }

  return result;
}

template <typename T>
TNT::Array2D<T> transpose(const TNT::Array2D<T>& m) {
  if (m.dim1() == 0) return TNT::Array2D<T>();

  int nrows = m.dim1();
  int ncols = m.dim2();

  TNT::Array2D<T> result(ncols, nrows);
  for (int i=0; i<nrows; i++) {
    for (int j=0; j<ncols; j++) {
      result[j][i] = m[i][j];
    }
  }

  return result;
}

inline std::string equivalentKey(const std::string key) {
  if (key == "C")
    return "C";

  if (key == "C#")
    return "Db";

  if (key == "Db")
    return "C#";

  if (key == "D")
    return "D";

  if (key == "D#")
    return "Eb";

  if (key == "Eb")
    return "D#";

  if (key == "E")
    return "E";

  if (key == "F")
    return "F";

  if (key == "F#")
    return "Gb";

  if (key == "Gb")
    return "F#";

  if (key == "G")
    return "G";

  if (key == "G#")
    return "Ab";

  if (key == "Ab")
    return "G#";

  if (key == "A")
    return "A";

  if (key == "A#")
    return "Bb";

  if (key == "Bb")
    return "A#";

  if (key == "B")
    return "B";

  return "";
}


/** Circularly rotate an input chromagram by an specified optimal transposition index (oti).
 * Expects input chromagram to be in the shape (frames , n_bins) where frames is no of frames and n_bins is no of chroma bins.
 * Throws an exception if the input chromagram is empty.
 */
template <typename T>
void rotateChroma(std::vector<std::vector<T> >& inputMatrix, int oti) {
  if (inputMatrix.empty())
    throw EssentiaException("rotateChroma: trying to rotate an empty matrix");
  for (size_t i=0; i<inputMatrix.size(); i++) {
    std::rotate(inputMatrix[i].begin(), inputMatrix[i].end() - oti, inputMatrix[i].end());
  }
}


/**
 * returns the dot product of two 1D vectors.
 * Throws an exception either one of the input arrays are empty.
 */
template <typename T>
T dotProduct(const std::vector<T>& xArray, const std::vector<T>& yArray) {
  if (xArray.empty() || yArray.empty())
    throw EssentiaException("dotProduct: trying to calculate the dotProduct of empty arrays!");
  return std::inner_product(xArray.begin(), xArray.end(), yArray.begin(), 0.0);
}


/**	
 * returns the q-th percentile of an 1D input array (same as numpy percentile implementation).	
 * Throws an exception if the input array is empty.	
 */ 
template <typename T> T percentile(const std::vector<T>& array, Real qpercentile) {
  if (array.empty())
    throw EssentiaException("percentile: trying to calculate percentile of empty array");

  std::vector<T> sorted_array = array;
  // sort the array
  std::sort(sorted_array.begin(), sorted_array.end());
  qpercentile /= 100.;

  Real k;
  int sortArraySize = sorted_array.size();
  if (sortArraySize > 1) {
    k = (sortArraySize - 1) * qpercentile;
  }
  else {
    // to avoid zero value in arrays with single element
    k = sortArraySize * qpercentile;
  }
  // apply interpolation
  Real d0 = sorted_array[int(std::floor(k))] * (std::ceil(k) - k);
  Real d1 = sorted_array[int(std::ceil(k))] * (k - std::floor(k));
  return d0 + d1;
}


/**
 * Sample covariance
 */
template <typename T> T covariance(const std::vector<T>& x, const T xMean, const std::vector<T>& y, const T yMean) {
  if (x.empty())
    throw EssentiaException("trying to calculate covariance of empty array");
  if (y.empty())
    throw EssentiaException("trying to calculate covariance of empty array");
  if (x.size() != y.size())
    throw EssentiaException("x and y should have the same size");

  T cov = (T) 0.0;

  for (uint i=0; i<x.size(); i++) {
    cov += (x[i] - xMean) * (y[i] - yMean);
  }

  return (T)(cov / (Real)x.size());
}


/**
 * Returns the sample Pearson correlation coefficient of a pair of vectors as described in,
 * https://en.wikipedia.org/wiki/Pearson_correlation_coefficient
 */
template <typename T> T pearsonCorrelationCoefficient(const std::vector<T>& x, const std::vector<T>& y) {
  if (x.empty())
    throw EssentiaException("trying to calculate covariance of empty array");
  if (y.empty())
    throw EssentiaException("trying to calculate covariance of empty array");
  if (x.size() != y.size())
    throw EssentiaException("x and y should have the same size");

  T xMean = mean(x);
  T yMean = mean(y);
  
  T cov = covariance(x, xMean, y, yMean);

  T xStddev = stddev(x, xMean);
  T yStddev = stddev(y, yMean);

  // When dealing with constants corraltion is 0 by convention. 
  if ((xStddev == (T)0.0) || (xStddev == (T)0.0) || (xStddev == (T)0.0)) return (T) 0.0;
  
  T corr = cov / (xStddev * yStddev);

  // Numerical error can yield results slightly outside the analytical range [-1, 1].
  // Clipping the output is a cheap way to mantain this contrain.
  // Seen in https://github.com/numpy/numpy/blob/v1.15.0/numpy/lib/function_base.py#L2403-L2406
  return std::max(std::min(corr, (T)1.0), (T)-1.0);
}


/**
 * Apply heaviside step function to an input m X n dimentional vector.
 * f(x) = if x<0: x=0; if x>=0: x=1
 * Throws an exception if the input array is empty.
 * returns a 2D binary vector of m X n shape
 */
template <typename T>
void heavisideStepFunction(std::vector<std::vector<T> >& inputArray) {
  if (inputArray.empty())
    throw EssentiaException("heavisideStepFunction: found empty array as input!");

  for (size_t i=0; i<inputArray.size(); i++) {
    for (size_t j=0; j<inputArray[i].size(); j++) {
      // initialize all non negative elements as zero otherwise as one
      inputArray[i][j] = (inputArray[i][j] < 0) ? 0 : 1;
    }
  }
}


/**
 * Pairwise euclidean distances between two 2D vectors.
 * Throws an exception if the input array is empty.
 * Returns a (m.shape[0], n.shape[0]) dimentional vector where m and n are the two input arrays
 * TODO: [add other distance metrics beside euclidean such as cosine, mahanalobis etc as a configurable parameter]
 */
template <typename T>
std::vector<std::vector<T> > pairwiseDistance(const std::vector<std::vector<T> >& m, const std::vector<std::vector<T> >& n) {

  if (m.empty() || n.empty())
    throw EssentiaException("pairwiseDistance: found empty array as input!");

  size_t mSize = m.size();
  size_t nSize = n.size();
  std::vector<std::vector<T> > pdist(mSize, std::vector<T>(nSize));
  for (size_t i=0; i<mSize; i++) {
      for (size_t j=0; j<nSize; j++) {
          Real item = dotProduct(m[i], m[i]) - 2*dotProduct(m[i], n[j]) + dotProduct(n[j], n[j]);
          pdist[i][j] = sqrt(item);
      }
  }
  if (pdist.empty())
      throw EssentiaException("pairwiseDistance: outputs an empty similarity matrix!");
  return pdist;
}

/**
 * Sets `squeezeShape`, `summarizerShape`, `broadcastShape` to perform operations 
 * on a Tensor with the shape of `tensor` along the `axis` dimension.
 */
template <typename T>
void tensorGeometricalInfo(const Tensor<T>& tensor, int& axis,
                           std::array<Eigen::Index, TENSORRANK - 1>& squeezeShape,
                           std::array<Eigen::Index, TENSORRANK>& summarizerShape,
                           std::array<Eigen::Index, TENSORRANK>& broadcastShape) {
  // To perform tensor operations along an specific axis we need to get
  // some geometrical information before:

  // First, an array with dimensions to squeeze (all but the axis).
  int i = 0;
  for (int j = 0; j < TENSORRANK; j++) {
    if (j != axis) {
      squeezeShape[i] = j;
      i++;
    }
  }

  // An array with all singleton dimension but the axis of interest.
  summarizerShape = {1, 1, 1, 1};
  summarizerShape[axis] = tensor.dimension(axis);

  // An array with the number of times we need to copy the accumulator
  // tensors per dimension in order to match the input tensor shape.
  broadcastShape = tensor.dimensions();
  broadcastShape[axis] = 1;
}

/**
 * Returns the mean of a tensor.
 */
template <typename T>
T mean(const Tensor<T>& tensor) {
  return (T)((TensorScalar)tensor.mean())(0);
}

/**
 * Returns the mean of a tensor along the given axis.
 */
template <typename T>
Tensor<T> mean(const Tensor<T>& tensor, int axis) {
  std::array<Eigen::Index, TENSORRANK - 1> squeezeShape;
  std::array<Eigen::Index, TENSORRANK> summarizerShape, broadcastShape;

  tensorGeometricalInfo(tensor, axis, squeezeShape, summarizerShape, broadcastShape);
  Tensor1D means = tensor.mean(squeezeShape);

  return TensorMap<Real>(means.data(), summarizerShape);
}

/**
 * Returns the standard deviation of a tensor.
 */
template <typename T>
T stddev(const Tensor<T>& tensor, const T mean) {
  // Substract the mean.
  Tensor<Real> tmp = tensor - tensor.constant(mean);
  // Sum of squares.
  Real sos = ((TensorScalar)tmp.pow(2).sum())(0);

  return sqrt(sos / tensor.size());
}

/**
 * Returns the standard deviation of a tensor along the given axis.
 */
template <typename T>
Tensor<T> stddev(const Tensor<T>& tensor, const Tensor<T> mean, int axis) {
  std::array<Eigen::Index, TENSORRANK - 1> squeezeShape;
  std::array<Eigen::Index, TENSORRANK> summarizerShape, broadcastShape;

  tensorGeometricalInfo(tensor, axis, squeezeShape, summarizerShape, broadcastShape);

  // Get the number of elements on each sub-tensor.
  Real normalization = tensor.size() / tensor.dimension(axis);

  // Substract the means along the axis using broadcast to replicate
  // them along the rest of dimensions.
  Tensor<Real> tmp = tensor - mean.broadcast(broadcastShape);

  // Cumpute the sum of squares.
  Tensor1D sos = tmp.pow(2).sum(squeezeShape);

  // Compute the standard deviations and put them into a Tensor along the axis.
  Tensor1D stds = (sos / normalization).sqrt();

  return TensorMap<Real>(stds.data(), summarizerShape);
}

/**
 * Returns the minimum of a tensor.
 */
template <typename T>
T tensorMin(const Tensor<T>& tensor) {
  return (T)((TensorScalar)tensor.minimum())(0);
}

/**
 * Returns the minimum of a tensor along the given axis.
 */
template <typename T>
Tensor<T> tensorMin(const Tensor<T>& tensor, int axis) {
  std::array<Eigen::Index, TENSORRANK - 1> squeezeShape;
  std::array<Eigen::Index, TENSORRANK> summarizerShape, broadcastShape;

  tensorGeometricalInfo(tensor, axis, squeezeShape, summarizerShape, broadcastShape);
  Tensor1D minima = tensor.minimum(squeezeShape);

  return TensorMap<Real>(minima.data(), summarizerShape);
}

/**
 * Returns the maximum of a tensor.
 */
template <typename T>
T tensorMax(const Tensor<T>& tensor) {
  return (T)((TensorScalar)tensor.maximum())(0);
}

/**
 * Returns the maximum of a tensor along the given axis.
 */
template <typename T>
Tensor<T> tensorMax(const Tensor<T>& tensor, int axis) {
  std::array<Eigen::Index, TENSORRANK - 1> squeezeShape;
  std::array<Eigen::Index, TENSORRANK> summarizerShape, broadcastShape;

  tensorGeometricalInfo(tensor, axis, squeezeShape, summarizerShape, broadcastShape);
  Tensor1D maxima = tensor.maximum(squeezeShape);

  return TensorMap<Real>(maxima.data(), summarizerShape);
}

/**
 * Rounds x up to the desired decimal place.
 */
template <typename T>
T roundToDecimal(T x, int decimal) {
  if (decimal < 0) {
    throw EssentiaException("the number of decimals has to be 0 or positive");
  }
  return round(pow(10, decimal) * x) / pow(10, decimal);
}

} // namespace essentia

#endif // ESSENTIA_MATH_H
