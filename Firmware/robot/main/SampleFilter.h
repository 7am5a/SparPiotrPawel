#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 100 Hz

* 0 Hz - 10 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 0.00025711532314909665 dB

* 40 Hz - 50 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = -124.04747396899853 dB

*/

#define SAMPLEFILTER_TAP_NUM 100

typedef struct {
  double history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, double input);
double SampleFilter_get(SampleFilter* f);

#endif