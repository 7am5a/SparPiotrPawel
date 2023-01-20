#ifndef SAMPLEFILTER_H_
#define SAMPLEFILTER_H_

/*

FIR filter designed with
 http://t-filter.appspot.com

sampling frequency: 100 Hz

* 0 Hz - 10 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = 6.73340759652856 dB

* 20 Hz - 50 Hz
  gain = 0
  desired attenuation = -45 dB
  actual attenuation = -41.105687178116376 dB

*/

#define SAMPLEFILTER_TAP_NUM 10

typedef struct {
  double history[SAMPLEFILTER_TAP_NUM];
  unsigned int last_index;
} SampleFilter;

void SampleFilter_init(SampleFilter* f);
void SampleFilter_put(SampleFilter* f, double input);
double SampleFilter_get(SampleFilter* f);

#endif