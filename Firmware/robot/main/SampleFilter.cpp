#include "SampleFilter.h"

static double filter_taps[SAMPLEFILTER_TAP_NUM] = {
  0.016805056324617446,
  0.06534501139706088,
  0.13340730476781168,
  0.21027529629892022,
  0.2588146479830493,
  0.2588146479830493,
  0.21027529629892022,
  0.13340730476781168,
  0.06534501139706088,
  0.016805056324617446
};

void SampleFilter_init(SampleFilter* f) {
  int i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i)
    f->history[i] = 0;
  f->last_index = 0;
}

void SampleFilter_put(SampleFilter* f, double input) {
  f->history[f->last_index++] = input;
  if(f->last_index == SAMPLEFILTER_TAP_NUM)
    f->last_index = 0;
}

double SampleFilter_get(SampleFilter* f) {
  double acc = 0;
  int index = f->last_index, i;
  for(i = 0; i < SAMPLEFILTER_TAP_NUM; ++i) {
    index = index != 0 ? index-1 : SAMPLEFILTER_TAP_NUM-1;
    acc += f->history[index] * filter_taps[i];
  };
  return acc;
}
