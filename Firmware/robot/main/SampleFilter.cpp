#include "SampleFilter.h"

static double filter_taps[SAMPLEFILTER_TAP_NUM] = {
  -8.197350783990626e-7,
  -0.0000018359267390763174,
  -0.000002608998977860687,
  -0.0000010043582646677392,
  0.000007174610955668735,
  0.00002882215214732658,
  0.00007359665139996495,
  0.00015307681349625442,
  0.00027869197765798074,
  0.00045833559265837093,
  0.0006919435767362891,
  0.000966786563561379,
  0.0012536782822055678,
  0.0015056032054382878,
  0.0016602554068350362,
  0.0016475435102330873,
  0.0014022145888360578,
  0.000880468545728976,
  0.00007798802655741681,
  -0.0009544679823831958,
  -0.002102475627627688,
  -0.0031907326074166005,
  -0.00400025056525724,
  -0.004300514814679652,
  -0.0038938666226687817,
  -0.0026657997539317967,
  -0.000631822060056067,
  0.0020301413957364055,
  0.0049724341494008385,
  0.007703236069131566,
  0.009647647921133254,
  0.0102400975098192,
  0.009036574487272976,
  0.005828866777606052,
  0.0007391914490742467,
  -0.00572646774688976,
  -0.012683724269759711,
  -0.01894288373925752,
  -0.02314745131769045,
  -0.023963280199797463,
  -0.020292401849363286,
  -0.011477978968185643,
  0.0025353414760551063,
  0.021117162371687183,
  0.042964290094753416,
  0.06621703021049219,
  0.0886677787452804,
  0.10803459897806078,
  0.12226017831810786,
  0.12979066067627826,
  0.12979066067627826,
  0.12226017831810786,
  0.10803459897806078,
  0.0886677787452804,
  0.06621703021049219,
  0.042964290094753416,
  0.021117162371687183,
  0.0025353414760551063,
  -0.011477978968185643,
  -0.020292401849363286,
  -0.023963280199797463,
  -0.02314745131769045,
  -0.01894288373925752,
  -0.012683724269759711,
  -0.00572646774688976,
  0.0007391914490742467,
  0.005828866777606052,
  0.009036574487272976,
  0.0102400975098192,
  0.009647647921133254,
  0.007703236069131566,
  0.0049724341494008385,
  0.0020301413957364055,
  -0.000631822060056067,
  -0.0026657997539317967,
  -0.0038938666226687817,
  -0.004300514814679652,
  -0.00400025056525724,
  -0.0031907326074166005,
  -0.002102475627627688,
  -0.0009544679823831958,
  0.00007798802655741681,
  0.000880468545728976,
  0.0014022145888360578,
  0.0016475435102330873,
  0.0016602554068350362,
  0.0015056032054382878,
  0.0012536782822055678,
  0.000966786563561379,
  0.0006919435767362891,
  0.00045833559265837093,
  0.00027869197765798074,
  0.00015307681349625442,
  0.00007359665139996495,
  0.00002882215214732658,
  0.000007174610955668735,
  -0.0000010043582646677392,
  -0.000002608998977860687,
  -0.0000018359267390763174,
  -8.197350783990626e-7
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
