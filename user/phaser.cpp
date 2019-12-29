/*
Copyright 2019 Tomoaki Itoh
This software is released under the MIT License, see LICENSE.txt.
//*/

#include "usermodfx.h"
#include "buffer_ops.h"
#include "LCWLowFreqOsc.h"
#include "LCWFixedMath.h"
#include "LCWTanhPow2Table.h"

#define PHASER_STAGE (8)
#define PHASER_RAM_SIZE (PHASER_STAGE + 1)

static __sdram float s_phaser_ram_z1[PHASER_RAM_SIZE];
static __sdram float s_phaser_ram_out[PHASER_RAM_SIZE];

static float s_inputGain;
static float s_time;
static float s_depth;

static LCWLowFreqOscBlock s_lfo;

__fast_inline int32_t clip_lfoOut(SQ15_16 lfoOut, SQ15_16 depth)
{
  const SQ15_16 center = LCW_SQ15_16(0.25);

  // lfoOut(= 0.0 .. 0.5) -> -depth .. +depth
  const SQ15_16 x = ((lfoOut - center) * depth) >> (16 - 2);

  if ( x < LCW_SQ15_16(-2.0) ) {
    // return 0.0
    return gLcwTanhPow2Table[0] << (16 - LCW_TANH_POW2_VALUE_BITS);
  }
  else if ( LCW_SQ15_16(2.0) <= x ) {
    // return 1.0
    return gLcwTanhPow2Table[LCW_TANH_POW2_TABLE_SIZE - 1] << (16 - LCW_TANH_POW2_VALUE_BITS);
  }
  else {
    uint32_t x2 = (uint32_t)( x + LCW_SQ15_16(2.0) );
    uint32_t i = x2 >> LCW_TANH_POW2_FRACTION_BITS;
    uint32_t frac = x2 & (0xFFFF >> LCW_TANH_POW2_FRACTION_BITS);

    int32_t val = (int32_t)gLcwTanhPow2Table[i] << (16 - LCW_TANH_POW2_VALUE_BITS);
    int32_t diff = ((int32_t)gLcwTanhPow2Table[i+1] << (16 - LCW_TANH_POW2_VALUE_BITS)) - val;

    // return 0.0 .. 1.0
    return val + ((diff * frac) >> (16 - LCW_TANH_POW2_FRACTION_BITS));
  }
}

// limit : equal or more than 1.f
__fast_inline float softlimiter(float c, float x, float limit)
{
  float th = limit - 1.f + c;
  float xf = si_fabsf(x);
  if ( xf < th ) {
    return x;
  }
  else {
    return si_copysignf( th + fx_softclipf(c, xf - th), x );
  }
}

void MODFX_INIT(uint32_t platform, uint32_t api)
{
  s_time = 0.25f;
  s_depth = 0.5f;
  s_inputGain = 0.f;
  
  s_lfo.dt = 0x10000;
  s_lfo.th = (int32_t)(0.5f * 0x10000);
  s_lfo.out = 0.f;
  s_lfo.out2 = 0.f;
}

void MODFX_PROCESS(const float *main_xn, float *main_yn,
                   const float *sub_xn,  float *sub_yn,
                   uint32_t frames)
{
  const float wet = 0.75f * s_depth;
  const float dry = 1.f - wet;

  const float * mx = main_xn;
  float * __restrict my = main_yn;
  const float * my_e = my + 2*frames;
  const float * sx = sub_xn;
  float * __restrict sy = sub_yn;

  float * __restrict z1 = &s_phaser_ram_z1[0];
  float * __restrict out = &s_phaser_ram_out[0];

  int32_t time = q16_pow2( LCW_SQ15_16(-4.2f + ((1.f - s_time) * 8.2f)) );
  s_lfo.dt = LCW_LFO_TIMER_MAX / (uint32_t)(((time >> 4) * (48000 >> 6)) >> 6);
  SQ15_16 depth = LCW_SQ15_16( s_depth );

  for (; my != my_e; ) {
    float xL = *mx;

    lfo_inc( &s_lfo );

    // s15.16 -> 0.0 .. 1.0
    float lfoOut = clip_lfoOut(s_lfo.out2 - LCW_SQ15_16(0.02), depth) / (float)(1 << 16);
    float b0 = ((lfoOut - 0.5f) * 1.89f) + 0.025f;

    int32_t i = PHASER_STAGE;
    out[0] = (xL * s_inputGain * dry) + (out[i] * wet);
    for (; 0<i; i--) {
      float wk = out[i-1] - (b0 * z1[i]); // b0 = a1
      out[i] = (b0 * wk) + z1[i]; // b1 = 1.f
      z1[i] = wk;
    }

    float yL = softlimiter( 0.05f, out[1], 1.f );

    mx += 2;
    sx += 2;
    *(my++) = yL;
    *(my++) = yL;
    *(sy++) = yL;
    *(sy++) = yL;

    if ( s_inputGain < 0.99998f ) {
      s_inputGain += ( (1.f - s_inputGain) * 0.0625f );
    }
    else { s_inputGain = 1.f; }
  }
}

void MODFX_RESUME(void)
{
  buf_clr_f32(
    (float * __restrict__)s_phaser_ram_z1,
    PHASER_RAM_SIZE );
  buf_clr_f32(
    (float * __restrict__)s_phaser_ram_out,
    PHASER_RAM_SIZE );
  s_inputGain = 0.f;
}

void MODFX_PARAM(uint8_t index, int32_t value)
{
  const float valf = q31_to_f32(value);
  switch (index) {
  case k_user_modfx_param_time:
    s_time = clip01f(valf);
    break;
  case k_user_modfx_param_depth:
    s_depth = clip01f(valf);
    break;
  default:
    break;
  }
}
