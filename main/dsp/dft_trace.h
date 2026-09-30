#ifndef DFT_TRACE_H
#define DFT_TRACE_H

#include <stdint.h>

void dft_trace_frame(
   const int16_t *frame,
   float frame_mean
);

#endif