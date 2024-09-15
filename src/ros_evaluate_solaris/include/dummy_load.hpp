// Copyright (c) 2024 JeffreySharp
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS_EVALUATE_SOLARIS__DUMMY_LOAD_HPP_
#define ROS_EVALUATE_SOLARIS__DUMMY_LOAD_HPP_

#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include "utils.hpp"

//----- 工具函数和相关配置
const int DUMMY_LOAD_ITER = 100;
int dummy_load_calib = 1;


// static inline void 
// dummy_load_ms(int load_ms) {
//   int i, j;
//   for (j = 0; j < dummy_load_calib * load_ms; j++)
//       for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
//           __asm__ volatile ("nop");
// }

static inline void 
dummy_load_ms(int load_ms) {
    int i, j;
    for (j = 0; j < dummy_load_calib * load_ms; j++)
        for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
            __asm__ volatile ("cpuid" ::: "eax", "ebx", "ecx", "edx");
}

static inline void 
dummy_load_100us(int load_100us) {
  int i, j;
  for (j = 0; j < (dummy_load_calib * load_100us /10); j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}

static inline void 
dummy_load_calibration() {
  volatile uint64_t ts_start, ts_end;
  while(1) {
    ts_start = get_clocktime(); // in ns
    dummy_load_ms(100);         // 100ms
    ts_end = get_clocktime();   // in ns
    uint64_t duration_ns = ts_end - ts_start;
    if (abs((int)duration_ns - 100*1000*1000) < 1000000) {// error margin: 1ms
      break;
    }
    dummy_load_calib = 100*1000*1000*dummy_load_calib / duration_ns;
    if (dummy_load_calib <= 0) {
      dummy_load_calib = 1;
    }
  }
  ts_start = get_clocktime(); // in ns
  dummy_load_ms(10);          // 10ms
  ts_end = get_clocktime();   // in ns
//   printf("|CALIBRATION TEST|[Setting: 10ms]->@time-measure: %lu. \r\n", ts_end-ts_start);
}

#endif // !ROS_EVALUATE_SOLARIS__DUMMY_LOAD_HPP_
