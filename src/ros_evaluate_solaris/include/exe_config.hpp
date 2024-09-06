#ifndef CONFIG_HPP_
#define CONFIG_HPP_

#include <stdlib.h>
#include <stdint.h>
#include <time.h>

//任务链数目
//节点数目
#define     NODE_NUM            7
//容器数量
#define     CONTAINER_NUM       2

extern int container_node_num[CONTAINER_NUM];
extern uint64_t container_id;

//线程数目
#define     TID_NUM             3
//回调函数参数
#define     C1_Period_MS    1000ms
#define     C1_Exec_MS      1
#define     C1_Msg_Size     10
#define     C2_Exec_MS      1
#define     C3_Exec_MS      1
#define     C3_Msg_Size     10
#define     C4_Exec_MS      1
#define     C5_Period_MS    1000ms
#define     C5_Exec_MS      1
#define     C5_Msg_Size     10
#define     C6_Exec_MS      1
#define     C6_Msg_Size     10
#define     C7_Exec_MS      1

static inline uint64_t get_clocktime() {
    long int        ns;
    uint64_t        all;
    time_t          sec;
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    sec   = spec.tv_sec;
    ns    = spec.tv_nsec;
    all   = static_cast<uint64_t> (sec) * 1000000000UL + static_cast<uint64_t> (ns);
    return all;  
}

#define DUMMY_LOAD_ITER	      100

int dummy_load_calib = 1;

static inline void dummy_load_ms(int load_ms) {
  int i, j;
  for (j = 0; j < dummy_load_calib * load_ms; j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}

static inline void dummy_load_100us(int load_100us) {
  int i, j;
  for (j = 0; j < (dummy_load_calib * load_100us /10); j++)
      for (i = 0 ; i < DUMMY_LOAD_ITER; i++) 
          __asm__ volatile ("nop");
}

static inline void dummy_load_calibration() {
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
  printf("|CALIBRATION TEST|[Setting: 10ms]->@time-measure: %lu. \r\n", ts_end-ts_start);
}

#endif