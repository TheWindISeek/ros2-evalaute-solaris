#ifndef ROS_EVALUATE_SOLARIS__UTILS_HPP_
#define ROS_EVALUATE_SOLARIS__UTILS_HPP_

#include <chrono>
#include <pthread.h>  
#include <stdio.h>

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


inline int set_cpu_test(int which_cpu) {  
    cpu_set_t mask;
    CPU_ZERO(&mask);
  
    CPU_SET(which_cpu, &mask);  
  
    // printf("thread %lu, i = %d\n", pthread_self(), which_cpu);  
    if(-1 == pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask)) {  
        return -1;  
    }  
    return 0;
}

#endif  // !ROS_EVALUATE_SOLARIS__UTILS_HPP_