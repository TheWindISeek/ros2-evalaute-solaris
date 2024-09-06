#ifndef ROS_EVALUATE_SOLARIS__CONFIG_HPP_
#define ROS_EVALUATE_SOLARIS__CONFIG_HPP_

#include <stdint.h>
#include <stdio.h>
#include "utils.hpp"

void chain_start_print(size_t count, uint32_t chain_idx, uint32_t node_idx) {
    uint64_t cur_time = get_clocktime();
    // printf("chain:%u-Begin:%u-start:%ld\n", chain_idx, node_idx, start_time);
    printf("chain:%u-node:%u-count:%lu-start:%ld\n", chain_idx, node_idx, count, cur_time);
}

void chain_end_print(size_t count, uint32_t chain_idx, uint32_t node_idx) {
    // printf("chain:%u-Begin:%u-end:%ld-exe:%ld\n", chain_idx, node_idx, end_time, end_time - start_time);
    uint64_t cur_time = get_clocktime();
    printf("chain:%u-node:%u-count:%lu-end:%ld\n", chain_idx, node_idx, count, cur_time);
}

#endif  // !ROS_EVALUATE_SOLARIS__CONFIG_HPP_