//
// Created by yue on 2023/12/10.
//

#ifndef DRAFT_LOWPASS_FILTER_H
#define DRAFT_LOWPASS_FILTER_H

typedef struct
{
    float k;
    float last_value;
    float curr_value;
} LowPassFilter_t;

#endif //DRAFT_LOWPASS_FILTER_H
