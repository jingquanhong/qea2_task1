#ifndef FFT_H
#define FFT_H

#include "arm_math.h"
#include "main.h"
#include "tim.h"
#include "adc.h"

#define FFT_LENGTH		1024 		//FFT�����С
#define SAMPLING_RATE 20000  // ADC����Ƶ��
//fft�����ṹ��
typedef struct {
    float dc_offset;   // ֱ������
    float main_freq;   // ��г��Ƶ��
    float main_amp;    // ��г����ֵ
    float second_freq; // �ڶ�г��Ƶ��
    float second_amp;  // �ڶ����ֵ
} SignalInfo_t;

void FFT_INIT(void);



SignalInfo_t capture_and_FFT(uint32_t FFT_LENGTH_, uint16_t *ADC_1_Value_DMA, float SAMPLING_RATE_);

#endif