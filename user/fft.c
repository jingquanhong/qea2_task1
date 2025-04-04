#include "fft.h"

arm_cfft_radix4_instance_f32 scfft;


uint16_t ADC_1_Value_DMA[FFT_LENGTH]={0};//ADC��������


float fft_inputbuf[FFT_LENGTH * 2];  // FFT �������루ʵ��+�鲿��
float fft_outputbuf[FFT_LENGTH*2];     // FFT ��ֵ���

SignalInfo_t signal_info_real;//�����ź���Ϣ

void FFT_INIT(void)
{
	  arm_cfft_radix4_init_f32(&scfft,FFT_LENGTH,0,1);
	
		HAL_TIM_Base_Start(&htim3);//tim3��ʱ����ADc
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_1_Value_DMA, FFT_LENGTH);
	
}



//����FFT�������
SignalInfo_t analyze_fft(float *fft_outputbuf, int fft_length, float fs) 
{
    SignalInfo_t signal_info = {0};

    // ֱ������
    signal_info.dc_offset = fft_outputbuf[0] / fft_length;

    float max_value1 = 0, max_value2 = 0;
    int max_index1 = 0, max_index2 = 0;

    
    for (int i = 1; i < fft_length / 2; i++) {
        if (fft_outputbuf[i] > max_value1) {
            max_value2 = max_value1;  
            max_index2 = max_index1;
            max_value1 = fft_outputbuf[i];
            max_index1 = i;
        } else if (fft_outputbuf[i] > max_value2) {
            max_value2 = fft_outputbuf[i];
            max_index2 = i;
        }
    }

    // ����г��Ƶ�ʢ�
    signal_info.main_freq = (float)max_index1 * fs / fft_length;
    signal_info.second_freq = (float)max_index2 * fs / fft_length;

    // �����ֵ
    signal_info.main_amp = (2.0f * max_value1) / fft_length;
    signal_info.second_amp = (2.0f * max_value2) / fft_length;

    return signal_info;
}

//ת��ADC Ȼ�����FFT����
SignalInfo_t capture_and_FFT(uint32_t FFT_LENGTH_, uint16_t *ADC_1_Value_DMA, float SAMPLING_RATE_)
{
	  SignalInfo_t signal_info = {0};

	  

    arm_cfft_radix4_instance_f32 scfft;  // FFT �ṹ��
    

    // ��ʼ�� FFT �ṹ
    arm_cfft_radix4_init_f32(&scfft, FFT_LENGTH_, 0, 1);

    // ���� FFT ��������
    for (uint32_t i = 0; i < FFT_LENGTH_; i++) {
        fft_inputbuf[2 * i] = ADC_1_Value_DMA[i];  // ʵ��
        fft_inputbuf[2 * i + 1] = 0;              // �鲿
    }

    // ִ�� FFT
    arm_cfft_radix4_f32(&scfft, fft_inputbuf);

    // �����ֵ
    arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH_);

    // ����Ƶ�׷��������Լ��� analyze_fft ������
    signal_info = analyze_fft(fft_outputbuf, FFT_LENGTH_, SAMPLING_RATE_);
     
		return signal_info;
}





