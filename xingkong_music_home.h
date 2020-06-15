#include "esphome.h"
using namespace esphome;

#include "driver/i2s.h"
//#include <FastLED.h>
#include "FFT.h"
#include "VisualEffect.h"

enum PLAYMODE
{
    MODE_SCROLL,
    MODE_ENERGY,
    MODE_SPECTRUM
};

class MusicLeds
{
private:
    //N_PIXELS: The number of the LEDS on the led strip, must be even.
    static const uint16_t N_PIXELS = 100;
    //MIN_VOLUME_THRESHOLD: If the audio's volume is less than this number, the signal will not be processed.
    static constexpr float MIN_VOLUME_THRESHOLD = 0.0003;
    //Microphone(type of PDM)'s WS Pin and DATA_IN Pin, connecting to GPIO
    static const int I2S_WS = 25;
    static const int I2S_SD = 22;
    static const int I2S_SCK = 26;

    // static const int PWMPin = 14;
    // static const int PWMChannel = 0;
    // static const int resolution = 1; //resolution in bits
    // static const int dutyCycle = 1;  //value when using bits from resolutio

    static const uint16_t BUFFER_SIZE = 256;
    // static const uint16_t BUFFER_SIZE = 512;
    static const uint8_t N_ROLLING_HISTORY = 2;
    // static const uint16_t SAMPLE_RATE = 88200;
    static const uint16_t SAMPLE_RATE = 44100;
    static const uint16_t N_MEL_BIN = 18;
    static constexpr float MIN_FREQUENCY = 200;
    static constexpr float MAX_FREQUENCY = 8000;

    float y_data[BUFFER_SIZE * N_ROLLING_HISTORY];
    class FFT *fft;
    class VisualEffect *effect;

    CRGB physic_leds[N_PIXELS];

    i2s_port_t i2s_num = I2S_NUM_0; // i2s port number
    // i2s_config_t i2s_config = {
    //     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    //     .sample_rate = SAMPLE_RATE,
    //     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    //     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    //     .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S),

    //     .intr_alloc_flags = 0,
    //     // .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
    //     .dma_buf_count = 8,
    //     .dma_buf_len = 64,
    //     .use_apll = true};

    // i2s_pin_config_t pin_config = {
    //     .bck_io_num = I2S_SCK,
    //     .ws_io_num = I2S_WS,
    //     .data_out_num = -1,
    //     .data_in_num = I2S_SD};

    i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
        .communication_format = I2S_COMM_FORMAT_PCM,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 2,
        .dma_buf_len = 64,
        .use_apll = false};

    PLAYMODE CurrentMode = MODE_SCROLL;

    void kalman(int16_t* buffer, size_t length);
    int16_t LowPassFilter_kalman(int16_t data);
    void xingkongman(int16_t* buffer, size_t length);
public:
    MusicLeds();
    ~MusicLeds();
    void ShowFrame(PLAYMODE CurrentMode, light::AddressableLight *p_it);
};

MusicLeds::MusicLeds()
{
    i2s_driver_install(I2S_NUM_0,  &i2s_config, 0, NULL);
    // i2s_set_pin(I2S_NUM_0, &pin_config);
    // i2s_stop(I2S_NUM_0);
    // i2s_set_sample_rates(I2S_NUM_0, SAMPLE_RATE);
    // i2s_start(I2S_NUM_0);

    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_3);

    // // configure PWM
    // ledcSetup(PWMChannel, SAMPLE_RATE * 128, resolution);
    // // attach pin to be controlled
    // ledcAttachPin(PWMPin, PWMChannel);
    // //write PWM to output pin.
    // ledcWrite(PWMChannel, dutyCycle);

    fft = new FFT(BUFFER_SIZE * N_ROLLING_HISTORY, N_MEL_BIN, MIN_FREQUENCY, MAX_FREQUENCY, SAMPLE_RATE, MIN_VOLUME_THRESHOLD);
    effect = new VisualEffect(N_MEL_BIN, N_PIXELS);
}

MusicLeds::~MusicLeds()
{
    i2s_stop(I2S_NUM_0);
    delete fft;
    delete effect;
}

void MusicLeds::ShowFrame(PLAYMODE CurrentMode, light::AddressableLight *p_it)
{
    static float mel_data[N_MEL_BIN];

    for (int i = 0; i < N_ROLLING_HISTORY - 1; i++)
        memcpy(y_data + i * BUFFER_SIZE, y_data + (i + 1) * BUFFER_SIZE, sizeof(float) * BUFFER_SIZE);

    int16_t l[512];

    unsigned int read_num;
    i2s_adc_enable(I2S_NUM_0);
    i2s_read(I2S_NUM_0, l, 512 * 2, &read_num, portMAX_DELAY);
    i2s_adc_disable(I2S_NUM_0);

    //滤波
    kalman(l, BUFFER_SIZE);
    xingkongman(l, BUFFER_SIZE);
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        // ets_printf("%d\n", l[i]);

        y_data[BUFFER_SIZE * (N_ROLLING_HISTORY - 1) + i] = l[i] / 32768.0;
    }
    fft->t2mel(y_data, mel_data);

    switch (CurrentMode)
    {
    case MODE_SCROLL:
        effect->visualize_scroll(mel_data, physic_leds);
        break;
    case MODE_ENERGY:
        effect->visualize_energy(mel_data, physic_leds);
        break;
    case MODE_SPECTRUM:
        effect->visualize_spectrum(mel_data, physic_leds);
        break;
    }

    // for (int i = 0; i < p_it->size(); i++) {
    //     light::ESPColor c;
    //     c.r = physic_leds[i].r;
    //     c.g = physic_leds[i].g;
    //     c.b = physic_leds[i].b;
    //     c.w = 0;
    //     (*p_it)[i] = c;
    // }
    for (int i = 0; i < N_PIXELS; i++)
    {
        light::ESPColor c;
        c.r = physic_leds[i].r;
        c.g = physic_leds[i].g;
        c.b = physic_leds[i].b;
        c.w = 0;
        (*p_it)[i] = c;
        //两组RGB灯带镜像显示
        (*p_it)[i + 100] = c;
    }
}

/**
 * 使用卡尔曼滤波器过滤掉大部分波形中的大部分尖刺
 * 然后将信号反转并更新到数组中
 * 
 * 其中16383这个常数可能跟16000的采样率有关
 * 这是我通过观察波形得出的
 */
void MusicLeds::kalman(int16_t *buffer, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
        int16_t temp = LowPassFilter_kalman(buffer[i]);
        temp = 16383 - temp;
        if (temp < 0)
            temp = 0;
        buffer[i] = temp;
        // ets_printf("before: %d, after: %d\n",16383 - temp, 16383 - buffer[i]);
        // ets_printf("%d\n", temp);
    }
}

/**
 * 卡尔曼滤波器
 */
int16_t MusicLeds::LowPassFilter_kalman(int16_t data)
{
    static int16_t Xlast = 0; //初值 Xlast&Xpre
    static int16_t P = 1;     //Plast&Pnow
    const int16_t Q = 3000;   //自己感觉的方差
    const int16_t R = 5000;   //测量器件的方差
    int16_t Kg, PP;           //
    int16_t Xnow;             //经过卡尔曼滤波的值 Xnow

    //1式A=1 无输入
    PP = P + Q;                         //Ppre=Plast+Q （2）
    Kg = PP / (PP + R);                 //更新Kg Ppre/Ppre+R（4）
    Xnow = Xlast + Kg * (data - Xlast); // Xnow = Xpre+Kg*(Z(k)-H*Xpre)(3)
    P = (1 - Kg) * PP;                  //Pnow=(I-Kg)*Ppre(5) 由于Pnow不会再使用，所以更新Plast=Pnow
    if (data < Xnow)
    {
        Xnow = data;
    }
    Xlast = Xnow;
    return Xnow;
}

// void MusicLeds::xingkongman(int16_t* buffer, size_t length)
// {
//     size_t pos = 0;
//     size_t culength = 0;
//     while (pos < length)
//     {
//         if (pos + 20 < length)
//         {
//             culength = 20;
//         }
//         else
//         {
//             culength = length - pos;
//         }
        
//         //求和、求平均、找出最大值
//         int8_t maxPos = -1;
//         int16_t maxFreq = 0;
//         int16_t sum = 0;
//         for (int8_t i = pos; i < culength; i++)
//         {
//             int16_t value = buffer[i];

//             sum += value;
//             if (value >= maxFreq)
//             {
//                 maxFreq = value;
//                 maxPos = i;
//             }
//         }
//         sum /= culength;
//         if (maxFreq > sum && maxPos != -1)
//         {
//             buffer[maxPos] = sum;
//         }
        
//         pos += culength;
//     }
// }

// void MusicLeds::xingkongman(int16_t* buffer, size_t length)
// {
//     //求和、求平均、找出最大值
//     int16_t maxFreq = 0;
//     int16_t sum = 0;
//     for (size_t i = 0; i < length; i++)
//     {
//         int16_t value = buffer[i];

//         sum += value;
//         if (value >= maxFreq)
//         {
//             maxFreq = value;
//         }
//     }
//     sum /= length;
//     for (size_t i = 0; i < length; i++)
//     {
//         if (maxFreq > sum)
//         {
//             buffer[i] = sum;
//         }
//     }
// }

/**
 * 星空曼滤波器（？）
 * 由于观察到当没有音频正在播放的时候，波形中会出现随机的尖刺
 * 此处尝试将其过滤
 * 
 * 如果使用了此过滤器还是存在大量尖刺，有可能是ESP32没有接地
 */
void MusicLeds::xingkongman(int16_t* buffer, size_t length)
{
    int16_t zeroCount = 0;

    for (size_t i = 0; i < length; i++)
    {
        int16_t value = buffer[i];
        if (value < 5)
            zeroCount++;
    }

    if (zeroCount > length * 4 / 5)
    {
        for (size_t i = 0; i < length; i++)
        {
            buffer[i] = 0;
        }
    }
}

class MusicLeds music_leds;
