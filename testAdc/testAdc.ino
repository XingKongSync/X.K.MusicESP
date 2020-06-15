#include <driver/i2s.h>
#include <driver/adc.h>
#define BUFFER_SIZE 256

void setup()
{
    Serial.begin(115200);
    Serial.println("Setup I2S");

    delay(1000);
    i2s_install();
    i2s_setadc();
}

void loop()
{
    i2s_adc_enable(I2S_NUM_0);

    int16_t l[BUFFER_SIZE];
    size_t read_num;
    esp_err_t ret = i2s_read(I2S_NUM_0, l, BUFFER_SIZE * 2, &read_num, portMAX_DELAY);
    //  Serial.println("ret: " + ret);
    //    ets_printf("ret %d\n",ret);
    read_num /=2;
    //滤波
    kalman(l, read_num);
    for (size_t i = 0; i < read_num; i++)
    {
        int16_t freq = 16128 - l[i];
        if (freq < 0)
            freq = 0;
//        Serial.println(freq);
//        ets_printf("after: %d\n",freq);
    }
    i2s_adc_disable(I2S_NUM_0);

    //    delay(1000);
}

void i2s_install()
{
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
        .communication_format = I2S_COMM_FORMAT_PCM,
        .intr_alloc_flags = 0, // default interrupt priority
        .dma_buf_count = 2,
        .dma_buf_len = 64,
        .use_apll = false};
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL));
}

void i2s_setadc()
{
    //ESP_ERROR_CHECK(adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_0));
    ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_3));
}

void kalman(int16_t* buffer, size_t length)
{
    for (size_t i = 0; i < length; i++)
    {
      int16_t temp = buffer[i];
        buffer[i] = LowPassFilter_kalman(buffer[i]);
        ets_printf("before: %d, after: %d\n",16383 - temp, 16383 - buffer[i]);
    }
    
}

int16_t LowPassFilter_kalman(int16_t data)
{
    static int16_t Xlast = 0; //初值 Xlast&Xpre
    static int16_t P = 1;      //Plast&Pnow
    const int16_t Q = 3000;      //自己感觉的方差
    const int16_t R = 5000;      //测量器件的方差
    int16_t Kg, PP;              //
    int16_t Xnow;                //经过卡尔曼滤波的值 Xnow

    //1式A=1 无输入
    PP = P + Q;                         //Ppre=Plast+Q （2）
    Kg = PP / (PP + R);                 //更新Kg Ppre/Ppre+R（4）
    Xnow = Xlast + Kg * (data - Xlast); // Xnow = Xpre+Kg*(Z(k)-H*Xpre)(3)
    P = (1 - Kg) * PP;                  //Pnow=(I-Kg)*Ppre(5) 由于Pnow不会再使用，所以更新Plast=Pnow
    Xlast = Xnow;

    return Xnow;
}
