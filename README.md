# X.K.MusicESP
ESP32 + WS2812 + Music

## 1. Summary
使用ESP32内置的ADC采集电脑输出的音频信号，通过RGB灯带展现出来

最初计划采用外置ADC将音频转为I2S格式传给ESP32，但是遇到了许多问题。
于是转为使用ESP32内置的ADC，虽然也踩了许多坑，但也逐一解决了

## 2. Wiring
+ 两条100颗灯珠的WS2812灯带串联，数据线接入ESP32的GPIO12
+ 3.5mm的AUX线，左声道接入ESP32的ADC1_CHANNEL_3，地线接EPS32的GND


## 2. Reference
大部分代码来自于：

[https://github.com/zhujisheng/audio-reactive-led-strip](https://github.com/zhujisheng/audio-reactive-led-strip)

其他参考/引用：

[https://esphome.io/](https://esphome.io/)<br/>
[https://esp32.com/viewtopic.php?f=12&t=10749](https://esp32.com/viewtopic.php?f=12&t=10749)<br/>
[https://www.reddit.com/r/esp32/comments/g48lzs/esp_32_i2s_and_cs4344_dac/](https://www.reddit.com/r/esp32/comments/g48lzs/esp_32_i2s_and_cs4344_dac/)<br/>
[https://www.youtube.com/watch?v=m8LwPNXqK9o&t=124s](https://www.youtube.com/watch?v=m8LwPNXqK9o&t=124s)<br/>
[https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html)

## 3. Lessions

踩坑总结：

1. 在通过I2S驱动使用内置ADC时，我读取了长度为512的I2S数据，但是只有前面一半数据可用，不知道为什么
1. 如果音频线和ESP32没有接地，采集出来的信号中会有杂波
1. 有的时候音量过大会导致ESP32重启，是因为电压超过了内置ADC的量程了吗？
1. 接地后，当电脑在没有播放音乐的时候，会出现随机的尖刺，不知道为什么。我使用卡尔曼滤波器 + 星空曼滤波器解决了
1. 我买的四段式AUX插座，如果把三段式音频插头插到底，会出现左右声道短路的情况，不知道其他AUX插座是不是这样

## 4. Demonstration
[https://www.bilibili.com/video/BV1vi4y1x7Pa/](https://www.bilibili.com/video/BV1vi4y1x7Pa/)