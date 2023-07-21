#include <Arduino.h>
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>


const char* ssid = "Keenetic-8995";
const char* password =  "S1e9r8g5ey";
const int record_time = 5;  // время записи в секундах

const int headerSize = 44;
const int waveDataSize = record_time * 88000;
const int numCommunicationData = 8000;
const int numPartWavData = numCommunicationData/4;
byte header[headerSize];
char communicationData[numCommunicationData];
char partWavData[numPartWavData];

File file;

//Создаем заголовок wav файла
void CreateWavHeader(byte* header, int waveDataSize){
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSizeMinus8 = waveDataSize + 44 - 8;
  header[4] = (byte)(fileSizeMinus8 & 0xFF);
  header[5] = (byte)((fileSizeMinus8 >> 8) & 0xFF);
  header[6] = (byte)((fileSizeMinus8 >> 16) & 0xFF);
  header[7] = (byte)((fileSizeMinus8 >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;  // linear PCM
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;  // linear PCM
  header[21] = 0x00;
  header[22] = 0x01;  // monoral
  header[23] = 0x00;
  header[24] = 0x44;  // sampling rate 44100
  header[25] = 0xAC;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x88;  // Byte/sec = 44100x2x1 = 88200
  header[29] = 0x58;
  header[30] = 0x01;
  header[31] = 0x00;
  header[32] = 0x02;  // 16bit monoral
  header[33] = 0x00;
  header[34] = 0x10;  // 16bit
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(waveDataSize & 0xFF);
  header[41] = (byte)((waveDataSize >> 8) & 0xFF);
  header[42] = (byte)((waveDataSize >> 16) & 0xFF);
  header[43] = (byte)((waveDataSize >> 24) & 0xFF);
}

//Инициализация интерфейса i2s
void I2S_Init(i2s_mode_t MODE, i2s_bits_per_sample_t BPS)
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = (44100),
      .bits_per_sample = BPS,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = 0,
      .dma_buf_count = 16,
      .dma_buf_len = 60
    };
    Serial.println("using ADC_builtin");
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    // GPIO36, VP
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_6);  
  
}

int I2S_Read(char *data, int numData)
{
  return i2s_read_bytes(I2S_NUM_0, (char *)data, numData, portMAX_DELAY);
}

void I2S_Write(char *data, int numData)
{
  i2s_write_bytes(I2S_NUM_0, (const char *)data, numData, portMAX_DELAY);
}

// Поднимаем Веб-сервер на порту 80
AsyncWebServer server(80);

void setup() {
  Serial.begin(115200); //Запускаем серийный порт
  WiFi.begin(ssid, password); //Подключаемся к WiFi
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println(WiFi.localIP());
  //Инициализируем файловую систему  
  SPIFFS.begin(true);

  // Отправляем команду серверу на начало записи
  server.on("/record_begin", HTTP_GET, [](AsyncWebServerRequest *request){
    file = SPIFFS.open("/audio.wav", FILE_WRITE); //Создаем файл для записи
    if(file){
      CreateWavHeader(header, waveDataSize);  //Создаем заголовок для wav
      Serial.println("Recording started");
      file.write(header, headerSize);//Записываем его в файл
  
  I2S_Init(I2S_MODE_ADC_BUILT_IN, I2S_BITS_PER_SAMPLE_32BIT); //Инициализируем интерфейс I2S
  //С помощьью встроенного ADC ESP32 получаем звуковой сигнал с микрофона, преобразовываем его в цифру и записываем в файл 
  for (int j = 0; j < waveDataSize/numPartWavData; ++j) {
    I2S_Read(communicationData, numCommunicationData);
    for (int i = 0; i < numCommunicationData/8; ++i) {
      partWavData[2*i] = communicationData[8*i + 2];
      partWavData[2*i + 1] = communicationData[8*i + 3];
    }
    file.write((const byte*)partWavData, numPartWavData);
  }
  file.close(); //Закрываем файл для записи
  Serial.println("finish");
      request->send(200, "text/plain", "Recording finished");
    } else { //Обрабатываем ошибку записи файла
      Serial.println("Failed to start recording");
      request->send(500, "text/plain", "Failed to start recording");
    }
  });

//Формируем главную страницу веб интерфейса
server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body>";
    html += "<h1>ESP32 Audio Recorder</h1>";
    html += "<button onclick=\"location.href='/record_begin'\" type='button'>Start Recording</button>";
    html += "<button onclick=\"location.href='/list'\" type='button'>Files list</button>";
    html += "<button onclick=\"location.href='/audio.wav'\" type='button'>Download WAV</button>";
    html += "</body></html>";
    request->send(200, "text/html", html);
});

//Формируем запрос серверу для скачивания созданного файла
server.on("/audio.wav", HTTP_GET, [](AsyncWebServerRequest *request){
  if(!SPIFFS.exists("/audio.wav")){
    return request->send(404, "text/plain", "File not found");
  }
  File file = SPIFFS.open("/audio.wav","r");
   if (file) {
        Serial.print("File size: ");
        Serial.println(file.size());
  request->send(file, "/audio.wav", "audio/wav", true);
   }else {
        request->send(500, "text/plain", "File does not exist");
      }

});

//Выводим содержимое файловой системы в веб интерфейс
server.on("/list", HTTP_GET, [](AsyncWebServerRequest *request){
    String html = "<html><body>";
    html += "<h1>SPIFFS Files:</h1>";
    html += "<ul>";
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while(file){
        html += "<li>";
        html += file.name();
        html += " (";
        html += file.size();
        html += " bytes)";
        html += "</li>";
        file = root.openNextFile();
    }
    html += "</ul>";
    html += "</body></html>";
    request->send(200, "text/html", html);
});


  // Запускаем сервер
  Serial.println("Server started");
  server.begin();
}

void loop() {
   
}