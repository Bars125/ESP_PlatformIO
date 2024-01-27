// Multipurpose variables
#define THRESHOLD 40                                  // Greater the value, more the sensitivity on the wake-up Pin
#define customDelay(ms) vTaskDelay(pdMS_TO_TICKS(ms)) // Macros TICKS to ms

// Peripheral Gpio set-up
#define Button GPIO_NUM_27                            // record audio / wake up trigger
#define wifi_state_led GPIO_NUM_26                    // Wifi status LED
#define record_state_led GPIO_NUM_14                  // Record process 

// INMP 441 Connection ports
#define I2S_SD GPIO_NUM_32
#define I2S_WS GPIO_NUM_25
#define I2S_SCK GPIO_NUM_33

// Recording variables
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_SAMPLE_BITS 16
#define I2S_READ_LEN (I2S_SAMPLE_BITS * 1024)
#define RECORD_TIME 5       // Record N seconds
#define I2S_CHANNEL_NUM 1
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)

//libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <driver/i2s.h>
#include <SPIFFS.h>
#include <FS.h>
#include <Base64.h>
//#include <HTTPClient.h>
#include "network_param.h"

//global variables
File file;
String response = "";
const char* global_encoded;
const char filename[] = "/recording.wav";
const int headerSize = 44;
bool isWIFIConnected = false;
bool isRecorded = false;

//func prototypes
void SPIFFSInit();
void wavHeader(byte* header, int wavSize);
void listSPIFFS();
void print_wakeup_touchpad();
void start_deep_sleep();
void i2s_Init();
void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len);
void i2s_adc();
void uploadFileGoogle();
void connectToWiFi();
void callGoogleSpeechApi(String base64Audio);
void encodeBase64(File file);

//main function
void setup() {

  // Setting up the Serial Monitor
  Serial.begin(115200);
  customDelay(500);
  Serial.println("Serial Monitor is ON");

  // Led indication setup
  pinMode(wifi_state_led, OUTPUT);
  pinMode(record_state_led, OUTPUT);
  pinMode(Button, INPUT_PULLUP);
  digitalWrite(wifi_state_led, 0);  // wifi status LED make sure it's 0
  digitalWrite(record_state_led, 0);

  // Wake up settings
  print_wakeup_touchpad();
  touchSleepWakeUpEnable(T3, THRESHOLD);  // Setup sleep, wakeup on Touch Pad 3 (GPIO15)

  //variables
  int Count = 10;  // seconds till Esp fall asleep (no button click)

  // ===== Programm  starts Here =====

  SPIFFSInit();  // initialize the SPI Flash File System
  i2s_Init();    // initialize the I2S interface

  // main cycle: recoding after pressing the button; quit when wifi conn-ion is lost;
  while (true) {

    if (WiFi.status() != WL_CONNECTED) {
      connectToWiFi();
    }

    do {
      if (digitalRead(Button) == LOW) {  // when the button was clicked
        Serial.println("Record Button's Pressed!");
        digitalWrite(record_state_led, 1);
        i2s_adc();
        break;
      } else {
        Serial.println("Left to Sleep: " + String(Count--));
        customDelay(1000);
      }
    } while (Count != 0);

    if (isWIFIConnected && isRecorded) {
      digitalWrite(record_state_led, 0);
      uploadFileGoogle();
      break;
    } else {
      Serial.println("CHAO-1 (when there was no record)!");
      start_deep_sleep();
    }
  }
  Serial.println("CHAO-2 (main cycle, after recording)!");
  start_deep_sleep();
}

void print_wakeup_touchpad() {
  touch_pad_t touchPin = esp_sleep_get_touchpad_wakeup_status();
  if (touchPin == 3) {
    Serial.println("Wakeup caused by touchpad on GPIO 15");
  } else {
    Serial.println("Wakeup not by touchpad");
  }
}

void start_deep_sleep() {
  Serial.println("");
  Serial.println("Going to sleep now");
  customDelay(500);
  esp_deep_sleep_start();
}

void i2s_Init() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S, // static_cast<i2s_comm_format_t>(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB)
    .intr_alloc_flags = 0,
    .dma_buf_count = 64,
    .dma_buf_len = 1024,
    .use_apll = 1
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len) {
  uint32_t j = 0;
  uint32_t dac_value = 0;
  for (int i = 0; i < len; i += 2) {
    dac_value = ((((uint16_t)(s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
    d_buff[j++] = 0;
    d_buff[j++] = dac_value * 256 / 2048;
  }
}

/*void i2s_adc_data_scale(uint8_t* d_buff, uint8_t* s_buff, uint32_t len) {
  uint32_t j = 0;
  uint32_t dac_value = 0;

  for (int i = 0; i < len; i += 2) {
    // Extracting 12 bits from 16-bit sample
    dac_value = ((uint16_t)(s_buff[i + 1]) << 8 | (s_buff[i]));

    // Scale the 12-bit DAC value to 8-bit range
    d_buff[j++] = 0;  // Assuming you want to keep the most significant byte as 0
    d_buff[j++] = dac_value >> 4;  // Scaling 12-bit to 8-bit (right shift by 4)
  }
}*/

void i2s_adc() {

  int i2s_read_len = I2S_READ_LEN;
  int flash_wr_size = 0;
  size_t bytes_read;

  char* i2s_read_buff = (char*)calloc(i2s_read_len, sizeof(char));
  uint8_t* flash_write_buff = (uint8_t*)calloc(i2s_read_len, sizeof(char));

  i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
  //i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);

  Serial.println(" *** Recording Start *** ");

  while (flash_wr_size < FLASH_RECORD_SIZE) {
    //read data from I2S bus, in this case, from ADC.
    i2s_read(I2S_PORT, (void*)i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
    //save original data from I2S(ADC) into flash.
    i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
    file.write((const byte*)flash_write_buff, i2s_read_len);
    flash_wr_size += i2s_read_len;
    Serial.printf("Sound recording %u%%\n", (flash_wr_size * 100 / FLASH_RECORD_SIZE));
    //Serial.printf("Never Used Stack Size: %u\n", uxTaskGetStackHighWaterMark(NULL));
  }

  file.close();
  isRecorded = true;
  
  free(i2s_read_buff);
  i2s_read_buff = NULL;
  free(flash_write_buff);
  flash_write_buff = NULL;

  listSPIFFS();
}

void uploadFileGoogle() {
  file = SPIFFS.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Failed to open the file.");
    return;
  }

  // Read file content
  encodeBase64(file);
  const char* base64Audio = global_encoded;
  file.close();

  //TEST
  //Serial.println(base64Audio);

  //Check if base64 encoding was successful
  if (base64Audio == nullptr || base64Audio[0] == '\0') { //base64Audio.isEmpty()
    Serial.println("Failed to encode audio to base64");
    return;
  }

  // Call Google Cloud Speech-to-Text API
  callGoogleSpeechApi(base64Audio);
  //String transcription = callGoogleSpeechApi(base64Audio);

  // Check if API call was successful
  if (response.isEmpty()) {
    Serial.println("Failed to get transcription from Google Cloud API");
    return;
  }
  Serial.println("Transcription: " + response);

  /*Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.124:8888/uploadAudio"); //http://192.168.0.185:8888/uploadAudio
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    String response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    Serial.println("====================      End      ====================");
  }else{
    Serial.println("Error");
  }
  file.close();
  client.end();*/
}

/*void encodeBase64(File file) {
  const int bufferSize = 512;
  byte buffer[bufferSize];
  size_t bytesRead;

  while (file.available()) {
    bytesRead = file.read(buffer, bufferSize);
    if (bytesRead > 0) {
      const char* chunk = base64::encode(buffer, bytesRead);
      chunk.replace("\n", "");  // delete last "\n"
      encoded += chunk;         // concatenate the chunks
    }
  }
}*/

void encodeBase64(File file) {
  const int bufferSize = 512;
  byte buffer[bufferSize];
  size_t bytesRead;

  // Use dynamic memory allocation for encoded
  const size_t maxEncodedSize = bufferSize * 2; // Adjust the size as needed
  char* encoded = (char*)malloc(maxEncodedSize);
  
  if (encoded == nullptr) {
    Serial.println("Memory allocation failed");
    return;
  }

  encoded[0] = '\0';  // Initialize as an empty string

  while (file.available()) {
    bytesRead = file.read(buffer, bufferSize);
    if (bytesRead > 0) {
      String chunkStr = base64::encode(buffer, bytesRead);
      const char* chunk = chunkStr.c_str();  // Convert String to const char*
      
      // Remove '\n' from the chunk
      size_t chunkLen = strlen(chunk);
      for (size_t i = 0; i < chunkLen; ++i) {
        if (chunk[i] != '\n') {
          strncat(encoded, &chunk[i], 1);
        }
      }
    }
  }
  global_encoded = encoded;
  // Free the allocated memory
  free(encoded);
}

void callGoogleSpeechApi(String base64Audio) {
  // Ensure WiFi is connected
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected");
    return;
  }

  // DNS resolution
  IPAddress ip;
  if (!WiFi.hostByName(server, ip)) {
    Serial.println("DNS resolution failed");
    return;
  }
  Serial.println("Host-IP address: " + ip.toString());

  // SSL/TLS setup
  WiFiClientSecure client;
  client.setCACert(root_cert);

  // Attempt to connect to Google Cloud API
  Serial.println("Attempting to connect to Google Cloud API");
  int connectAttempts = 0;
  while (!client.connect(server, 443)) {

    Serial.println("Couldn't connect to 443 port... ");
    customDelay(1000);
    connectAttempts++;

    if (connectAttempts > 5) {
      Serial.println("Failed to connect after multiple attempts");
      return;
    }
  }
  Serial.println("Connected to Google Cloud API");

  // Prepare the request
  String requestBody = "{\"config\": {\"encoding\":\"LINEAR16\",\"sampleRateHertz\":16000,\"languageCode\":\"en-US\"},\"audio\":{\"content\":\"" + base64Audio + "\"}}";
  String contentLength = String(requestBody.length());
  String request = "POST " + String(googleCloudEndpoint) + "?key=" + String(googleCloudApiKey) + " HTTP/1.1\r\n" + "Host: " + server + "\r\n" + "Content-Type: application/json\r\n" + "Content-Length: " + contentLength + "\r\n\r\n" + requestBody;

  // Send the request
  Serial.println("Sending request:");
  Serial.println(request);
  client.print(request);

  // Receive and process the response
  unsigned long timeout = millis();
  while (client.connected()) {
    if (client.available()) {
      char temp = client.read();
      response += temp;
      timeout = millis();  // Reset the timeout counter since we received data
    } else if (millis() - timeout > 10000) {
      Serial.println("Timeout occurred while waiting for response");
      client.stop();
      return;
    }
  }

  // Print response information
  if (response.length() == 0) {
    Serial.println("Failed to get response from Google Cloud API");
  } else {
    Serial.println("Response length: " + String(response.length()));
    Serial.println("Response content:");
    Serial.println(response);
  }

  // Close the connection
  client.stop();
}

void connectToWiFi() {
  isWIFIConnected = false;
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  for (byte i; i < 10; i++) {
    Serial.println("Connecting to WiFi...");
    customDelay(1000);
    if (WiFi.status() == WL_CONNECTED) {
      isWIFIConnected = true;
      digitalWrite(wifi_state_led, 1);
      Serial.print("Connected to WiFi network: ");
      Serial.println(WiFi.SSID());
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      break;
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("CHAO-3 (Wifi-func)!");
    start_deep_sleep();
  }
}

void SPIFFSInit() {
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialisation failed!");
  }

  SPIFFS.remove(filename);
  file = SPIFFS.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("File is not available!");
  }

  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE);

  file.write(header, headerSize);
  listSPIFFS();
}

void wavHeader(byte* header, int wavSize) {
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSize = wavSize + headerSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;
  header[21] = 0x00;
  header[22] = 0x01;
  header[23] = 0x00;
  header[24] = 0x80;
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;
  header[29] = 0x7D;
  header[30] = 0x01;
  header[31] = 0x00;
  header[32] = 0x02;
  header[33] = 0x00;
  header[34] = 0x10;
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
}

void listSPIFFS() {
  Serial.println(F("\r\nListing SPIFFS files:"));
  //static const char line[] PROGMEM = "";

  //Serial.println(FPSTR(line));
  Serial.println(F("File name Size"));
  //Serial.println(FPSTR(line));

  fs::File root = SPIFFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  fs::File file = root.openNextFile();
  while (file) {

    if (file.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file.name();
      Serial.print(fileName);
    } else {
      String fileName = file.name();
      Serial.print("  " + fileName);
      // File path can be 31 characters maximum in SPIFFS
      int spaces = 33 - fileName.length();  // Tabulate nicely
      if (spaces < 1)
        spaces = 1;
      while (spaces--) Serial.print(" ");
      String fileSize = (String)file.size();
      spaces = 10 - fileSize.length();  // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      Serial.println(fileSize + " bytes");
    }

    file = root.openNextFile();
  }

  //Serial.println(FPSTR(line));
  Serial.println();
  customDelay(100);
}

void loop() {
}