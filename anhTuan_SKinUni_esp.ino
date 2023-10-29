

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <SoftwareSerial.h>

#define SERVER "skinuni.tech"  //need to host this first
#define PORT 80
#define NUMOFVAR 7

ESP8266WiFiMulti wifiMulti;
SocketIOclient socketIO;
WiFiManager wifiManager;
SoftwareSerial mySerial(D6, D5);  // Tạo giao diện UART ảo với chân RX là D5, chân TX là D6

uint32_t last_check = 0;
uint32_t lastRead = 0;
uint32_t lastSendData = 0;

String data[NUMOFVAR];

void socketIOEvent(socketIOmessageType_t type, uint8_t *payload, size_t length) {
  String text1 = (char *)payload;
  switch (type) {
    case sIOtype_DISCONNECT:
      Serial.printf("[IOc] Disconnected!\n");
      break;
    case sIOtype_CONNECT:
      Serial.printf("[IOc] Connected to url: %s\n", payload);
      // join default namespace (no auto join in Socket.IO V3)
      socketIO.send(sIOtype_CONNECT, "/");
      break;
    case sIOtype_EVENT:
      break;
    case sIOtype_ACK:
      Serial.printf("[IOc] get ack: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_ERROR:
      Serial.printf("[IOc] get error: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_EVENT:
      Serial.printf("[IOc] get binary: %u\n", length);
      hexdump(payload, length);
      break;
    case sIOtype_BINARY_ACK:
      Serial.printf("[IOc] get binary ack: %u\n", length);
      hexdump(payload, length);
      break;
  }
}

void setup() {
  Serial.begin(9600);    // Baud rate của UART với Serial Monitor
  mySerial.begin(9600);  // Baud rate của UART
  Serial.println();
  Serial.println("wifi connecting");
  wifiManager.autoConnect("SKINUNI", "88888888");
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  //ket noi server
  socketIO.begin(SERVER, PORT, "/socket.io/?EIO=4");  //socket.io/?EIO=4
  socketIO.onEvent(socketIOEvent);
}



void loop() {
  socketIO.loop();

  if (mySerial.available()) {                              // Kiểm tra xem có dữ liệu đang được gửi đến không
    String receivedData = mySerial.readStringUntil('\n');  // Đọc chuỗi dữ liệu cho đến khi gặp ký tự '\n'

    // Tách dữ liệu bằng dấu phẩy

    int startIndex = 0;
    int commaIndex;

    for (int i = 0; i < NUMOFVAR; i++) {
      commaIndex = receivedData.indexOf(',', startIndex);
      if (commaIndex != -1) {
        data[i] = receivedData.substring(startIndex, commaIndex);
        startIndex = commaIndex + 1;
      } else {
        data[i] = receivedData.substring(startIndex);
        break;
      }
    }

#ifdef DEBUG
    for (int i = 0; i < NUMOFVAR; i++) {
      Serial.print("Data " + String(i + 1) + ": ");
      Serial.println(data[i]);
    }
#endif
  }
  //gui len server
  if (millis() - lastSendData > 5000) {
    sendData();
    Serial.println("upload done ");
    socketIO.loop();
    lastSendData = millis();
  }
}
//}

void sendData() {
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  array.add("/esp/data");
  JsonObject param1 = array.createNestedObject();

  param1["temp"] = data[0];
  param1["humi"] = data[1];
  param1["uv"] = data[2];
  param1["uvi"] = data[3];
  param1["co"] = data[4];
  param1["gas"] = data[5];
  param1["dust"] = data[6];
  param1["client"] = "esp";


  String output;
  serializeJson(doc, output);
  socketIO.sendEVENT(output);
  //Serial.println(output);
  delay(20);
}
