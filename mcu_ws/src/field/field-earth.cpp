#include <WiFi.h>
#include "earth.h"
#include <esp_now.h>
#include <field-message.h>

// IR receiver pin
const uint8_t irPin = 15;

// Earth element configuration
Field::EarthSetup setupEarth = Field::EarthSetup("Earth", irPin);
Field::EarthDriver Earth1 = Field::EarthDriver(setupEarth);

// ESP-NOW state
uint8_t sequenceNum = 0;
Field::ElementStatus earthStatus = Field::ElementStatus::NOT_ONLINE;
bool allReceived = false;

unsigned long lastStatusUpdate = 0;
unsigned long lastResultUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 1000;
const unsigned long RESULT_UPDATE_INTERVAL = 2000;

// Received message buffer
Field::FieldMessage receivedMessage;
volatile bool hasNewMessage = false;

// ESP-NOW callbacks
void onDataRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len <= sizeof(Field::FieldMessage)) {
    memcpy(&receivedMessage, data, len);
    hasNewMessage = true;
  }
}

void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  // Optional logging
}

void initEspNow() {
  // Ensure clean WiFi state
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Disable WiFi sleep for reliable ESP-NOW
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Add broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, Field::BROADCAST_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Print MAC
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("Earth MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1],
                mac[2], mac[3], mac[4], mac[5]);

  earthStatus = Field::ElementStatus::READY;
}

void sendStatus() {
  Field::StatusUpdateMessage msg;
  msg.header.type = Field::MessageType::STATUS_UPDATE;
  msg.header.senderId = Field::ElementId::EARTH;
  msg.header.sequenceNum = sequenceNum++;
  msg.status = earthStatus;

  esp_now_send(Field::BROADCAST_MAC, (uint8_t*)&msg, sizeof(msg));
}

void sendColorResults() {
  Field::ColorResultMessage msg;
  msg.header.type = Field::MessageType::COLOR_RESULT;
  msg.header.senderId = Field::ElementId::EARTH;
  msg.header.sequenceNum = sequenceNum++;
  msg.correctCount = Earth1.getCorrectCount();
  msg.wrongCount = Earth1.getWrongColorCount();

  for (int i = 0; i < 4; i++) {
    msg.results[i].antennaId = static_cast<Field::ElementId>(i + 1);
    msg.results[i].expectedColor = Earth1.getExpectedColor(i);
    msg.results[i].receivedColor =
        Field::EarthDriver::antennaToFieldColor(Earth1.getAntennaColor(i));
    msg.results[i].correct = Earth1.isColorCorrect(i);
  }

  esp_now_send(Field::BROADCAST_MAC, (uint8_t*)&msg, sizeof(msg));
}

void processMessages() {
  if (!hasNewMessage) return;
  hasNewMessage = false;

  switch (receivedMessage.header.type) {
    case Field::MessageType::COMMAND: {
      const Field::CommandMessage& cmd = receivedMessage.command;

      // Check if for us or broadcast
      if (cmd.targetId != Field::ElementId::EARTH &&
          static_cast<uint8_t>(cmd.targetId) != 0xFF) {
        return;
      }

      switch (cmd.command) {
        case Field::FieldCommand::RESET:
          Serial.println("RESET command received");
          ESP.restart();
          break;

        case Field::FieldCommand::START:
          Serial.println("START command received");
          Earth1.reset();
          allReceived = false;
          earthStatus = Field::ElementStatus::NOT_ACTIVATED;
          sendStatus();
          break;

        case Field::FieldCommand::PING:
          sendStatus();
          break;

        default:
          break;
      }
      break;
    }

    case Field::MessageType::COLOR_REPORT: {
      // Receive expected color from antenna element
      const Field::ColorReportMessage& report = receivedMessage.colorReport;
      uint8_t antennaIndex =
          static_cast<uint8_t>(report.header.senderId) - 1;  // 0-3

      if (antennaIndex < 4) {
        Earth1.setExpectedColor(antennaIndex, report.color);
      }
      break;
    }

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Field-Earth starting...");

  Earth1.init();
  initEspNow();

  Serial.println("Ready - waiting for antenna signals");
}

void loop() {
  unsigned long now = millis();

  // Process incoming ESP-NOW messages
  processMessages();

  // Update IR receiver
  Earth1.update();

  // Check if all antennas received
  if (Earth1.getStatus() && !allReceived) {
    Serial.println("ALL ANTENNAS RECEIVED!");
    Serial.println(Earth1.getInfo());
    allReceived = true;
    earthStatus = Field::ElementStatus::ACTIVATED;
    sendStatus();
    sendColorResults();
  }

  // Send periodic status updates
  if (now - lastStatusUpdate > STATUS_UPDATE_INTERVAL) {
    lastStatusUpdate = now;
    if (!allReceived && earthStatus != Field::ElementStatus::ACTIVATED) {
      earthStatus = Field::ElementStatus::NOT_ACTIVATED;
    }
    sendStatus();
  }

  // Send periodic color results (for controller display)
  if (now - lastResultUpdate > RESULT_UPDATE_INTERVAL) {
    lastResultUpdate = now;
    if (Earth1.hasReceivedTransmission()) {
      sendColorResults();
    }
  }
}
