#include <NimBLEDevice.h>

#define RXD2 9
#define TXD2 10
#define LED_PIN 8

HardwareSerial& arduinoSerial = Serial1;

// UUIDs for Nordic UART Service (NUS)
#define UART_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* txCharacteristic;
bool deviceConnected = false;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    Serial.println("Client connected");
    deviceConnected = true;
    digitalWrite(LED_PIN, HIGH);  // Turn on LED when connected
  }
  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    Serial.println("Client disconnected");
    deviceConnected = false;
    digitalWrite(LED_PIN, LOW);   // Turn off LED when disconnected
    pServer->startAdvertising();  // Keep advertising
  }
};

class RxCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar, NimBLEConnInfo& connInfo) override {
    std::string rxVal = pChar->getValue();
    Serial.print("BLE RX: ");
    Serial.println(rxVal.c_str());
    arduinoSerial.print(rxVal.c_str());  // Forward to Arduino
  }
};

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);  // Make sure LED starts in off

  NimBLEDevice::init("BLE-RobotBridge");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // Max power

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* uartService = pServer->createService(UART_SERVICE_UUID);

  NimBLECharacteristic* rxCharacteristic = uartService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxCharacteristic->setCallbacks(new RxCallback());

  txCharacteristic = uartService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    NIMBLE_PROPERTY::NOTIFY
  );

  uartService->start();

  delay(100);

  NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(UART_SERVICE_UUID);
  pAdvertising->setMinInterval(32);  // 20ms
  pAdvertising->setMaxInterval(64);  // ~40ms
  pAdvertising->start();

  Serial.println("BLE UART bridge ready");
  arduinoSerial.println("TEST FROM ESP32");
}

void loop() {
  if (deviceConnected && arduinoSerial.available()) {
    String data = "";
    while (arduinoSerial.available()) {
      data += (char)arduinoSerial.read();
      delay(1);  // Give time for buffer to fill
    }
    if (data.length()) {
      txCharacteristic->setValue(data.c_str());
      txCharacteristic->notify();
      Serial.print("UART -> BLE: ");
      Serial.println(data);
    }
  }

  delay(1);  // tiny delay to let BLE and serial breathe
}