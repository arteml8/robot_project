#include <NimBLEDevice.h>

#define RXD2 3
#define TXD2 2
HardwareSerial& arduinoSerial = Serial1;

// UUIDs for Nordic UART Service (NUS)
#define UART_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX   "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX   "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic* txCharacteristic;
bool deviceConnected = false;

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer) {
    Serial.println("Client connected");
    deviceConnected = true;
  }
  void onDisconnect(NimBLEServer* pServer) {
    Serial.println("Client disconnected");
    deviceConnected = false;
    pServer->startAdvertising();  // Keep advertising
  }
};

class RxCallback : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar) {
    std::string rxVal = pChar->getValue();
    Serial.print("BLE RX: ");
    Serial.println(rxVal.c_str());
    arduinoSerial.print(rxVal.c_str());  // Forward to Arduino
  }
};

void setup() {
  Serial.begin(115200);
  arduinoSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  NimBLEDevice::init("BLE-RobotBridge");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);  // Max power

  NimBLEServer* pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* uartService = pServer->createService(UART_SERVICE_UUID);

  NimBLECharacteristic* rxCharacteristic = uartService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    NIMBLE_PROPERTY::WRITE
  );
  rxCharacteristic->setCallbacks(new RxCallback());

  txCharacteristic = uartService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    NIMBLE_PROPERTY::NOTIFY
  );

  uartService->start();

  NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(UART_SERVICE_UUID);
  pAdvertising->setMinInterval(32);  // 20ms
  pAdvertising->setMaxInterval(64);  // ~40ms
  pAdvertising->start();

  Serial.println("BLE UART bridge ready");
}

void loop() {
  if (deviceConnected && arduinoSerial.available()) {
    char c = arduinoSerial.read();
    txCharacteristic->setValue(&c, 1);
    txCharacteristic->notify();
    Serial.print("UART -> BLE: ");
    Serial.println(c);
  }

  delay(1);  // tiny delay to let BLE and serial breathe
}