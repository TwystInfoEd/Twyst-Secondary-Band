#include "ble_band.h"

namespace {

const char *BLE_SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char *BLE_RX_UUID      = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const char *BLE_TX_UUID      = "beb5483f-36e1-4688-b7f5-ea07361b26a8";

static NimBLECharacteristic *s_txCharacteristic = nullptr;
static bool s_connected = false;
static unsigned long s_lastHeartbeatMs = 0;

void sendResponse(const String &text)
{
    if (s_txCharacteristic == nullptr || !s_connected) {
        Serial.printf("[BLE] TX skipped (connected=%d, char=%p)\n", s_connected ? 1 : 0, (void *)s_txCharacteristic);
        return;
    }

    s_txCharacteristic->setValue(text.c_str());
    s_txCharacteristic->notify(reinterpret_cast<const uint8_t *>(text.c_str()), text.length(), true);
    Serial.printf("[BLE] TX: %s\n", text.c_str());
}

class ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer *server) override
    {
        (void)server;
        s_connected = true;
        Serial.println("[BLE] Main band connected");
        Serial.println("[BLE] Ready for commands: ping, status, hello");
    }

    void onDisconnect(NimBLEServer *server) override
    {
        (void)server;
        s_connected = false;
        Serial.println("[BLE] Main band disconnected");
        NimBLEDevice::startAdvertising();
        Serial.println("[BLE] Advertising restarted");
    }
};

class RxCallbacks : public NimBLECharacteristicCallbacks
{
    void onWrite(NimBLECharacteristic *characteristic) override
    {
        String payload = characteristic->getValue().c_str();
        payload.trim();

        Serial.printf("[BLE] RX: %s\n", payload.c_str());

        if (payload.equalsIgnoreCase("ping")) {
            sendResponse("pong");
            return;
        }

        if (payload.equalsIgnoreCase("hello")) {
            sendResponse("hello_ack");
            return;
        }

        if (payload.equalsIgnoreCase("status")) {
            String status = "status connected=" + String(s_connected ? 1 : 0) +
                            " uptime_ms=" + String(millis());
            sendResponse(status);
            return;
        }

        if (payload.length() == 0) {
            Serial.println("[BLE] Empty payload received");
            sendResponse("error empty_payload");
            return;
        }

        Serial.println("[BLE] Unknown command received");
        sendResponse("error unknown_command");
    }
};

}

void initBLE()
{
    Serial.println("[BLE] Initializing BLE stack");

    NimBLEDevice::init("Twyst-Secondary-Band");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    NimBLEServer *server = NimBLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    NimBLEService *service = server->createService(BLE_SERVICE_UUID);

    NimBLECharacteristic *rxCharacteristic = service->createCharacteristic(
        BLE_RX_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    rxCharacteristic->setCallbacks(new RxCallbacks());

    s_txCharacteristic = service->createCharacteristic(
        BLE_TX_UUID,
        NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::READ
    );
    const char *readyText = "Twyst Secondary Band ready";
    s_txCharacteristic->setValue(reinterpret_cast<const uint8_t *>(readyText), strlen(readyText));

    service->start();

    NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
    advertising->addServiceUUID(BLE_SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->start();

    Serial.println("[BLE] Advertising as Twyst-Secondary-Band");
    Serial.println("[BLE] Send 'ping' or 'status' from the main band to test the link");
}

void bleLoop()
{
    if (!s_connected) {
        return;
    }

    unsigned long now = millis();
    if (now - s_lastHeartbeatMs >= 5000) {
        s_lastHeartbeatMs = now;
        sendResponse("heartbeat uptime_ms=" + String(now));
    }
}

bool bleIsConnected()
{
    return s_connected;
}

bool bleSendText(const String &text)
{
    if (!s_connected || s_txCharacteristic == nullptr) {
        Serial.printf("[BLE] Not sent, connected=%d\n", s_connected ? 1 : 0);
        return false;
    }

    sendResponse(text);
    return true;
}

bool bleSendFrame(const String &frame)
{
    return bleSendText("sec2 " + frame);
}