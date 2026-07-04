#include "../include/ble_band.h"

#include <NimBLEDevice.h>

namespace {

const char *kTargetName    = "Twyst-Main-Band";
const char *kServiceUuid   = "4fafc201-1fb5-459e-8fcc-c5c9c331914b";
const char *kMainRxUuid    = "beb5483e-36e1-4688-b7f5-ea07361b26a8";
const char *kMainTxUuid    = "beb5483f-36e1-4688-b7f5-ea07361b26a8";

enum class SecBleState {
    Idle,
    Scanning,
    Connecting,
    Ready,
    Lost,
};

static SecBleState s_state = SecBleState::Idle;
static NimBLEAdvertisedDevice *s_targetDevice = nullptr;
static NimBLEClient *s_client = nullptr;
static NimBLERemoteCharacteristic *s_mainRx = nullptr;
static NimBLERemoteCharacteristic *s_mainTx = nullptr;
static bool s_notificationsEnabled = false;
static bool s_disconnected = false;
static int s_disconnectReason = 0;
static unsigned long s_lostSinceMs = 0;
static unsigned long s_readySinceMs = 0;
static unsigned long s_lastTestCommandMs = 0;

void clearTarget() {
    if (s_targetDevice != nullptr) {
        delete s_targetDevice;
        s_targetDevice = nullptr;
    }
}

void clearSessionFlags() {
    s_mainRx = nullptr;
    s_mainTx = nullptr;
    s_notificationsEnabled = false;
}

void setLostState(int reason) {
    s_disconnectReason = reason;
    s_disconnected = true;
    clearSessionFlags();
    clearTarget();
    s_lostSinceMs = millis();
    s_state = SecBleState::Lost;
}

void onMainNotify(NimBLERemoteCharacteristic *characteristic, uint8_t *data, size_t length, bool isNotify) {
    (void)characteristic;
    (void)isNotify;

    String payload;
    payload.reserve(length + 1);
    for (size_t i = 0; i < length; ++i) {
        const char character = static_cast<char>(data[i]);
        if (character == '\r' || character == '\n' || (character >= 32 && character <= 126)) {
            payload += character;
        } else {
            payload += '?';
        }
    }
    payload.trim();

    Serial.printf("[SEC-BLE] RX raw len=%u: ", static_cast<unsigned int>(length));
    for (size_t i = 0; i < length; ++i) {
        Serial.printf("%02X ", data[i]);
    }
    Serial.println();

    if (payload.startsWith("heartbeat")) {
        Serial.printf("[SEC-BLE] RX <- %s\n", payload.c_str());
        return;
    }

    if (payload.startsWith("imu ")) {
        Serial.printf("[SEC-BLE] RX <- %s\n", payload.c_str());
        return;
    }

    Serial.printf("[SEC-BLE] RX <- %s\n", payload.c_str());
}

bool sendCommand(const char *command) {
    if (s_mainRx == nullptr) {
        return false;
    }

    Serial.printf("[SEC-BLE] TX -> %s\n", command);
    const bool ok = s_mainRx->writeValue(reinterpret_cast<const uint8_t *>(command), strlen(command), true);
    if (!ok) {
        Serial.printf("[SEC-BLE] TX failed for %s\n", command);
    }
    return ok;
}

class ClientCallbacks : public NimBLEClientCallbacks {
    void onDisconnect(NimBLEClient *client) override {
        (void)client;
        Serial.println("[SEC-BLE] Disconnected, rescanning");
        setLostState(-1);
    }
};

class ScanCallbacks : public NimBLEAdvertisedDeviceCallbacks {
    void onResult(NimBLEAdvertisedDevice *advertisedDevice) override {
        if (s_targetDevice != nullptr) {
            return;
        }

        const bool nameMatch = advertisedDevice->haveName() &&
                               advertisedDevice->getName() == kTargetName;
        const bool serviceMatch = advertisedDevice->isAdvertisingService(NimBLEUUID(kServiceUuid));

        if (!nameMatch && !serviceMatch) {
            return;
        }

        s_targetDevice = new NimBLEAdvertisedDevice(*advertisedDevice);
        NimBLEDevice::getScan()->stop();
        Serial.println("[SEC-BLE] Found main band, connecting...");
    }
};

static ScanCallbacks s_scanCallbacks;

bool discoverAndPrepare() {
    NimBLERemoteService *service = s_client->getService(kServiceUuid);
    if (service == nullptr) {
        Serial.println("[SEC-BLE] Service discovery failed");
        return false;
    }

    s_mainRx = service->getCharacteristic(kMainRxUuid);
    s_mainTx = service->getCharacteristic(kMainTxUuid);

    if (s_mainRx == nullptr || s_mainTx == nullptr) {
        Serial.println("[SEC-BLE] Characteristic discovery failed");
        return false;
    }

    if (!s_mainRx->canWrite()) {
        Serial.println("[SEC-BLE] Main RX characteristic is not writable");
        return false;
    }

    Serial.println("[SEC-BLE] Service/char discovery OK");

    if (!s_mainTx->canNotify()) {
        Serial.println("[SEC-BLE] Main TX characteristic does not support notifications");
        return false;
    }

    if (!s_mainTx->subscribe(true, onMainNotify)) {
        Serial.println("[SEC-BLE] Failed to enable notifications");
        return false;
    }

    s_notificationsEnabled = true;
    Serial.println("[SEC-BLE] Notifications enabled");
    return true;
}

bool connectToMain() {
    if (s_targetDevice == nullptr) {
        return false;
    }

    if (s_client == nullptr) {
        s_client = NimBLEDevice::createClient();
        s_client->setClientCallbacks(new ClientCallbacks(), false);
        s_client->setConnectionParams(12, 12, 0, 51);
        s_client->setConnectTimeout(5);
    }

    clearSessionFlags();

    if (!s_client->connect(s_targetDevice, false)) {
        Serial.println("[SEC-BLE] Connection attempt failed");
        return false;
    }

    Serial.println("[SEC-BLE] Connected");

    if (!discoverAndPrepare()) {
        s_client->disconnect();
        return false;
    }

    if (!sendCommand("hello")) {
        return false;
    }

    sendCommand("ping");
    sendCommand("status");
    s_readySinceMs = millis();
    s_lastTestCommandMs = s_readySinceMs;

    return true;
}

void startScan() {
    NimBLEScan *scan = NimBLEDevice::getScan();
    clearTarget();
    scan->stop();
    scan->setAdvertisedDeviceCallbacks(&s_scanCallbacks, false);
    scan->setInterval(45);
    scan->setWindow(15);
    scan->setActiveScan(true);
    scan->start(0, nullptr, false);
    Serial.println("[SEC-BLE] Scanning...");
}

} // namespace

void initBleClient() {
    Serial.println("[SEC-BLE] Initializing BLE client");
    NimBLEDevice::init("Twyst-Secondary-Band");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    s_state = SecBleState::Idle;
}

void bleClientLoop() {
    switch (s_state) {
        case SecBleState::Idle:
            startScan();
            s_state = SecBleState::Scanning;
            break;

        case SecBleState::Scanning:
            if (s_targetDevice != nullptr) {
                s_state = SecBleState::Connecting;
            }
            break;

        case SecBleState::Connecting:
            if (connectToMain()) {
                clearTarget();
                s_state = SecBleState::Ready;
                s_disconnected = false;
                s_disconnectReason = 0;
            } else {
                setLostState(-1);
            }
            break;

        case SecBleState::Ready:
            if (s_disconnected || s_client == nullptr || !s_client->isConnected()) {
                Serial.println("[SEC-BLE] Disconnected, rescanning");
                setLostState(s_disconnectReason);
                break;
            }

            if (s_notificationsEnabled) {
                const unsigned long now = millis();
                if (now - s_lastTestCommandMs >= 5000) {
                    s_lastTestCommandMs = now;
                    sendCommand("ping");
                }
            }
            break;

        case SecBleState::Lost:
            if (millis() - s_lostSinceMs >= 1000) {
                s_disconnected = false;
                s_disconnectReason = 0;
                s_readySinceMs = 0;
                s_lastTestCommandMs = 0;
                startScan();
                s_state = SecBleState::Scanning;
            }
            break;
    }
}

bool bleSendCommand(const String &command) {
    return sendCommand(command.c_str());
}
