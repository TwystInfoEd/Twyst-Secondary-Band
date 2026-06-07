#ifndef TWYST_BACKEND_H
#define TWYST_BACKEND_H

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFi.h>

struct TwystBackendFrame {
    float acc_x {0.0f};
    float acc_y {0.0f};
    float acc_z {0.0f};
    float gyro_x {0.0f};
    float gyro_y {0.0f};
    float gyro_z {0.0f};
    float roll {0.0f};
    float pitch {0.0f};
    float yaw {0.0f};
    unsigned long timestamp {0};
    bool hasTimestamp {false};
};

class TwystBackendClient {
public:
    TwystBackendClient();

    void setBaseUrl(const String& baseUrl);
    const String& getBaseUrl() const;

    bool connectWiFi(const char* ssid, const char* password, unsigned long timeoutMs = 15000);
    bool isWiFiConnected() const;

    bool startRecord(const String& motionName, String* responseBody = nullptr);
    bool sendRecordFrame(const TwystBackendFrame& frame, String* responseBody = nullptr);
    bool stopRecord(int bezierOrder = 8, String* responseBody = nullptr);

    bool startCompare(const String& referenceName, String* responseBody = nullptr);
    bool sendCompareFrame(const TwystBackendFrame& frame, String* responseBody = nullptr);
    bool stopCompare(String* responseBody = nullptr);

    // Asynchronous frame sending (optional). When started, frames will be queued
    // and sent from a background FreeRTOS task. Enqueueing returns immediately.
    bool beginAsyncFrameSender(size_t queueLength = 16, unsigned int taskPriority = 1);
    void stopAsyncFrameSender();
    bool enqueueFrame(const TwystBackendFrame& frame);

    bool listMotions(String* responseBody = nullptr);
    bool getMotion(const String& name, String* responseBody = nullptr);
    bool deleteMotion(const String& name, String* responseBody = nullptr);

private:
    String baseUrl_;

    // Opaque handles to avoid exposing FreeRTOS headers in the public header.
    void* frameQueue_{nullptr};
    void* frameTaskHandle_{nullptr};

    String makeUrl(const String& path) const;
    static String buildFrameJson(const TwystBackendFrame& frame);
    static String buildMotionJson(const char* key, const String& value);
    bool requestJson(const char* method, const String& path, const String& body, String* responseBody);

    static void frameWorkerTask(void* pvParameters);
};

#endif
