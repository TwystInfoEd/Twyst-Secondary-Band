#include "TwystBackend.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {
String escapeJsonString(const String& value) {
    String escaped;
    escaped.reserve(value.length() + 8);

    for (size_t index = 0; index < value.length(); ++index) {
        const char character = value[index];
        if (character == '"' || character == '\\') {
            escaped += '\\';
        }
        escaped += character;
    }

    return escaped;
}

String encodePathSegment(const String& value) {
    String encoded;
    encoded.reserve(value.length() * 3);

    for (size_t index = 0; index < value.length(); ++index) {
        const unsigned char character = static_cast<unsigned char>(value[index]);
        const bool unreserved =
            (character >= 'A' && character <= 'Z') ||
            (character >= 'a' && character <= 'z') ||
            (character >= '0' && character <= '9') ||
            character == '-' || character == '_' || character == '.' || character == '~';

        if (unreserved) {
            encoded += static_cast<char>(character);
        } else {
            const char hexDigits[] = "0123456789ABCDEF";
            encoded += '%';
            encoded += hexDigits[(character >> 4) & 0x0F];
            encoded += hexDigits[character & 0x0F];
        }
    }

    return encoded;
}

void appendField(String& json, bool& first, const char* key, const String& value, bool quoted) {
    if (!first) {
        json += ',';
    }
    first = false;
    json += '"';
    json += key;
    json += '"';
    json += ':';
    if (quoted) {
        json += '"';
        json += value;
        json += '"';
    } else {
        json += value;
    }
}

String floatToString(float value) {
    String text = String(value, 3);
    text.trim();
    return text;
}

String unsignedLongToString(unsigned long value) {
    return String(value);
}
} // namespace

TwystBackendClient::TwystBackendClient() : baseUrl_("http://localhost:8000") {
    // leave async sender disabled by default; call beginAsyncFrameSender()
}

void TwystBackendClient::setBaseUrl(const String& baseUrl) {
    baseUrl_ = baseUrl;
    while (baseUrl_.endsWith("/")) {
        baseUrl_.remove(baseUrl_.length() - 1);
    }
}

const String& TwystBackendClient::getBaseUrl() const {
    return baseUrl_;
}

bool TwystBackendClient::connectWiFi(const char* ssid, const char* password, unsigned long timeoutMs) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    const unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
        delay(250);
    }

    return WiFi.status() == WL_CONNECTED;
}

bool TwystBackendClient::isWiFiConnected() const {
    return WiFi.status() == WL_CONNECTED;
}

bool TwystBackendClient::startRecord(const String& motionName, String* responseBody) {
    return requestJson("POST", "/record/start", buildMotionJson("motion_name", motionName), responseBody);
}

bool TwystBackendClient::sendRecordFrame(const TwystBackendFrame& frame, String* responseBody) {
    // If async sender is active, enqueue the frame for background sending.
    if (frameQueue_ != nullptr) {
        return enqueueFrame(frame);
    }
    return requestJson("POST", "/frame", buildFrameJson(frame), responseBody);
}

bool TwystBackendClient::stopRecord(int bezierOrder, String* responseBody) {
    String path = "/record/stop?bezier_order=";
    path += String(bezierOrder);
    return requestJson("POST", path, String(), responseBody);
}

bool TwystBackendClient::startCompare(const String& referenceName, String* responseBody) {
    return requestJson("POST", "/compare/start", buildMotionJson("reference_name", referenceName), responseBody);
}

bool TwystBackendClient::sendCompareFrame(const TwystBackendFrame& frame, String* responseBody) {
    if (frameQueue_ != nullptr) {
        return enqueueFrame(frame);
    }
    return requestJson("POST", "/frame", buildFrameJson(frame), responseBody);
}

bool TwystBackendClient::stopCompare(String* responseBody) {
    return requestJson("POST", "/compare/stop", String(), responseBody);
}

bool TwystBackendClient::listMotions(String* responseBody) {
    return requestJson("GET", "/motions", String(), responseBody);
}

bool TwystBackendClient::getMotion(const String& name, String* responseBody) {
    String path = "/motions/";
    path += encodePathSegment(name);
    return requestJson("GET", path, String(), responseBody);
}

bool TwystBackendClient::deleteMotion(const String& name, String* responseBody) {
    String path = "/motions/";
    path += encodePathSegment(name);
    return requestJson("DELETE", path, String(), responseBody);
}

String TwystBackendClient::makeUrl(const String& path) const {
    if (path.startsWith("http://") || path.startsWith("https://")) {
        return path;
    }

    String url = baseUrl_;
    if (!path.startsWith("/")) {
        url += '/';
    }
    url += path;
    return url;
}

String TwystBackendClient::buildFrameJson(const TwystBackendFrame& frame) {
    String json;
    json.reserve(192);
    json += '{';

    bool first = true;
    appendField(json, first, "acc_x", floatToString(frame.acc_x), false);
    appendField(json, first, "acc_y", floatToString(frame.acc_y), false);
    appendField(json, first, "acc_z", floatToString(frame.acc_z), false);
    appendField(json, first, "gyro_x", floatToString(frame.gyro_x), false);
    appendField(json, first, "gyro_y", floatToString(frame.gyro_y), false);
    appendField(json, first, "gyro_z", floatToString(frame.gyro_z), false);
    appendField(json, first, "roll", floatToString(frame.roll), false);
    appendField(json, first, "pitch", floatToString(frame.pitch), false);
    appendField(json, first, "yaw", floatToString(frame.yaw), false);

    if (frame.hasTimestamp) {
        appendField(json, first, "timestamp", unsignedLongToString(frame.timestamp), false);
    }

    json += '}';
    return json;
}

String TwystBackendClient::buildMotionJson(const char* key, const String& value) {
    String json;
    const String escaped = escapeJsonString(value);
    json.reserve(64 + escaped.length());
    json += '{';
    json += '"';
    json += key;
    json += '"';
    json += ':';
    json += '"';
    json += escaped;
    json += '"';
    json += '}';
    return json;
}

bool TwystBackendClient::requestJson(const char* method, const String& path, const String& body, String* responseBody) {
    if (!isWiFiConnected()) {
        return false;
    }

    WiFiClient client;
    HTTPClient http;
    const String url = makeUrl(path);

    if (!http.begin(client, url)) {
        return false;
    }

    http.addHeader("Content-Type", "application/json");

    int statusCode = -1;
    if (strcmp(method, "GET") == 0) {
        statusCode = http.GET();
    } else if (strcmp(method, "DELETE") == 0) {
        statusCode = http.sendRequest("DELETE");
    } else {
        statusCode = http.POST(body);
    }

    if (responseBody != nullptr) {
        *responseBody = http.getString();
    }

    http.end();
    return statusCode >= 200 && statusCode < 300;
}

bool TwystBackendClient::beginAsyncFrameSender(size_t queueLength, UBaseType_t taskPriority) {
    if (frameQueue_ != nullptr) {
        return true; // already running
    }

    QueueHandle_t q = xQueueCreate((UBaseType_t)queueLength, sizeof(TwystBackendFrame));
    if (q == nullptr) {
        return false;
    }

    TaskHandle_t taskHandle = nullptr;
    // stack size 8192 bytes - provides headroom for HTTPClient usage
    BaseType_t res = xTaskCreate(
        frameWorkerTask,
        "TwystFrameWorker",
        8192 / sizeof(StackType_t),
        this,
        taskPriority,
        &taskHandle);

    if (res != pdPASS) {
        vQueueDelete(q);
        return false;
    }

    frameQueue_ = static_cast<void*>(q);
    frameTaskHandle_ = static_cast<void*>(taskHandle);
    return true;
}

void TwystBackendClient::stopAsyncFrameSender() {
    if (frameTaskHandle_ != nullptr) {
        vTaskDelete(static_cast<TaskHandle_t>(frameTaskHandle_));
        frameTaskHandle_ = nullptr;
    }
    if (frameQueue_ != nullptr) {
        vQueueDelete(static_cast<QueueHandle_t>(frameQueue_));
        frameQueue_ = nullptr;
    }
}

bool TwystBackendClient::enqueueFrame(const TwystBackendFrame& frame) {
    if (frameQueue_ == nullptr) {
        return false;
    }
    // try to post without blocking; if full, drop the frame
    BaseType_t res = xQueueSend(static_cast<QueueHandle_t>(frameQueue_), &frame, 0);
    return res == pdTRUE;
}

void TwystBackendClient::frameWorkerTask(void* pvParameters) {
    TwystBackendClient* self = static_cast<TwystBackendClient*>(pvParameters);
    QueueHandle_t q = static_cast<QueueHandle_t>(self->frameQueue_);
    TwystBackendFrame frame;

    for (;;) {
        if (xQueueReceive(q, &frame, portMAX_DELAY) == pdTRUE) {
            String resp;
            // Use the unified /frame endpoint
            self->requestJson("POST", "/frame", buildFrameJson(frame), &resp);
        }
    }
}
