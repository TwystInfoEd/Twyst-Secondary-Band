# TwystBackend

Small ESP32 helper for sending Twyst IMU frames to the FastAPI backend.

## What it does

- Connects to Wi-Fi.
- Sends `record` and `compare` session requests.
- Serializes the backend frame schema:
  - `acc_x`, `acc_y`, `acc_z`
  - `gyro_x`, `gyro_y`, `gyro_z`
  - `roll`, `pitch`, `yaw`
  - optional `timestamp`

## Example

```cpp
#include "TwystBackend.h"

TwystBackendClient backend;

void setup() {
    Serial.begin(115200);
    backend.setBaseUrl("http://192.168.1.50:8000");
    backend.connectWiFi("your-ssid", "your-password");
  // Start the async frame sender with a small queue so sending doesn't block
  backend.beginAsyncFrameSender(16, 1);
    backend.startCompare("squat_reference");
}

void loop() {
    TwystBackendFrame frame;
    frame.acc_x = -0.195f;
    frame.acc_y = 0.006f;
    frame.acc_z = 1.036f;
    frame.gyro_x = 3.275f;
    frame.gyro_y = -1.153f;
    frame.gyro_z = 0.069f;
    frame.roll = 0.40f;
    frame.pitch = 10.42f;
    frame.yaw = 0.00f;

    // Enqueue frame; returns immediately. If the queue is full the frame is dropped.
    backend.sendCompareFrame(frame);
    delay(200);
}
```

## Supported endpoints

- `POST /record/start`
- `POST /frame`
- `POST /record/stop?bezier_order=8`
- `GET /motions`
- `GET /motions/{name}`
- `DELETE /motions/{name}`
- `POST /compare/start`
- `POST /compare/stop`
