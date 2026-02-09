# Data Center Sentinel — Advanced IoT Monitoring & Security (ESP32 + MQTT + JSON Telemetry)

![ESP32](https://img.shields.io/badge/ESP32-Arduino-blue)
![Protocol](https://img.shields.io/badge/Protocol-MQTT-informational)
![Telemetry](https://img.shields.io/badge/Telemetry-JSON-success)
![Simulator](https://img.shields.io/badge/Simulator-Wokwi-purple)

A compact **edge-to-cloud sentinel** designed for **data-center style environments**, combining:
- **Environmental control** (temperature/humidity monitoring + fan automation with hysteresis)
- **Physical security** (armed/disarmed intrusion detection using PIR + door sensor)
- **Real-time observability** via **MQTT topics** and **structured JSON payloads**


---

## Why this project
Data centers face two critical risk categories:
1) **Overheating & thermal instability** (can degrade hardware and increase failure rates)  
2) **Unauthorized physical access** (perimeter breach + interior motion)

This system provides **automated mitigation + remote monitoring** with low-cost components and clear decision logic.

---

## Key capabilities (what makes it “advanced”)
### Environmental automation (anti-chatter control)
- Uses a **hysteresis band** to prevent relay “chatter”:
  - **Fan ON** when temperature exceeds **30°C**
  - **Fan OFF** only after temperature drops below **28°C**
- Adds a humidity warning threshold (**70%**) for early awareness.

### Security state machine (armed/disarmed)
- **Armed mode**: monitors **door perimeter + PIR motion**
- Triggers **local audible alarm** (buzzer) and publishes alarm events to MQTT
- **Disarmed mode**: ignores intrusion sensors and keeps alarm off

### Edge-to-cloud telemetry
- Publishes **structured JSON** to MQTT for dashboards, logging, analytics, or alerting.
- Supports **remote command & control** (ARM/DISARM/STATUS/TEST_ALARM).

---

## Architecture (edge → cloud)
```mermaid
flowchart LR
  subgraph Edge["Server Room (Edge)"]
    ESP["ESP32 Microcontroller"]
    DHT["DHT22 (Temp/Humidity)"]
    PIR["PIR Motion Sensor"]
    DOOR["Reed Switch (Door)"]
    RELAY["Relay (Cooling Fan)"]
    BUZZ["Buzzer (Alarm)"]
    BTN["Push Button (Arm/Disarm)"]
    LED["Status LED"]
    DHT --> ESP
    PIR --> ESP
    DOOR --> ESP
    BTN --> ESP
    ESP --> RELAY
    ESP --> BUZZ
    ESP --> LED
  end

  subgraph Cloud["Cloud Layer"]
    MQTT["MQTT Broker (HiveMQ Public)"]
  end

  subgraph User["User Layer"]
    DASH["Dashboard / Subscriber (Any MQTT Client)"]
  end

  ESP -- WiFi + MQTT --> MQTT
  MQTT --> DASH
