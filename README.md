## 🔥Industrial Hazard System using Mesh Network

A scalable, reliable industrial safety system to detect gas leaks and fires using a **mesh network of ESP8266 nodes**, integrated with a **local MQTT broker (Mosquitto)** and **Node-RED dashboard** for real-time monitoring and alerts.

### 🚀 Project Overview

This project uses a **mesh network** of ESP8266 nodes to detect **gas (e.g., LPG, methane)** and **fire** across different zones. A **dual-ESP gateway** architecture was implemented to overcome communication conflicts—one ESP handles the mesh network, and the other handles MQTT communication. These two ESPs communicate using **Serial**, ensuring robust data transfer and smooth integration with a **Node-RED dashboard** via **Mosquitto MQTT**.

### ✅ Features
- 🔗 **Reliable ESP8266 mesh network** using PainlessMesh
   
- 🔥 Detection of **gas and fire hazards**
   
- 💬 **Dual-ESP gateway**: one for Mesh, one for MQTT
   
- 🔁 **Serial communication** between the two gateway ESPs
   
- 📡 **Local MQTT broker (Mosquitto)** for message handling
   
- 📊 **Node-RED dashboard** for real-time monitoring and alerts
   
- 🔊 **Buzzer and LED alerts** at relay nodes
 
 - 📅 Sensor data updates every **1 second**

### 🧰Hardware Components 
This project used simple hardware like **ESP8266** (D1 Mini) for both gateway and nodes, **MQ-6 Gas Sensors** for gas detection (LPG, Butane, Combustible gases), **IR Fire detector Module** for detecting fire, **buzzer and LED** for alarms and **Li-Po Batteries** for power supply.


### 📡 System Architecture
![Screenshot 2025-05-19 012831](https://github.com/user-attachments/assets/947a179e-a5e4-4193-926b-0b0e01bddc46)

In this following figure **A** represents the gateway node which is a part of the mesh network consisting of other relay nodes. The gateway is connected to the **MQTT Broker / Server** which helps the dashboard to visualize data and status on devices.

### 📋 Data Flow Summary

-   **Relay nodes** send gas/fire status every **1 second** via PainlessMesh.
    
-   The **Mesh ESP** (gateway node 1) receives data and forwards it over **Serial**.
    
-   The **MQTT ESP** (gateway node 2) reads Serial input and publishes to the **local MQTT broker**.
    
-   **Node-RED** subscribes to MQTT topics and updates the dashboard in real time.

### 🧠 Node Behavior
- Each node operates independently and pushes its data periodically.

- No need for a central hashmap or memory-intensive tracking—data freshness is ensured by frequent updates (1s).

- Local alerts (buzzers, LEDs) are triggered immediately when hazards are detected.


### 📊 Node-RED Dashboard
The dashboard includes:

- 🔥 Real-time gas and fire status per room

- 🕒 Timestamp of the latest update

- 📈 Trend graphs 

- 🔘 Control buttons for resetting buzzer

- 🟢 Node connectivity inferred from live updates

<img src="https://github.com/user-attachments/assets/a543c9c2-170f-4477-a845-dd6a28b13e47" alt="Alt Text" width="600" height="300">

<img src="https://github.com/user-attachments/assets/28f56c4c-c05a-48ed-b1f6-0fe44217ecb4" alt="Alt Text" width="600" height="300">


### 📸 Images & Circuit Diagrams


<img src="https://github.com/user-attachments/assets/9489fb52-94d1-4c3d-9ea2-623135f877ea" alt="Alt Text" width="300" height="400">
<img src="https://github.com/user-attachments/assets/0e69d76d-fbc8-41c3-a62f-a72736db007b" alt="Alt Text" width="400" height="300">






