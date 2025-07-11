# 🛩️ Autonomous FOD Detection System using VTOL Drone

This project is a real-time Foreign Object Debris (FOD) detection system that uses YOLO ML Model on the server and a VTOL drone for automatic scanning and geo-tagging of debris on runways or racing circuits. Built for accuracy, safety, and automation.

---

## 🔍 System Overview

A VTOL drone conducts aerial scans and streams live video to the server. The server uses a trained AI model to detect debris in real-time. If any FOD is found, it marks its location, retrieves precise GPS coordinates from a telemetry module, logs them, and alerts the ground team.

An automatic pick-and-place rover for FOD removal is currently a work in progress.

---
```mermaid
flowchart LR
    A[🛩️ VTOL Drone Launch] --> B[Configure Autonomous Flight Path]
    B --> C[Start Autonomous Flight]
    C --> D[GoPro Camera Activated]
    D --> E[Live Video Stream via VTX/HDMI TX-RX]
    E --> F[Server Receives Video Feed]
    F --> G[YOLO AI Model Processes Frame]
    G --> H{FOD Detected?}
    
    H -->|Yes| I[Mark FOD in Video Frame]
    I --> J[Retrieve GPS Coordinates from Telemetry Module]
    J --> K[Log Location to Database]
    K --> L[Send Alert to Ground Team]
    L --> M[Continue Scanning]
    
    H -->|No| N{Flight Path Complete?}
    N -->|No| O[Continue to Next Waypoint]
    O --> E
    N -->|Yes| P[Return to Home Position]
    P --> Q[Land Safely]
    Q --> R[Mission Complete]
    
    M --> N
    
    %% Manual Override Path
    S[RX Remote Control] -.->|Manual Override| T[Manual Flight Control]
    T --> U{Conflict Resolution}
    U -->|Resume Auto| C
    U -->|Manual Landing| V[Manual Landing]
    V --> W[Mission Ended]
    
    %% Future Enhancement
    X[🚧 Future: Pick-and-Place Rover] -.->|WIP| Y[Automatic FOD Removal]
    L -.->|Coordinates| X
    
    %% Hardware Components
    subgraph Hardware ["🔧 Hardware Components"]
        H1[VTOL Drone]
        H2[GoPro Camera]
        H3[VTX/HDMI TX-RX]
        H4[Telemetry Module]
        H5[RX Remote]
    end
    
    %% Software Components
    subgraph Software ["💻 Software Components"]
        S1[YOLO AI Model]
        S2[Server Processing]
        S3[Database Storage]
        S4[Alerting System]
    end
    
    %% Styling - GitHub optimized with high contrast
    classDef drone fill:#ffffff,stroke:#0366d6,stroke-width:3px,color:#24292e
    classDef detection fill:#f6f8fa,stroke:#6f42c1,stroke-width:3px,color:#24292e
    classDef alert fill:#ffffff,stroke:#d73a49,stroke-width:3px,color:#24292e
    classDef manual fill:#fff8f0,stroke:#f66a0a,stroke-width:3px,color:#24292e
    classDef future fill:#f0fff4,stroke:#28a745,stroke-width:3px,stroke-dasharray: 5 5,color:#24292e
    
    class A,B,C,D,E,O,P,Q,R drone
    class F,G,H,I,J detection
    class K,L,M alert
    class S,T,U,V,W manual
    class X,Y future
```

---

## ⚙️ Components

### Hardware
- **VTOL Drone** – Capable of autonomous flight and manual override  
- **GoPro Camera** – Mounted on the drone for video capture  
- **VTX / HDMI TX-RX** – Transfers live video feed to the server  
- **Telemetry Module** – Provides real-time GPS data to the server  
- **RX Remote** – Manual control in conflict scenarios  

### Software
- **YOLO/AI Model** – Detects FOD in video frames  
- **Server** – Processes video, logs coordinates, and alerts ground team  
- **Database** – Stores FOD detection data for audit and review  
- **Alerting System** – Sends notification and co-ordinates upon detection  

---

## 🔁 Workflow

1. VTOL drone is launched and performs a configured autonomous flight
2. It streams live video to the server via VTX/HDMI TX-RX for low latency
3. The server analyzes each frame using a YOLO model trained for FOD detection
4. If FOD is detected:
   - Marked in video
   - Coordinates retrieved from telemetry module
   - Location logged into database
   - Ground team is alerted instantly
5. If no FOD is detected, drone continues or returns home safely

---

## 🧷 Folder Structure

| Path             | Description                         |
|------------------|-------------------------------------|
| `app.py`         | Core application logic              |
| `templates/`     | Frontend assets (HTML files)        |
| `mindmap.png`    | Flowchart of the system             |
| `README.md`      | Project documentation               |




---

## 🔧 Work in Progress

We are currently developing an **autonomous pick-and-place rover** that will work alongside the detection system to **remove detected FODs** from the runway or track in real time.

This rover will:
- Be deployed upon detection confirmation
- Navigate to the marked GPS coordinates
- Pick up FODs using a robotic mechanism
- Work autonomously or via manual override when needed

---

## 👥 Team

- **Ankit Prajapati** (Team Lead)
- **Yuvraj Tiwari** 
- **Moksh Dandotiya**  
- **Ankit Gurjar** 

---


## 📜 License

This project is **not open-source**. All rights are **strictly reserved** by the original authors.  
Any reproduction, redistribution, or unauthorized use of the code, models, or ideas is **strictly prohibited** without prior written permission.

See `LICENSE.md` for full legal terms.

---

## 👨‍💻 Author
- Written and maintained by Moksh Dandotiya.
- Contact: moksh@duck.com

---



