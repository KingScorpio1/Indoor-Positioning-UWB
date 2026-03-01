# AI-Enhanced UWB Positioning System v2.0

An advanced indoor positioning application that uses **Decawave DWM1000** Ultra-Wideband (UWB) hardware combined with a **Neural Network (AI)** to provide high-accuracy, real-time tracking in complex environments.

![Python](https://img.shields.io/badge/python-3.10+-blue.svg)
![TensorFlow](https://img.shields.io/badge/TensorFlow-2.x-orange.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

## 🚀 Overview
Standard UWB signals often suffer from "multipath" jumps and jitter caused by metal objects and furniture. This system solves that by:
1. **Live Data Streaming:** Consuming Modbus RTU data from UWB anchors.
2. **AI Correction:** Using a pre-trained Deep Learning model to compare raw UWB distances against LiDAR ground truth.
3. **Visualization:** Real-time 2D and 3D mapping of both the "Raw Hardware" path and the "AI Corrected" path.

## ✨ Key Features
- **Real-Time 2D/3D Tracking:** Dual-view visualization with automatic camera scaling.
- **Dual-Path Mapping:** Displays Raw (Hardware) path in dashed red and AI-Corrected path in solid green.
- **Hardware Diagnostics:** Live monitoring of Standard Noise, First Path Power, and Receive Power levels.
- **Data Logging:** Export session data to Excel for post-processing and further model training.
- **Modbus Integration:** Full control over Anchor configuration (Antenna delay, Channel, Pulse Repetition Frequency).

## 🛠 Project Structure
```text
uwb_positioning_project/
├── main_app.py           # Main GUI application (Tkinter)
├── visualization.py      # Matplotlib-based real-time 2D/3D engine
├── managers.py           # Data storage for Tags, Anchors, and Logs
├── hardware_interface.py # Modbus RTU communication layer
├── data_parser.py        # Binary packet decoding logic
├── network_handler.py    # TCP Server for robot/LiDAR communication
├── data/                 # Training data, CSVs, and Rosbags
│   └── models/           # Trained .h5 Neural Network models
└── config.py             # Global settings and anchor coordinates