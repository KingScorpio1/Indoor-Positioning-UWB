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
```

## ⚙️ Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/YOUR_USERNAME/uwb-positioning-project.git
   cd uwb-positioning-project
   ```

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```
   *Note: Requires `tensorflow`, `matplotlib`, `minimalmodbus`, `pandas`, `ttkbootstrap`, and `scikit-learn`.*

## 🚦 Usage

1. **Connect Hardware:** Plug in your Master Anchor via USB to your computer.
2. **Run the Application:**
   ```bash
   python main_app.py
   ```
3. **Configure the System:**
   - Select the correct **COM Port** from the dropdown menu.
   - Switch to the **Anchor Setup** tab to ensure anchor coordinates (A, B, C, D) match your physical environment.
4. **Start Tracking:** 
   - Click **Connect**.
   - Click **Start**. 
   - Navigate to the **2D View** or **3D View** tabs to see the live tracking.

## 🧠 AI Training Workflow
To improve the AI accuracy for your specific room:

1. **Data Collection:** Drive a robot equipped with both a UWB Tag and a LiDAR sensor.
2. **Recording:** Record `rosbags` while covering the entire floor area (aisles, corners, and near interference sources like monitors).
3. **Preprocessing:** Synchronize UWB distances with LiDAR ground-truth coordinates and save to `data/merged_training_data.csv`.
4. **Train the Model:**
   ```bash
   python data/train_nn.py
   ```
5. **Deployment:** The new model will be saved to `data/models/nn_model/` and will be automatically loaded the next time the application starts.

## 📝 Authors
- **Sule Smith** - *Initial Work & Hardware Integration*
- **Contributors** - *Special thanks to the contributors listed in the Decawave datasheet for translation and technical support.*

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
