# Fruit Sorting System

A simple prototype of an automated fruit sorting device using Raspberry Pi, a camera module, and a machine learning model. The system captures fruit images, classifies them using a trained TFlite model, and moves fruits to the correct output using stepper motors.

---

## Overview
This project demonstrates a basic fruit sorting mechanism using:

- Raspberry Pi for processing  
- Camera for image capture  
- TFlite model for classification  
- Stepper motor and servo for mechanical sorting  

The system is designed for lightweight edge deployment and real-time operation.

---

## Features
- Real-time fruit image capture  
- TFlite-based machine learning inference  
- Stepper motor sorting mechanism  
- Easy to modify and extend  
- Sensor integration (ultrasonic or IR)

---

## Requirements

### Hardware
- Raspberry Pi 4 or 5  
- Camera Module (Pi Camera / Arducam)
- Servo mg995
- Stepper Motor + Driver (L298N / A4988)  
- Jumper wires & power supply  
- Sensors & Coral USB TPU

### Software
- Raspberry Pi OS  
- Python 3.10+  
- Packages listed in `requirements.txt`

---

## Installation

### Clone the repository
```bash
git clone https://github.com/zhraan/Fruit_sorting_system.git
cd Fruit_sorting_system
```

### Create virtual environment with access system package (--system-site-packages)
```bash
python3 -m venv env
source env/bin/activate --system-site-packages
```

### Install dependencies
```bash
pip install -r requirements.txt
```
Some libraries cannot be installed via pip and must be installed using APT.

#### Install Picamera2:
```bash
sudo apt update
sudo apt install -y python3-picamera2
```

#### Install GPIO library:
```bash
sudo apt install -y python3-rpi.gpio
```
### Before running
- Enable camera:
```bash
sudo raspi-config
```
- Interface Options → Camera → Enable
- Ensure motors and sensors are wired correctly
- Place the model file best.tflie in the project folder

### Running the System
Using the script:
```bash 
start_app_server.sh
```
Or run the main Python file:

bash
```bash
python rasp/picamera/ta_picamera_stepper.py
```

## Project Structure
```bash
Fruit_sorting_system/
│
├── rasp/
│   └── picamera/
|   |    └── picam
│   |         └── ta_picamera_stepper.py                    # Main script (for mangosteen classification)
|   |         └── cavendish/
|   |             └── model_360_linieractuator.py           # Second script (for banana classification)
|   |    
│   └── Model manggis EdgeTPU/
|   |    └── best_saved_model/
|   |        └── best_full_integer_quant_edgetpu.tflite     # ML model (for mangosteen classification)
│   └── Model Pisang Cavendish/
|        └── best_saved_model/
|            └── best_full_integer_quant_edgetpu.tflite     # ML model (for banana classification)
|
├── best.onnx                         
├── start_app_server.sh                                     # Start script
├── requirements.txt
└── README.md
```

## Model
The file best.tflite contains the machine learning model used to classify fruits.
Training is done separately and only the inference model is included here.




