# LicheeRV-Nano-YOLOV8N

This is an embedded prototype of Yolo Object Tracking on a [LicheeRV Nano](https://wiki.sipeed.com/hardware/en/lichee/RV_Nano/1_intro.html), based on these main GitHub repositories:

- [Cvitek](https://github.com/milkv-duo/cvitek-tdl-sdk-sg200x)
- [LicheeRV Official Repo](https://github.com/scpcom/LicheeRV-Nano-Build)

# Objective

Develop an AI object tracker/recognizer using the powerful NPU unit available on the board to identify people/objects based on the [YOLO AI pretrained Model](https://milkv.io/docs/duo/application-development/tdl-sdk/tdl-sdk-yolov8) and stream the results directly through a local host server.

# Designing Process

First, we designed a flow diagram to serve as a step-by-step guide for developing and deploying this model:

![Flow Diagram](image.png)

Initially, we boot an embedded Linux image with the necessary files from the mentioned repositories. Once the system is running, the software pipeline follows the architecture below.

## 1. Hardware Input

The entry point is the **GC4653** sensor. The system begins by parsing the `sensor_cfg.ini` file to dynamically determine the sensor's resolution capabilities.

The raw data travels through the MIPI/CSI interface to the **VI (Video Input)** module. Here, the Image Signal Processor (ISP) cleans the signal and prepares it for the pipeline. This module is then "bound" via software to the video processing subsystem, ensuring a direct data path without CPU intervention.

## 2. Memory Management (VB)

Before processing any video, we initialize the **Video Buffer (VB) Pools**. This is a critical step in embedded systems to manage RAM efficiently. Based on the code configuration, we allocate distinct Direct Memory Access (DMA) blocks:

* **Pool VI:** Reserved for the raw sensor resolution.
* **Pool VENC:** Reserved for the high-quality video output (1920x1080).
* **Pool TDL:** Reserved for the AI input (downscaled to 640x480).
* **Pool PREPROC:** A specific RGB buffer for the algorithm preprocessing.

## 3. Video Processing (VPSS)

The core signal processing happens in the **VPSS (Video Process Sub-System)**. We configure **Group 0** to act as a splitter and scaler, generating two simultaneous streams from the single camera input:

1.  **Channel 0 (VENC):** Scales the image to **1920x1080 (NV21)**. This high-resolution stream is intended for the human eye (encoding and streaming).
2.  **Channel 1 (TDL):** Scales the image to **640x480 (NV21)**. This lower-resolution stream is optimized for the NPU, allowing the YOLOv8 model to run faster without processing unnecessary pixels.

## 4. Application Logic & TDL

The application logic is split into two asynchronous threads to maximize performance:

### Thread TDL (AI Inference)

This thread pulls frames from VPSS Channel 1. It runs the **YOLOv8** model using the `cvimodel` loaded at startup. When an object (e.g., a person) is detected, the coordinates are rescaled to match the 1080p output and stored in a shared "Global Object Metadata" structure protected by a Mutex.

### Thread VENC (Encoding & Drawing)

This thread handles the visual output. It pulls the high-res frame from VPSS Channel 0 and performs the following steps:

1.  **Draw:** It retrieves the latest metadata from the AI thread and draws the bounding boxes directly onto the NV21 buffer.
2.  **Encode:** The modified frame is sent to the VENC hardware to be compressed into **H.264**.
3.  **Buffer:** The resulting bitstream is stored in a Ring Buffer in RAM, ready for distribution.

## 5. HTTP Server (CivetWeb)

Finally, a **CivetWeb** server runs on port 8080 to distribute the content. It exposes specific endpoints mapped to the Ring Buffer:

* `/stream`: Provides a raw H.264 stream, ideal for desktop players like VLC or FFplay.
* `/mse_stream`: Wraps the H.264 packets into **fragmented MP4 (fMP4)** containers. This allows modern web browsers to play the low-latency stream directly using Media Source Extensions (MSE) without needing external plugins.
* `/`: Serves a web interface with JavaScript controls to view the camera and tracking statistics.