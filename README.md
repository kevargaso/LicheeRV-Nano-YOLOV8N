# LicheeRV-Nano-YOLOV8N

This is an embedded prototype of Yolo Object Tracking on a [LicheeRV Nano](https://wiki.sipeed.com/hardware/en/lichee/RV_Nano/1_intro.html), based on these main GitHub repositories:

- [Cvitek](https://github.com/milkv-duo/cvitek-tdl-sdk-sg200x)
- [SPCOM](https://github.com/scpcom/LicheeRV-Nano-Build)

# Objective

Develop an AI object tracker/recognizer using the powerful NPU unit available on the board to identify people/objects based on the [YOLO AI pretrained Model](https://milkv.io/docs/duo/application-development/tdl-sdk/tdl-sdk-yolov8) and stream the results directly through a local host server.

# Designing Process

First, we designed a flow diagram to serve as a step-by-step guide for developing and deploying this model:

![Flow Diagram](image.png)

## Booting Process and files preparation

Initially, we boot an embedded Linux image with the necessary files:

- **sample/cvi_tdl/sample_vi_od.c**:
  This is the core source code for Object Detection. It implements a callback function "SampleOD" using a **multithreaded architecture** to parallelize processing and balance the workload between the CPU and the NPU.

  The file structure is organized into three key components:

  1.  **Threading Logic:**
      * **Inference Thread:** Pulls frames from the secondary channel (`VPSS_CHN1`) and executes the NPU inference.
      * **Encoding Thread:** Pulls frames from the main channel (`VPSS_CHN0`), retrieves metadata from the inference thread, draws bounding boxes/labels, and pushes the frame to the RTSP server.

  2.  **Video Middleware Configuration (`get_middleware_config`):**
      * **VI (Video Input):** Captures raw sensor data (settings loaded from `sensor_cfg.ini`).
      * **VB (Video Buffer Pools):** Allocates 3 DMA memory pools:
          * Pool 1: Native Sensor Resolution.
          * Pool 2: AI Input (Resized).
          * Pool 3: AI Pre-processing (RGB Planar).
      * **VPSS (Video Process Sub-System):** Sets up a "Dual" mode group (`Grp0`) splitting the signal:
          * `CHN0`: 1920x1080 (Visualization/RTSP).
          * `CHN1`: 1920x1080 (AI Model Input).

  3.  **TDL SDK Integration:**
      * **Supported Models:** MobileDetV2 (person, vehicle, pets, coco80), YOLOv3, and YOLOX.
      * **Filtering:** Hardcoded to filter for **PERSONS** (`CVI_TDL_DET_TYPE_PERSON`) in this sample.
      * **Threshold:** Configurable via command-line argument (default: 0.5).


- **[COCO trainig data set](https://cocodataset.org/#home)**: It´s the brain used on the NPU to identify:

    - Object segmentation
    - Recognition in context
    - Superpixel stuff segmentation
    - 330K images (>200K labeled)
    - 1.5 million object instances
    - 80 object categories
    - 91 stuff categories
    - 5 captions per image
    - 250,000 people with keypoints


## 1. Hardware Input

The entry point is the **GC4653** sensor. The system begins by parsing the `sensor_cfg.ini` file to dynamically determine the sensor's resolution capabilities.

The raw data travels through the MIPI/CSI interface to the **VI (Video Input)** module. Here, the Image Signal Processor (ISP) cleans the signal and prepares it for the pipeline. This module is then "bound" via software to the video processing subsystem, ensuring a direct data path without CPU intervention.

## 2. Memory Management (VB)

Before processing any video, we initialize the **Video Buffer (VB) Pools**. This is a critical step in embedded systems to manage RAM efficiently. Based on the code configuration, we allocate distinct Direct Memory Access (DMA) blocks:

* **Pool VI:** Reserved for the raw sensor resolution.
* **Pool VENC:** Reserved for the high-quality video output (1920x1080).
* **Pool TDL:** Reserved for the AI input (downscaled to 640x480).
* **Pool PREPROC:** A specific RGB buffer for the algorithm preprocessing.

## 2.1 Runtime & Model Initialization (TDL)

With the memory pools ready, the next stage is initializing the TDL runtime and loading the YOLOv8 model.  
This process relies on the Cvitek AI framework, which requires the model to be converted into the `.cvimodel` format beforehand.

During initialization, the following operations occur:

- The TDL engine allocates internal NPU memory for weights, intermediate tensors, and output layers.
- A preprocessing module is enabled to convert incoming NV21 frames into RGB planar format, since the NPU cannot operate directly on NV21.
- The model input dimensions (640×480) are validated to match the VPSS Channel 1 output.

This initialization ensures that all inference operations run entirely inside the dedicated hardware NPU, avoiding unnecessary CPU usage and guaranteeing real-time performance on the SG2002-based LicheeRV Nano.




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

4.1 Person Detection, Tracking & People Counting
This module adds the logic that transforms raw YOLO detections into stable tracked IDs and a real-time people counter.  
It runs in parallel with the inference thread and shares metadata with the encoder thread.

### 4.1.1 Person Recognition (YOLOv8 on NPU)
Each frame from VPSS Channel 1 (640×480) is sent to the TDL engine running the YOLOv8 `.cvimodel`.  
The model outputs multiple detected objects, but this application filters only the class:

- **PERSON (CVI_TDL_DET_TYPE_PERSON)**

For each detection, the model returns:

- Bounding Box (x, y, width, height)
- Confidence score
- Class ID
- Timestamp

Bounding boxes are later rescaled to match the 1920×1080 image from VPSS Channel 0.

### 4.1.2 Tracking Logic (ID Assignment)
YOLOv8 does not provide tracking IDs, so software logic assigns identities between frames:

1. The system stores the detections from the previous frame.
2. For every new bounding box, it calculates:
   - IoU (Intersection over Union)  
   - Centroid distance  
3. If the new box matches a previous one → **same ID**.
4. If it does not match any → **new unique ID is created**.
5. IDs without updates for several frames are removed.

This produces stable tracking even if several people enter or leave the scene.

### 4.1.3 People Counting
The counting mechanism is based on **new unique IDs**:

- When a new person ID is created → **counter = counter + 1**
- People are not double-counted as long as their ID stays active
- If someone exits and re-enters, they get a new ID (simple counting mode)

This method is reliable for entrances, hallways, and areas with linear flow.

### 4.1.4 Metadata Sharing (Thread-Safe)
The tracking module updates a shared metadata structure protected by a Mutex that includes:

- Active person IDs
- Bounding boxes for drawing
- Total number of detected persons
- Last update timestamp

This metadata is consumed by:

- The **encoding thread** (to draw boxes and labels on the NV21 frame)
- The **HTTP/MSE server** (to expose live analytics)


## 5. HTTP Server (CivetWeb)

Finally, a **CivetWeb** server runs on port 8080 to distribute the content. It exposes specific endpoints mapped to the Ring Buffer:

* `/stream`: Provides a raw H.264 stream, ideal for desktop players like VLC or FFplay.
* `/mse_stream`: Wraps the H.264 packets into **fragmented MP4 (fMP4)** containers. This allows modern web browsers to play the low-latency stream directly using Media Source Extensions (MSE) without needing external plugins.
* `/`: Serves a web interface with JavaScript controls to view the camera and tracking statistics.
