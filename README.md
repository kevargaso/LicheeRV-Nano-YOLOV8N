# LicheeRV-Nano-YOLOV8N.
This is a embedded prototype of Yolo Object Tracking in a [Licheerv Nano](https://wiki.sipeed.com/hardware/en/lichee/RV_Nano/1_intro.html)
 based in this main GitHub repositories:

- [Cvitek](https://github.com/milkv-duo/cvitek-tdl-sdk-sg200x)
- [Licheerv Official Repo](https://github.com/scpcom/LicheeRV-Nano-Build)

# Objective.
Develop and AI object tracker/recognizer using the powerfull NPU unit available on the board to identify people/objects based on [YOLO AI pretrained Model](https://milkv.io/docs/duo/application-development/tdl-sdk/tdl-sdk-yolov8) and streaming directly throught a local host server.

# Designing process.
First, we made a flow diagram to use like a step by step guide to develop and deploy this model:

![Flow Diagram ](image.png)


in addition, firstly we boot a embeded linux image with the necessary files from the mentioned repositories, omce we have it, we followed the flow diagram.