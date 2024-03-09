
# Video Extraction

## Camera placement setup

![Camera placement]()

## Software environment setup

Set up your Azure Kinect DK: https://learn.microsoft.com/en-us/azure/kinect-dk/set-up-azure-kinect-dk

## Video recording

Recording tool instructions: https://learn.microsoft.com/en-us/azure/kinect-dk/record-external-synchronized-units.

Example of usage:
```
k4arecorder.exe --external-sync <master|subordinate> --depth-mode NFOV_UNBINNED -e -8 -r 15 -l 120 <output_path.mkv>
```
Camera settings: Exposure = 1/8000 seconds | frame rate 15 FPS | recording length 120 seconds

External-sync parameter depends on the hardware setup.

## 
