
# Video Extraction

## Hardware setup

Set up device: https://learn.microsoft.com/en-us/azure/kinect-dk/set-up-azure-kinect-dk#set-up-hardware

Set up sychronization: https://learn.microsoft.com/en-us/azure/kinect-dk/multi-camera-sync

Example of camera placement:

<img src="https://github.com/joaozenobio/HiwiClausthal/blob/cd0dc995efff87501e41aebead7184d85dce845e/Video%20Extraction/camera_setup.jpg" alt="drawing" width="500"/>

Right is the master.

## Software environment setup

Set up your Azure Kinect DK: https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download

NuGet package: https://www.nuget.org/packages/Microsoft.Azure.Kinect.Sensor/

## Video recording

Recording tool instructions: https://learn.microsoft.com/en-us/azure/kinect-dk/record-external-synchronized-units.

Example of usage:
```
k4arecorder.exe --external-sync <master|subordinate> --depth-mode NFOV_UNBINNED -e -8 -r 15 -l 120 <output_path.mkv>
```

Camera settings: Exposure = 1/8000 seconds | frame rate 15 FPS | recording length 120 seconds

External-sync parameter depends on whether the device is the master or one of the subordinates.

## Extracting data from the recordings

PlaybackExtraction.cpp contains the function playbackExtraction which takes the path to a recording and creates a folder with the same name as the recording file and extract the data from it into the following tree:

\<recording name\> <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\----color <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|---- images <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\---- timestamps.txt <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\----depth <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\---- images <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\---- point_clouds <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\---- raw_matrices <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\---- timestamps.txt <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\----ir <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\---- images <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\---- raw_matrices <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\---- timestamps.txt <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;|\----imu.json <br>

- Each directory contain images from a different camera and a timestamp file containing the timestamps of all the images inside the images folder. 

- The depth and IR camera, the original matrix returned from the sensors is saved at the raw_matrices folder.

- The depth sensor the sensor data is also converted to point clouds.

## Extracting data online

OnlineExtraction.cpp contains the function onlineExtraction which takes a duration for a new recording, an output path and the number of devices. It creates the output directory and extract the data online into the same tree as the playbackExtraction.

## References

https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/transformation <br>
https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/fastpointcloud <br>
https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/green_screen <br>
https://gist.github.com/UnaNancyOwen/9f16ce7ea4c2673fe08b4ce4804fc209 <br>
https://github.com/nlohmann/json <br>
https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1188 <br>
