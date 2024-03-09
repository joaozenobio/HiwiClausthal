
# Video Extraction

## Hardware setup

Set up device: https://learn.microsoft.com/en-us/azure/kinect-dk/set-up-azure-kinect-dk#set-up-hardware

Set up sychronization: https://learn.microsoft.com/en-us/azure/kinect-dk/multi-camera-sync

Example of camera placement:

<img src="https://github.com/joaozenobio/HiwiClausthal/blob/52b50694362890f05ee62358d329435e1d92a1fd/Video%20Extraction/cameras_setup.png" alt="drawing" width="500"/>

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

PlaybackExtraction.cpp contains the functions playbackExtraction which takes the path to a recording and creates a folder with the same name as the recording file and extract the data from it into the following tree:

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

