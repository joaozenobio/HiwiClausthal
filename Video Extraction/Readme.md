
# Video Extraction

## Hardware setup

Set up device: https://learn.microsoft.com/en-us/azure/kinect-dk/set-up-azure-kinect-dk#set-up-hardware

Set up sychronization: https://learn.microsoft.com/en-us/azure/kinect-dk/multi-camera-sync

Example of camera placement (Right is the master):

<img src="https://github.com/joaozenobio/HiwiClausthal/blob/cd0dc995efff87501e41aebead7184d85dce845e/Video%20Extraction/camera_setup.jpg" alt="drawing" width="500"/>

## Software environment setup

1. Clone the repository at a desired directory.
2. Set up your Azure Kinect DK by downloading the Sensor SDK: https://learn.microsoft.com/en-us/azure/kinect-dk/sensor-sdk-download.
3. Download Microsoft Visual Studio 2022: https://visualstudio.microsoft.com/.
4. Download the packages for C++ Desktop development when Visual Studio 2022 starts for the first time.
5. Open the Visual Studio 2022 NuGet package manager (under the "project" dropdown button list) and install the Azure Kinect Sensor package by searching its name: https://www.nuget.org/packages/Microsoft.Azure.Kinect.Sensor/.
6. Install the nlohmann.json package by searching its name: https://www.nuget.org/packages/nlohmann.json/.
8. Download OpenCV's latest release: https://opencv.org/releases/ and extract it to a desired directory.
9. Include the OpenCV bin folder, commonly at opencv\build\x64\vc<some-version>\bin, to the Windows system PATH by accessing the Windows system properties, then the Environment Variables and, under the system variables list, editing the path variable and adding the complete path to the bin folder. Move it to the top of the list for higher priority.
10. Open project properties and choose to modify the release configuration with the platform x64.
11. Expand the C/C++ tab, select the General tab and edit the Additional Include Directories using the arrow button that can be found on the right part of the text box.
12. Under the C/C++ tab, select the Language tab, edit the C++ Language Standart and select ISO C++20 Standard (/std:c++20).
13. Use the new line and then the ellipsis button to browse for a directory and select the include folder, commonly inside the OpenCV directory at opencv/build/include.
14. Expand the linker tab, select the General tab, edit the Additional Library Directories and include the lib folder, commonly at opencv\build\x64\vc<some-version>\lib, using the same method as the previous steps.
15. Under the linker tab, select the Input tab, edit the Additional Dependencies and include the opencv_world lib file, commonly at opencv\build\x64\vc<some-version>\lib/opencv_world<some_version>.lib (without "d" at the end of the name), using the same method as the previous steps, but this time only copying and pasting the file name instead of the full path of the file.
16. Modify the version of the application by selecting the release version.
17. Build the application.

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
