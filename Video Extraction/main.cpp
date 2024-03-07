// https://learn.microsoft.com/en-us/azure/kinect-dk/record-external-synchronized-units
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/transformation
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/fastpointcloud
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop/examples/green_screen
// https://gist.github.com/UnaNancyOwen/9f16ce7ea4c2673fe08b4ce4804fc209
// https://github.com/nlohmann/json
// https://github.com/microsoft/Azure-Kinect-Sensor-SDK/issues/1188

// k4arecorder.exe command used: k4arecorder.exe --external-sync <master|subordinate> --depth-mode NFOV_UNBINNED -e -8 -r 15 -l 120 <output_path.mkv> 

#include "OnlineExtraction.hpp"
#include "PlaybackExtraction.hpp"

int main() {

	// Online settings
	
	//int recording_duration = 15;
	//std::string base_path = "C:\\Users\\zenob\\Desktop\\output";
	//int num_devices = 3;
	// online_extraction(recording_duration, base_path, num_devices);

	// Playback settings

	std::string input_path = "C:\\Users\\zenob\\Desktop\\dataset\\task1\\person1\\front.mkv";
	playback_extraction(input_path);

	return 0;
}