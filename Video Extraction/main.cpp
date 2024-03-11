#include "OnlineExtraction.hpp"
#include "PlaybackExtraction.hpp"

int main() {

	// Online settings
	
	int recording_duration = 15;
	std::string base_path = "C:\\Users\\zenob\\Desktop\\output";
	int num_devices = 3;
	onlineExtraction(recording_duration, base_path, num_devices);

	// Playback settings

	//std::string input_path = "C:\\Users\\zenob\\Desktop\\dataset\\task1\\person1\\front.mkv";
	//playbackExtraction(input_path);

	return 0;
}