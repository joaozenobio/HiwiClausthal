#include "../include/OnlineExtraction.hpp"
#include "../include/PlaybackExtraction.hpp"

int main() {

	// Online settings
	
	//int recording_duration = 15;
	//std::string base_path = "C:\\Users\\zenob\\Desktop\\output";
	//int num_devices = 2;
	//onlineExtraction(recording_duration, base_path, num_devices);

	// Playback settings

	std::ifstream filein("C:\\Users\\zenob\\Desktop\\files.txt");
	for (std::string input_path; std::getline(filein, input_path); )
	{
		playbackExtraction(input_path);
	}

	return 0;
}