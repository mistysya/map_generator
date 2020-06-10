#ifndef MAP_GENERATOR_HPP
#define MAP_GENERATOR_HPP
#include <ios>
#include <string>
#include <thread>
#include <fstream>

#include "geometry_msgs/Point.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "ConfigFile.hpp"
#include "Position.hpp"

using namespace std;

class MapGenerator {
    public:
        int show_freq;
        bool use_thread;
        bool show_map_in_thread;

        MapGenerator(int argc, char** argv, bool *runMainThread);
        void run();
        Mat getMap();
        int initialize();
        Point getPosition();
	void publisherInitialize();
	void publishPosition();
	void publishMap();

    private:
        int argc;
        char **argv;
        bool is_record_data;
	bool is_publish;
        bool *runMainThread;
        string record_path;
        Point cur_pos;
        Mat cur_map;
        Position pos;
	ros::Publisher position_pub;
	image_transport::Publisher map_pub;

        void initFromConfig(string config_path);
};
#endif
