#ifndef MAP_GENERATOR_HPP
#define MAP_GENERATOR_HPP
#include <ios>
#include <string>
#include <thread>
#include <fstream>

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

    private:
        int argc;
        char **argv;
        bool is_record_data;
        bool *runMainThread;
        string record_path;
        Point cur_pos;
        Mat cur_map;
        Position pos;

        void initFromConfig(string config_path);
};
#endif