#include "MapGenerator.hpp"

MapGenerator::MapGenerator(int argc, char** argv, bool *runMainThread) {
    this->argc = argc;
    this->argv = argv;
    this->runMainThread = runMainThread;
    this->show_freq = 100; // ms
    this->record_path = "";
    this->is_record_data = false;
    this->use_thread = false;
    this->show_map_in_thread = true;
}

void MapGenerator::initFromConfig(string config_path) {
    ConfigFile cf(config_path);

    string map_path   = cf.Value("Position", "map_path");
    string rec_path   = cf.Value("Position", "record_path");
    int width         = cf.Value("Position", "width");
    int height        = cf.Value("Position", "height");
    int bias_x        = cf.Value("Position", "bias_x");
    int bias_y        = cf.Value("Position", "bias_y");
    int scale_ratio   = cf.Value("Position", "scale_ratio");
    int freq          = cf.Value("Position", "show_freq");
    bool is_show_map  = cf.Value("Position", "is_show_map");
    bool is_draw_path = cf.Value("Position", "is_draw_path");
    bool is_save_map  = cf.Value("Position", "is_save_map");
    bool is_rec            = cf.Value("MapGenerator", "is_record_data");
    bool u_thread          = cf.Value("MapGenerator", "use_thread");
    bool show_map_thread   = cf.Value("MapGenerator", "show_map_in_thread");
    this->show_freq = freq;
    this->record_path = rec_path;
    this->is_record_data = is_rec;
    this->use_thread = u_thread;
    this->show_map_in_thread = show_map_thread;
    if (this->use_thread)
        is_show_map = false;

    this->pos.initMap(map_path, width, height, bias_x, bias_y, scale_ratio, this->show_freq, is_show_map, is_draw_path, is_save_map);
}

int MapGenerator::initialize() {
    this->pos.initSubscribe(argc, argv);
    if (this->argc == 1) {
        this->pos.initMap("map1.png", 1024, 512, 0, 0, 75, this->show_freq, true, true, false);
    }
    else if (this->argc == 2) {
        initFromConfig(this->argv[1]);
    }
    else {
        cout << "[ERROR] Please check your parameter" << endl;
        cout << "        It should be 1). Default setting without any parameter" << endl;
        cout << "                     2). Custom setting with only 1 parameter as its config file path" << endl;
        return 1;
    }
    return 0;
}

Point MapGenerator::getPosition() {
    return this->cur_pos;
}

Mat MapGenerator::getMap() {
    return this->cur_map;
}

void MapGenerator::run() {
    int counter;
    std::ofstream log;
    if (this->is_record_data) {
        counter = 0;
        log = std::ofstream(this->record_path + "/logfile.txt", std::ios_base::app | std::ios_base::out);
    }

    ros::Rate loop_rate(1000 / this->show_freq); // Hz
    while (ros::ok() && *(this->runMainThread)) {
        this->cur_pos = this->pos.getPosition();
        this->cur_map = this->pos.getMap();
        if (this->is_record_data) {
            cout << this->record_path + "/images/" + to_string(counter) + ".png" << endl;
            log << cur_pos.x << " " << cur_pos.y << "\n"; 
            imwrite(this->record_path + "/images/" + to_string(counter) + ".png", cur_map);
            counter += 1;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
