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
    this->is_publish = true;
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
    bool is_use_zed2  = cf.Value("Position", "is_use_zed2");
    cout << "CONFIG:" << is_use_zed2 << endl;
    bool is_rec            = cf.Value("MapGenerator", "is_record_data");
    bool u_thread          = cf.Value("MapGenerator", "use_thread");
    bool show_map_thread   = cf.Value("MapGenerator", "show_map_in_thread");
    bool is_publish        = cf.Value("MapGenerator", "is_publish");
    this->show_freq = freq;
    this->record_path = rec_path;
    this->is_record_data = is_rec;
    this->use_thread = u_thread;
    this->show_map_in_thread = show_map_thread;
    this->is_publish = is_publish;
    if (this->use_thread)
        is_show_map = false;

    this->pos.initMap(map_path, width, height, bias_x, bias_y, scale_ratio, this->show_freq, is_show_map, is_draw_path, is_save_map, is_use_zed2);
}

int MapGenerator::initialize() {
    //this->pos.initSubscribe(argc, argv);
    if (this->argc == 1) {
        this->pos.initMap("map1.png", 1024, 512, 0, 0, 75, this->show_freq, true, true, false, true);
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
    this->pos.initSubscribe(argc, argv);
    return 0;
}

Point MapGenerator::getPosition() {
    //if (this->is_publish)
    //this->publishPosition();
    //cout << "GET POINT" << endl;
    return this->cur_pos;
}

Mat MapGenerator::getMap() {
    //if (this->is_publish)
    //this->publishMap();
    //cout << "GET MAP" << endl;
    return this->cur_map;
}

void MapGenerator::publisherInitialize() {
    ros::init(argc, argv, "position_publisher");
    cout << "Publisher initialize successfully" << endl;
    ros::NodeHandle n;
    this->position_pub = n.advertise<geometry_msgs::Point>("position/point", 1);
    cout << "Create position/point topic successfully" << endl;
    image_transport::ImageTransport it(n);
    this->map_pub = it.advertise("position/map", 1);
    cout << "Create position/map topic successfully" << endl;
}

void MapGenerator::publishPosition() {
    geometry_msgs::Point pos_msg;
    pos_msg.x = this->cur_pos.x;
    pos_msg.y = this->cur_pos.y;
    pos_msg.z = 0;

    this->position_pub.publish(pos_msg);
    ros::spinOnce();
}

void MapGenerator::publishMap() {
    sensor_msgs::ImagePtr map_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", this->cur_map).toImageMsg();
    this->map_pub.publish(map_msg);
    ros::spinOnce();
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
	if (this->is_publish) {
	    this->publishPosition();
	    this->publishMap();
	}
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
