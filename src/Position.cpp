#include "Position.hpp"
#include <cmath>
#include <fstream>
#include <algorithm>

Position::Position() {
    this->counter = 0;
    this->road_thickness = 15;
    this->path_thickness = 2;
    this->pos_radius = 3;
    this->bg_color = Scalar(0, 0, 0);
    this->road_color = Scalar(200, 200, 200);
    this->path_color = Scalar(20, 255, 20);
    this->pos_color = Scalar(0, 0, 255);
}

void Position::initMap(string map_filepath, int width, int height, int bias_x, int bias_y, int scale_ratio, int show_freq, bool is_show_map, bool is_draw_path, bool is_save_map, bool is_use_zed2) {
    this->map_filepath = map_filepath;
    this->width = width;
    this->height = height;
    this->bias_x = width / 2 + bias_x;
    this->bias_y = height / 2 - bias_y;
    this->scale_ratio = scale_ratio;
    this->show_freq = show_freq;
    this->is_show_map = is_show_map;
    this->is_draw_path = is_draw_path;
    this->is_save_map = is_save_map;
    this->is_use_zed2 = is_use_zed2;
    this->pos_x = 0 + this->bias_x;
    this->pos_y = this->bias_y - 0;
    this->loadMap();
    if (this->is_show_map)
        namedWindow("Map");
}

void Position::saveMap() {
    imwrite(this->map_filepath, this->map_calculate);
}

void Position::loadMap() {
    if (FILE *file = fopen(this->map_filepath.c_str(), "r")) {
        fclose(file);
        this->map_calculate = imread(this->map_filepath, IMREAD_COLOR);
        if (this->map_calculate.rows != this->height || this->map_calculate.cols != this->width) {
            cout << "[WARN] Image shape is unexpected!" << endl;
            cout << "       Expected shape(w, h): " << this->width << " ," << this->height << endl;
            cout << "       Actually shape(w, h): " << this->map_calculate.cols << " ," << this->map_calculate.rows << endl;
            cout << "       Reset image with actually shape." << endl;
            if (this->is_save_map) {
                cout << "[INFO] Use new shape setting & repalce the old file." << endl;
                this->map_calculate = Mat(this->height, this->width, CV_8UC3, this->bg_color);
                this->saveMap();
            }
            else {
                this->width = this->map_calculate.cols;
                this->height = this->map_calculate.rows;
            }
        }
    }
    else {
        cout << "[INFO] File doesn't exist. Create a new file." << endl;
        this->map_calculate = Mat(this->height, this->width, CV_8UC3, this->bg_color);
        this->saveMap();
    }
    if (this->is_show_map)
        this->map_calculate.copyTo(this->map_display);
}

void Position::checkSize() {
    int bleed = 50;
    int increase_x = 0;
    int increase_y = 0;
    if (this->pos_x > this->width - bleed) // increase right (+)
        increase_x = this->pos_x - this->width + bleed;
    if (this->pos_x < bleed) // increase left (-)
        increase_x = this->pos_x - bleed;
    if (this->pos_y > this->height - bleed) // increase down (+)
        increase_y = this->pos_y - this->height + bleed;
    if (this->pos_y < bleed) // increase up (-)
        increase_y = this->pos_y - bleed;
    if (increase_x != 0 || increase_y != 0)
        this->resizeMap(increase_x, increase_y);
}

void Position::resizeMap(int increase_x, int increase_y) {
    this->width += abs(increase_x);
    this->height += abs(increase_y);
    Mat new_map = Mat(this->height, this->width, CV_8UC3, this->bg_color);
    
    int roi_x = (increase_x > 0) ? 0 : abs(increase_x);
    int roi_y = (increase_y > 0) ? 0 : abs(increase_y);
    Mat roi = new_map(Rect(roi_x, roi_y, this->map_calculate.cols, this->map_calculate.rows));
    Mat mask;
    cvtColor(this->map_calculate, mask, cv::COLOR_BGR2GRAY);
    this->map_calculate.copyTo(roi, mask);
    this->map_calculate = new_map;
    new_map.copyTo(this->map_display);

    // Update point info & bias info
    Point adjust_pt = Point(0, 0);
    if (increase_x < 0) {
        this->pos_x += abs(increase_x);
        this->bias_x += abs(increase_x);
        adjust_pt.x = abs(increase_x);
    }
    if (increase_y < 0) {
        this->pos_y += abs(increase_y);
        this->bias_y += abs(increase_y);
        adjust_pt.y = abs(increase_y);
    }
    for (int i = 0; i < pre_points.size(); i++) {
        pre_points[i] += adjust_pt;
    }
}

void Position::drawMap() {
    this->checkSize();
    Point new_point = Point(this->pos_x, this->pos_y);

    if (this->pre_points.size() == 0) {
        this->previous_point = new_point;
        this->pre_points.push_back(new_point);
    }
    // Draw new road on calculate map
    line(this->map_calculate, this->previous_point, new_point, 
         this->road_color, this->road_thickness);
    // Draw new road on display map
    if (this->is_show_map) {
        this->map_calculate.copyTo(this->map_display);
        line(this->map_display, this->previous_point, new_point, 
             this->road_color, this->road_thickness);
    }

    // Update previous points
    this->previous_point = new_point;

    if (this->is_show_map) {
        // Draw path
        if (this->is_draw_path) {
            this->pre_points.push_back(new_point);

            // remove old point and old path
            if (this->pre_points.size() > 10){
                line(this->map_display, pre_points[0], pre_points[1], 
                     this->road_color, this->path_thickness);
                this->pre_points.pop_front();
            }

            // Draw path from previous points
            for (int i = 1; i < pre_points.size(); i++) {
                line(this->map_display, pre_points[i-1], pre_points[i], 
                     this->path_color, this->path_thickness);
            }
        }

        // Draw current position
        circle(this->map_display, new_point, this->pos_radius, this->pos_color, FILLED);
    }
}

void Position::initSubscribe(int argc, char** argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle handler;

    if (ros::ok())
        ROS_INFO("ROS system run successfully!");
    else {
        cout << "[ERROR] ROS system run failed!!!!!" << endl;
        return;
    }
    cout << "STATUS: " << this->is_use_zed2 << endl;
    if (this->is_use_zed2){
	this->sub = handler.subscribe("/zed2/zed_node/pose", 10, &Position::zed2Callback, this);
	cout << "USE_ZED2" << endl;
    }
    else {
        this->sub = handler.subscribe("/loop_fusion/pose_graph_path", 10, &Position::vinsfusionCallback, this);
	cout << "USE_VINS_FUSION" << endl;
    }
}

void Position::zed2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    double raw_x = msg->pose.position.x;
    double raw_y = msg->pose.position.y;
    cout << "x:" << raw_x << " " << "y:" << raw_y << endl;
    this->updatePosition(raw_x, raw_y);
}

void Position::vinsfusionCallback(const nav_msgs::Path::ConstPtr& msg) {
    double raw_x = msg->poses.back().pose.position.x;
    double raw_y = msg->poses.back().pose.position.y;
    this->updatePosition(raw_x, raw_y);
}

void Position::updatePosition(float raw_x, float raw_y) {
    // double raw_x = msg->poses.back().pose.position.x;
    // double raw_y = msg->poses.back().pose.position.y;
    // Get appropriate position value
    this->pos_x = round(raw_x * this->scale_ratio) + this->bias_x;
    this->pos_y = this->bias_y - round(raw_y * this->scale_ratio);
    this->drawMap();
        // Put text on display map
        string raw_text = "Raw: (" + to_string(raw_x) + ", " + to_string(raw_y) + ")";
        string pos_text = "Position: (" + to_string(this->pos_x) + ", " + to_string(this->pos_y) + ")";
        putText(this->map_display, //target image
                raw_text, //text
                cv::Point(10, 20), //top-left position
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                CV_RGB(118, 185, 0), //font color
                2);
        putText(this->map_display, //target image
                pos_text, //text
                cv::Point(10, 60), //top-left position
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                CV_RGB(118, 185, 0), //font color
                2);
    // Save map once every 5 second.
    if (this->is_save_map && ++this->counter % (5 * 1000 / this->show_freq) == 0){
        this->saveMap();
        cout << "[INFO] Saving map." << endl;
    }
    ROS_INFO("Position-> x:[%f] y:[%f]", raw_x, raw_y);
}

Point Position::getPosition() {
    //cout << "[INFO] " << this->pos_x << " " << this->pos_y << endl;
    return Point(this->pos_x, this->pos_y);
}

Mat Position::getMap() {
    if (this->is_show_map) {
        imshow("Map", this->map_display);
        waitKey(this->show_freq);
    }
    return this->map_calculate;
}
