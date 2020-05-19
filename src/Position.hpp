#ifndef POSITION_HPP
#define POSITION_HPP
#include <deque>
#include <string>
#include <ros/ros.h>  // 引用 ros.h 檔
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;
using namespace cv;

class Position {
    private:
        string map_filepath;
        int width;
        int height;
        int bias_x;
        int bias_y;
        int road_thickness;
        int path_thickness;
        int pos_radius;
        int scale_ratio;
        int show_freq;
        int counter;
        int pos_x;
        int pos_y;
        bool is_show_map;
        bool is_draw_path;
        bool is_save_map;
	bool is_use_zed2;
        ros::Subscriber sub;
        Mat map_calculate;
        Mat map_display;
        Point previous_point;
        Scalar bg_color;
        Scalar road_color;
        Scalar path_color;
        Scalar pos_color;
        deque<Point> pre_points;

	void zed2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void vinsfusionCallback(const nav_msgs::Path::ConstPtr& msg);
	void updatePosition(float raw_x, float raw_y);
        // void pathCallback(const nav_msgs::Path::ConstPtr& msg);
        void drawMap();
        void checkSize();
        void resizeMap(int increase_x, int increase_y);

    public:
        Position();
        void initMap(string map_filepath, int width, int height, int bias_x, int bias_y, int scale_ratio, int show_freq, bool is_show_map, bool is_draw_path, bool is_save_map, bool is_use_zed2);
        void saveMap();
        void loadMap();
        void initSubscribe(int argc, char** argv);
        Point getPosition();
        Mat getMap();
};

#endif
