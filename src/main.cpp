#include <thread>
#include <signal.h>

#include "MapGenerator.hpp"

bool runMainThread = true;

void main_recv_sigint(int sig) {
    cout << "main_recv_sigint: Executed\n" << endl;
    runMainThread = false;
    return;
}

int main(int argc, char** argv) {
    MapGenerator map_generator(argc, argv, &runMainThread);
    int status = map_generator.initialize();
    map_generator.publisherInitialize();
    if (status)
        return status;

    if (map_generator.use_thread) {
        std::thread pos_th (&MapGenerator::run, &map_generator);
        signal(SIGINT, main_recv_sigint);

        if (map_generator.show_map_in_thread)
            namedWindow( "Display window", WINDOW_AUTOSIZE ); // Create a window for display.

        // -------------------------------------------------------------------------------------
        // Where you can add other code below

        Point a;
        Mat b;
        while (runMainThread) {
            a = map_generator.getPosition();
            map_generator.getMap().copyTo(b);
            if (map_generator.show_map_in_thread) {
                circle(b, a, 3, Scalar(0, 0, 255), FILLED);
                imshow( "Display window", b);                // Show our image inside it.
            }
            waitKey(map_generator.show_freq);
        }

        // Where you can add other code above
        // -------------------------------------------------------------------------------------

        cout << "Waiting for MapGenerator() to exit" << endl;
        pos_th.join(); 
        cout << "Wait for MapGenerator() exit done" << endl;
    }
    else {
        map_generator.run();
    }

    return 0;
}
