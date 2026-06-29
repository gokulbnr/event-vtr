#include <xcorr.h>
// #include <evk4.h>

#include <thread>


int main (int argc, char *argv[]) {

    // Initialize ROS node
    ros::init(argc, argv, "teach_repeat_node");
    ros::NodeHandle nh;

    // Parse command line arguments
    // Usage: xcorr [--mode time|count] [--events-per-frame N] [--accumulation-time-us T]
    //             [--sliding] [--hop-events N]
    std::string frame_mode = "time";  // Default to time-interval mode
    uint32_t events_per_frame = 10000;  // Default event count
    uint32_t accumulation_time_us = 66000;  // Default accumulation time
    bool use_sliding = false;  // Sliding window mode disabled by default
    uint32_t hop_events = 0;  // Hop size for sliding mode (0 = use events_per_frame)
    
    // Parse all arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--mode" && i + 1 < argc) {
            frame_mode = argv[++i];
        } else if (arg == "--events-per-frame" && i + 1 < argc) {
            events_per_frame = std::stoul(argv[++i]);
        } else if (arg == "--accumulation-time-us" && i + 1 < argc) {
            accumulation_time_us = std::stoul(argv[++i]);
        } else if (arg == "--sliding") {
            use_sliding = true;
        } else if (arg == "--hop-events" && i + 1 < argc) {
            hop_events = std::stoul(argv[++i]);
        }
    }
    
    // Validate frame mode
    if (frame_mode != "time" && frame_mode != "count") {
        ROS_WARN("Invalid frame mode: %s. Using 'time' mode.", frame_mode.c_str());
        frame_mode = "time";
    }
    
    ROS_INFO("Frame generation mode: %s", frame_mode.c_str());
    if (frame_mode == "count") {
        ROS_INFO("Events per frame: %u", events_per_frame);
        if (use_sliding) {
            ROS_INFO("Sliding mode enabled, hop events: %u", (hop_events > 0 ? hop_events : events_per_frame));
        }
    } else {
        ROS_INFO("Accumulation time: %u us", accumulation_time_us);
    }

    // Create an instance of TeachRepeat
    TeachRepeat teach_repeat(nh);
    
    // Configure frame generation parameters
    teach_repeat.frame_mode_ = frame_mode;
    teach_repeat.events_per_frame_ = events_per_frame;
    teach_repeat.use_sliding_ = use_sliding;
    teach_repeat.hop_events_ = hop_events;
    teach_repeat.accumulation_time_us_ = accumulation_time_us;
    
    teach_repeat.start();

    // Start the ROS event loop
    ros::spin();

    return 0;
}