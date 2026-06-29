#include <iomanip>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <metavision/sdk/base/utils/log.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/sdk/stream/camera.h>
// #include <metavision/sdk/core/algorithms/on_demand_frame_generation_algorithm.h>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>

#include <SDL2/SDL.h>
// #include <metavision/sdk/core/algorithms/event_buffer_reslicer_algorithm.h>

std::string package_path = ros::package::getPath("cam_pub");

bool init_SDL(SDL_Window*& window, SDL_Renderer*& renderer, SDL_Texture*& texture, int width, int height) {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        std::cerr << "SDL could not initialize. SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    window = SDL_CreateWindow("CD Events",
                              SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED,
                              width, height,
                              SDL_WINDOW_SHOWN);
    if (!window) {
        std::cerr << "Window could not be created. SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        std::cerr << "Renderer could not be created. SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    texture = SDL_CreateTexture(renderer,
                                SDL_PIXELFORMAT_RGB24,
                                SDL_TEXTUREACCESS_STREAMING,
                                width, height);
    if (!texture) {
        std::cerr << "Texture could not be created. SDL_Error: " << SDL_GetError() << "\n";
        return false;
    }

    return true;
}

class FramerateEstimator {
public:
    FramerateEstimator(int window_size = 1000) : window_size_(window_size), frames_count_(0), frame_times_index_(0) {
        frame_times_.resize(window_size_);
        last_update_ = std::chrono::high_resolution_clock::now();
    }

    void update() {
        auto now = std::chrono::high_resolution_clock::now();
        frame_times_[frame_times_index_] = now;
        frame_times_index_ = (frame_times_index_ + 1) % window_size_;
        frames_count_++;
        last_update_ = now;
    }

    float getFPS() {
        if (frames_count_ < window_size_) {
            return 0.0f;
        }
        
        auto now = std::chrono::high_resolution_clock::now();
        auto oldest = frame_times_[(frame_times_index_) % window_size_];
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - oldest).count();
        
        // Avoid division by zero
        if (duration == 0) {
            return 0.0f;
        }
        
        return (window_size_ - 1) * 1000000.0f / duration;
    }

private:
    std::vector<std::chrono::high_resolution_clock::time_point> frame_times_;
    int window_size_;
    size_t frames_count_;
    int frame_times_index_;
    std::chrono::high_resolution_clock::time_point last_update_;
};

FramerateEstimator framerate_estimator;

int flag = 0;
std::atomic<bool> should_exit{false};

void signal_handler(int signal) {
    if (signal == SIGINT) {
        MV_LOG_INFO() << "\nReceived SIGINT (Ctrl+C). Shutting down cleanly...";
        should_exit = true;
    }
}

int main(int argc, char *argv[]) {
    // Register signal handler for Ctrl+C
    std::signal(SIGINT, signal_handler);

    ros::init(argc, argv, "raw_image_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1);

    std::string event_file_path;
    std::string frame_mode = "time";  // Default to time-interval mode

    int prev_time = 0;

    // Parse command line arguments
    // Usage: evk4 [event_file_path] [--mode time|count] [--events-per-frame N] [--accumulation-time-us T]
    //             [--sliding] [--hop-events N]
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
        } else if (arg[0] != '-') {
            // Treat non-option arguments as file path (first one only)
            if (event_file_path.empty()) {
                event_file_path = arg;
            }
        }
    }
    
    // Validate frame mode
    if (frame_mode != "time" && frame_mode != "count") {
        MV_LOG_WARNING() << "Invalid frame mode: " << frame_mode << ". Using 'time' mode.";
        frame_mode = "time";
    }
    
    MV_LOG_INFO() << "Frame generation mode: " << frame_mode;
    if (frame_mode == "count") {
        MV_LOG_INFO() << "Events per frame: " << events_per_frame;
        if (use_sliding) {
            MV_LOG_INFO() << "Sliding mode enabled, hop events: " << (hop_events > 0 ? hop_events : events_per_frame);
        }
    } else {
        MV_LOG_INFO() << "Accumulation time: " << accumulation_time_us << " us";
    }

    // std::string in_cam_config_path = "/home/gokulbnr/cam_ws/src/cam-pub/config/settings.json";
    std::string in_cam_config_path = package_path + "/config/settings.json";

    while (!should_exit) {

        Metavision::Camera camera;
        // If the filename is set, then read from the file
        if (!event_file_path.empty()) {
                camera = Metavision::Camera::from_file(event_file_path);
        } else {
                camera = Metavision::Camera::from_first_available();
                if (!in_cam_config_path.empty()) {
                    camera.load(in_cam_config_path);
                }
        }

        // Get the geometry of the camera
        const Metavision::I_Geometry &geometry = camera.geometry();


        // Setup CD frame generator
        std::mutex cd_frame_generator_mutex;
        Metavision::CDFrameGenerator cd_frame_generator(geometry.get_width(), geometry.get_height());
        
        // Configure frame generation mode
        if (frame_mode == "count") {
            if (use_sliding) {
                // Use sliding window mode with overlap
                cd_frame_generator.set_sliding_event_count_mode(events_per_frame, hop_events);
                MV_LOG_INFO() << "Using sliding event-count based frame generation";
            } else {
                cd_frame_generator.set_event_count_mode(events_per_frame);
                MV_LOG_INFO() << "Using event-count based frame generation";
            }
        } else {
            cd_frame_generator.set_display_accumulation_time_us(accumulation_time_us);
            MV_LOG_INFO() << "Using time-interval based frame generation";
        }

        // Metavision::EventBufferReslicerAlgorithm reslicer(
        // nullptr, Metavision::EventBufferReslicerAlgorithm::Condition::make_n_us(1e6/1500));

        std::mutex cd_frame_mutex;
        cv::Mat cd_frame; // = cv::Mat::ones(geometry.get_height(), geometry.get_width(), CV_8UC3) * 255;
        Metavision::timestamp cd_frame_ts{0};

        // Event timestamp accumulation (for events added since last generated frame)
        std::mutex event_ts_mutex;
        Metavision::timestamp accum_first_event_ts{0};
        Metavision::timestamp accum_last_event_ts{0};

        // Last frame's first/last event timestamps (copied from accum when a frame is generated)
        Metavision::timestamp last_frame_first_event_ts{0};
        Metavision::timestamp last_frame_last_event_ts{0};

        // Latency measurement: time from first event received to frame generation
        std::mutex latency_mutex;
        std::chrono::high_resolution_clock::time_point first_event_receive_time{};
        std::chrono::high_resolution_clock::time_point frame_generation_time{};
        double last_latency_ms{0.0};
        cd_frame_generator.start(
            100, [&cd_frame_mutex, &cd_frame, &cd_frame_ts,
                 &event_ts_mutex, &accum_first_event_ts, &accum_last_event_ts,
                 &last_frame_first_event_ts, &last_frame_last_event_ts,
                 &latency_mutex, &first_event_receive_time, &frame_generation_time, &last_latency_ms]
                (const Metavision::timestamp &ts, const cv::Mat &frame) {
                std::unique_lock<std::mutex> lock(cd_frame_mutex);
                cd_frame_ts = ts;
                cd_frame = frame;
                framerate_estimator.update();

                // Record frame generation time and calculate latency
                {
                    std::unique_lock<std::mutex> lat_lock(latency_mutex);
                    frame_generation_time = std::chrono::high_resolution_clock::now();
                    if (first_event_receive_time.time_since_epoch().count() > 0) {
                        last_latency_ms = std::chrono::duration<double, std::milli>(
                            frame_generation_time - first_event_receive_time
                        ).count();
                    }
                }

                // Atomically copy accumulated event timestamps into the "last frame" slots
                {
                    std::unique_lock<std::mutex> ev_lock(event_ts_mutex);
                    last_frame_first_event_ts = accum_first_event_ts;
                    last_frame_last_event_ts = accum_last_event_ts;
                    // reset accumulators for the next frame
                    accum_first_event_ts = 0;
                    accum_last_event_ts = 0;
                }
                
                // Reset event receive time for next frame
                {
                    std::unique_lock<std::mutex> lat_lock(latency_mutex);
                    first_event_receive_time = std::chrono::high_resolution_clock::time_point{};
                }
                flag = 1;
            });

        // SDL_Window* window = nullptr;
        // SDL_Renderer* renderer = nullptr;
        // SDL_Texture* texture = nullptr;
        // if (!init_SDL(window, renderer, texture, geometry.get_width(), geometry.get_height())) {
        //     return -1;
        // }

        int cd_events_cb_id =
            camera.cd().add_callback([&cd_frame_generator_mutex, &cd_frame_generator,
                                       &event_ts_mutex, &accum_first_event_ts, &accum_last_event_ts,
                                       &latency_mutex, &first_event_receive_time]
                                      (const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
                
                // Record time when first event is received (only on first event of new batch)
                {
                    std::unique_lock<std::mutex> lat_lock(latency_mutex);
                    if (first_event_receive_time.time_since_epoch().count() == 0) {
                        first_event_receive_time = std::chrono::high_resolution_clock::now();
                    }
                }

                std::unique_lock<std::mutex> lock(cd_frame_generator_mutex);
                cd_frame_generator.add_events(ev_begin, ev_end);

                // Update accumulated first/last event timestamps for events in this batch
                std::unique_lock<std::mutex> ev_lock(event_ts_mutex);
                for (const Metavision::EventCD *it = ev_begin; it != ev_end; ++it) {
                    Metavision::timestamp t = it->t;
                    if (accum_first_event_ts == 0 || t < accum_first_event_ts) {
                        accum_first_event_ts = t;
                    }
                    if (accum_last_event_ts == 0 || t > accum_last_event_ts) {
                        accum_last_event_ts = t;
                    }
                }
            });

        // Start the camera streaming
        try {
            camera.start();
        } catch (const Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
            if (e.code().value() == Metavision::CameraErrorCode::ConnectionError) {
                // do_retry = true;
                MV_LOG_INFO() << "Trying to reopen camera...";
                continue;
            }
        }

        SDL_Event event;

        while (camera.is_running() && !should_exit) {
            // Check for exit signal and break immediately
            if (should_exit) {
                MV_LOG_INFO() << "Exit signal received, stopping camera...";
                camera.stop();
                break;
            }

            if (flag == 1) {
                std::unique_lock<std::mutex> lock(cd_frame_mutex);
                if (!cd_frame.empty()) {

                // std::vector<cv::Mat> channels;
                // cv::Mat binary;
                // cv::split(cd_frame, channels); // channels[0] = Blue, channels[1] = Green, channels[2] = Red

                // cv::threshold(channels[0], binary, 150, 255, cv::THRESH_BINARY);


                // std::set<std::tuple<uchar, uchar, uchar>> uniquePixels;

                // // Iterate over all pixels
                // for (int row = 0; row < cd_frame.rows; ++row) {
                //     for (int col = 0; col < cd_frame.cols; ++col) {
                //         cv::Vec3b pixel = cd_frame.at<cv::Vec3b>(row, col); // BGR order
                //         uniquePixels.insert(std::make_tuple(pixel[0], pixel[1], pixel[2]));
                //     }
                // }

                // std::cout << "------------------------------------------\n";
                // for (const auto& p : uniquePixels) {
                //     std::cout << "("
                //         << static_cast<int>(std::get<0>(p)) << ", "
                //         << static_cast<int>(std::get<1>(p)) << ", "
                //         << static_cast<int>(std::get<2>(p)) << ")\n";
                // }
                // std::cout << "------------------------------------------\n";

                // cv::flip(binary, binary, -1);


                // ROS Handling
                sensor_msgs::Image msg;
                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "camera";
                msg.height = cd_frame.rows;
                msg.width = cd_frame.cols;
                msg.encoding = "bgr8";
                msg.is_bigendian = false;
                msg.step = cd_frame.cols * cd_frame.elemSize();  // bytes per row
                msg.data.assign(cd_frame.datastart, cd_frame.dataend);
                msg.header.stamp = ros::Time::now();

                pub.publish(msg);


                // SDL Handling
                    // int status = SDL_UpdateTexture(texture, NULL, cd_frame.data, cd_frame.step);

                    // if (status != 0) {
                    //     std::cerr << "SDL_UpdateTexture error: " << SDL_GetError() << std::endl;
                    // }

                    // SDL_RenderClear(renderer);
                    // SDL_RenderCopy(renderer, texture, NULL, NULL);
                    // SDL_RenderPresent(renderer);

                    // Print frame timestamp, fps, first/last event timestamps and their difference
                    Metavision::timestamp first_ts = last_frame_first_event_ts;
                    Metavision::timestamp last_ts = last_frame_last_event_ts;
                    
                    // Get latency
                    double latency_ms_local = 0.0;
                    {
                        std::unique_lock<std::mutex> lat_lock(latency_mutex);
                        latency_ms_local = last_latency_ms;
                    }
                    
                    if (first_ts != 0 && last_ts != 0 && last_ts >= first_ts) {
                        std::cout << cd_frame_ts << " " << framerate_estimator.getFPS()
                                  << " fps | first=" << first_ts << " last=" << last_ts
                                  << " diff=" << (last_ts - first_ts) << " | latency=" << std::fixed 
                                  << std::setprecision(3) << latency_ms_local << " ms\n";
                    } else {
                        std::cout << cd_frame_ts << " " << framerate_estimator.getFPS() << " fps | first=N/A last=N/A"
                                  << " | latency=" << std::fixed << std::setprecision(3) << latency_ms_local << " ms\n";
                    }

                    std::cout << "------------------------------------------" << cd_frame_ts -prev_time << "\n";
                    prev_time = cd_frame_ts;

                    flag = 0;
                }
            }
        }

        // Small sleep to reduce CPU usage and allow signal handler interruption
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT || (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)) {
                camera.stop();  // trigger camera stop to break the loop
            }
        }

        // }
        // while (SDL_PollEvent(&event)) {
        //     if (event.type == SDL_QUIT) {
        //         camera.stop();
        //         // do_retry = false;
        //     }
        // } 

        // unregister callbacks to make sure they are not called anymore
        // if (cd_events_cb_id >= 0) {
        //     camera.cd().remove_callback(cd_events_cb_id);
        // }

        // Stop the camera streaming, optional, the destructor will automatically do it
        try {
            camera.stop();
        } catch (const Metavision::CameraException &e) {
            MV_LOG_ERROR() << e.what();
        }
        cd_frame_generator.stop();
        
        if (should_exit) {
            MV_LOG_INFO() << "Camera loop ended. Cleaning up...";
            break;
        }
        
        MV_LOG_INFO() << "Trying to reopen camera...";
    }

    MV_LOG_INFO() << "Application shutdown complete.";
    return 0;
}