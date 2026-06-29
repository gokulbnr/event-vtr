#include <iostream>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <metavision/sdk/core/utils/rate_estimator.h>


#include <xcorr.h>

class EVK4 {
public:
    EVK4(std::string, std::string);
    ~EVK4();
    Metavision::Camera& getCamera();

private:
    std::string config_file_path_;
    std::string event_file_path_;

    Metavision::Camera camera_;
    int cd_events_cb_id_;
    std::unique_ptr<Metavision::CDFrameGenerator> cd_frame_generator_;
    int accumulation_time_us_ = 66000;
    int frames_per_second_ = 1000;
    int flag = 0;
};

EVK4::EVK4(std::string config_file_path, std::string event_file_path) : config_file_path_(config_file_path), event_file_path_(event_file_path) {
    if (!event_file_path_.empty()) {
        camera_ = Metavision::Camera::from_file(event_file_path_);
    } else {
        camera_ = Metavision::Camera::from_first_available();
        if (!config_file_path_.empty()) {
            camera_.load(config_file_path_);
        }
    }

    const Metavision::I_Geometry &geometry_ = camera_.geometry();

    // Setup CD event rate estimator
    double avg_rate, peak_rate;
    Metavision::RateEstimator cd_rate_estimator(
        [&avg_rate, &peak_rate](Metavision::timestamp ts, double arate, double prate) {
            avg_rate  = arate;
            peak_rate = prate;
        },
        100000, 1000000, true);
    
    // Setup CD frame generator
    std::mutex cd_frame_generator_mutex;
    cd_frame_generator_ = std::make_unique<Metavision::CDFrameGenerator>(geometry_.get_width(), geometry_.get_height());
    cd_frame_generator_->set_display_accumulation_time_us(accumulation_time_us_);

    // setup frame receive callback
    std::mutex cd_frame_mutex;
    cv::Mat cd_frame;
    Metavision::timestamp cd_frame_ts{0};
    cd_frame_generator_->start(
        frames_per_second_, [this, &cd_frame_mutex, &cd_frame, &cd_frame_ts](const Metavision::timestamp &ts, const cv::Mat &frame) {
            std::unique_lock<std::mutex> lock(cd_frame_mutex);
            cd_frame_ts = ts;
            std::cout << "CD Frame received at timestamp: " << ts << std::endl;
            frame.copyTo(cd_frame);   // Potential spot to trigger an visual xCorr call
            flag = 1;
        });

    cd_events_cb_id_ = camera_.cd().add_callback(
    [this, &cd_frame_generator_mutex, &cd_rate_estimator](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        std::unique_lock<std::mutex> lock(cd_frame_generator_mutex);
        cd_frame_generator_->add_events(ev_begin, ev_end);
        cd_rate_estimator.add_data(std::prev(ev_end)->t, std::distance(ev_begin, ev_end));
    });

    // Start the camera streaming
    // camera_.start();

    // while (camera_.is_running()) {
    // }
}

Metavision::Camera& EVK4::getCamera() {
    return camera_;
}

EVK4::~EVK4() {
    // unregister callbacks to make sure they are not called anymore
    if (cd_events_cb_id_ >= 0) camera_.cd().remove_callback(cd_events_cb_id_);

    // Stop the camera
    camera_.stop();
    cd_frame_generator_->stop();
}
