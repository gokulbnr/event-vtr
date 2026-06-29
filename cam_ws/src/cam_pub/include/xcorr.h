
#include <torch/torch.h>
#include <torch/fft.h>

#include <iostream>
#include <filesystem>
#include <vector>

#include <angles/angles.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <metavision/sdk/stream/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>

#include <utils.h>
#include <cam_pub/Goal.h>

#include <chrono>
#include <thread>

#include <SDL2/SDL.h>
#include <fstream>

#include <ctime>
#include <iomanip>
#include <sstream>

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


class TeachRepeat {
    public:
        void setupParameters();
        void setupSubscribers();
        void setupPublishers();
        TeachRepeat(ros::NodeHandle& nh);
        void start();

        // Frame generation parameters (public for command-line configuration)
        std::string frame_mode_;
        uint32_t events_per_frame_;
        bool use_sliding_;
        uint32_t hop_events_;
        int accumulation_time_us_;

    private:
        void processOdomData(const nav_msgs::Odometry::ConstPtr& msg);
        bool delta_frame_in_bounds(const tf::Transform& delta_frame);
        std::vector<tf::Transform> loadPoses(const std::vector<std::string>& pose_files);
        void makeNewGoal(double rotation_correction = 0.0, double path_correction = 1.0);
        GOAL_STATE updateGoalIndex();
        bool isTurningGoal(const tf::Transform& goal_frame, const tf::Transform& next_goal_frame);
        void updateGoal(const tf::Transform& goal_frame, bool new_goal = true, bool turning_goal = false);
        void publishGoal(const tf::Transform& pose, double lookahead_distance, bool stop_at_goal);
        void processImgData();
        std::tuple<std::vector<double>, std::vector<double>, cv::Mat> concat_fft_match_images_debug(
            const cv::Mat& image_raw, const cv::Mat& template_image_raw, int num_imgs);
        torch::Tensor fftconvolve_torch_cpp(const torch::Tensor& a, const torch::Tensor& b, const std::string& mode);
        void loadReferenceImages();

        std::string config_file_path_;
        std::string event_file_path_;

        Metavision::Camera camera_;
        int cd_events_cb_id_;
        std::unique_ptr<Metavision::CDFrameGenerator> cd_frame_generator_;
        int frames_per_second_;
        int flag = 0;

        ros::NodeHandle nh_;

        ros::Subscriber sub_odom_;

        ros::Publisher goal_pub_;

        bool first_goal_ = true;
        tf::Transform zero_odom_offset_;
        tf::Transform last_odom_pose_;
        bool last_odom_pose_received_ = false;
        std::mutex mutex_;
        bool running_;
        int goal_index_ = 0;
        tf::Transform  goal_;
        tf::Transform  last_goal_;
        tf::Transform  goal_plus_lookahead_;
        bool stop_at_end_;
        int goal_number_ = 0;

        double sum_path_correction_ = 0.0;
        double sum_theta_correction_ = 0.0;

        double LOOKAHEAD_DISTANCE_RATIO;
        double GOAL_DISTANCE_SPACING;
        double GOAL_THETA_TOLERANCE;
        double TURNING_TARGET_RANGE_DISTANCE_RATIO;

        std::string record_file_path;
        std::string load_dir_param;
        std::string load_dir;
        std::vector<tf::Transform> poses_;

        char* reference_imgs_filepath_ = "/home/qcr/gokul/teach-repeat-data/2025-08-15_16:31:54/full";
        std::vector<cv::Mat> reference_imgs_;
        int search_range_ = 3;
        double rotation_correction_gain_;
        double path_correction_gain_;
        double image_recognition_threshold_ = 0.1;
        double FIELD_OF_VIEW_RAD = 35.943 * M_PI / 180.0; // 35.943 degrees in radians

        double resize_factor;

        // For logging FPS
        std::vector<std::pair<double, double>> fps_log;
        std::chrono::high_resolution_clock::time_point logging_start_time = std::chrono::high_resolution_clock::now();
        int logging_interval_ms = 1000; // Log every 1000ms (1 second)
        std::vector<std::pair<double, double>> loop_log; // {timestamp, loop_duration_ms}
        std::vector<std::pair<double, double>> corrections_log; // {sum_theta_corrections, sum_path_corrections}
};

TeachRepeat::TeachRepeat(ros::NodeHandle& nh) : nh_(nh) {
    setupParameters();
    setupSubscribers();
    setupPublishers();
}


void TeachRepeat::setupParameters() {
    LOOKAHEAD_DISTANCE_RATIO = 1.3;
    GOAL_DISTANCE_SPACING = 0.3;
    GOAL_THETA_TOLERANCE = 15.0;
    TURNING_TARGET_RANGE_DISTANCE_RATIO = 0.2;

    goal_ = tf::Transform::getIdentity();
    last_goal_ = tf::Transform::getIdentity();
    goal_plus_lookahead_ = tf::Transform::getIdentity();
    stop_at_end_ = true;

    zero_odom_offset_ = tf::Transform::getIdentity();
    first_goal_ = true;

    record_file_path = "/home/qcr/gokul/recording.raw";
    // load_dir_param = "~/`/teach-repeat-data/2025-06-04_17:01:38";
    load_dir_param = "~/gokul/teach-repeat-data/2025-08-15_16:31:54";
    load_dir = expandUser(load_dir_param);
    if (load_dir.back() != '/') load_dir += "/";
    std::vector<std::string> pose_files;
    for (const auto& entry : std::filesystem::directory_iterator(load_dir)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if (filename.size() >= 9 && filename.substr(filename.size() - 9) == "_pose.txt") {
                pose_files.push_back(load_dir + filename);
            }
        }
    }
    std::sort(pose_files.begin(), pose_files.end());
    poses_ = loadPoses(pose_files);

    config_file_path_ = "/home/qcr/cam_ws/src/cam-pub/config/settings.json";
    event_file_path_ = "";  // Set to empty if you want to use the first available camera
    accumulation_time_us_ = 66000;
    frames_per_second_ = 500;

    frame_mode_ = "time";  // Default to time-interval mode
    events_per_frame_ = 10000;  // Default event count
    use_sliding_ = false;  // Sliding window mode disabled by default
    hop_events_ = 0;  // Hop size for sliding mode (0 = use events_per_frame)

    rotation_correction_gain_ = 0.0015;
    path_correction_gain_ = 0.000005;
    image_recognition_threshold_ = 0.1;
    FIELD_OF_VIEW_RAD = 35.943 * M_PI / 180.0;

    resize_factor = 0.25;

    loadReferenceImages();

    running_ = true;
}


void TeachRepeat::setupSubscribers() {
    sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("odom", 10, &TeachRepeat::processOdomData, this);
}


void TeachRepeat::setupPublishers() {
    goal_pub_ = nh_.advertise<cam_pub::Goal>("goal", 1);
}

std::vector<tf::Transform> TeachRepeat::loadPoses(const std::vector<std::string>& pose_files) {
    std::vector<tf::Transform> raw_poses;
    for (const auto& file : pose_files) {
        std::string content = readFile(file);
        nlohmann::json j = nlohmann::json::parse(content);
        raw_poses.push_back(jsonToPose(j));
    }

    std::vector<tf::Transform> poses;
    if (!raw_poses.empty()) {
        tf::Transform first_inv = raw_poses.front().inverse();
        for (const auto& pose : raw_poses) {
            poses.push_back(first_inv * pose);
        }
    }

    return poses;
}


bool TeachRepeat::delta_frame_in_bounds(const tf::Transform& delta_frame) {
    double distance = delta_frame.getOrigin().length();
    double roll, pitch, yaw;
    delta_frame.getBasis().getRPY(roll, pitch, yaw);
    double angle = angles::normalize_angle(yaw);  // requires <angles/angles.h>

    if (distance < (LOOKAHEAD_DISTANCE_RATIO * GOAL_DISTANCE_SPACING)) {
        if (std::abs(angle) < M_PI * GOAL_THETA_TOLERANCE / 180.0) {
            return true;
        } else {
            ROS_INFO("within goal distance, but theta offset not within tolerance = %.2f degrees [distance = %.3f]", angle * 180.0 / M_PI, distance);
        }
    }
    return false;
}


void TeachRepeat::processOdomData(const nav_msgs::Odometry::ConstPtr& msg) {
    if (running_) {
        // if (!last_image_.empty()) {
            std::lock_guard<std::mutex> lock(mutex_);

            // std::cout << "Processing odom data: " << msg->header.stamp << std::endl;

            if (first_goal_) {
                tf::poseMsgToTF(msg->pose.pose, zero_odom_offset_);
                first_goal_ = false;
            }

            nav_msgs::Odometry msg_copy = *msg;
            msg_copy = subtract_odom(msg_copy, zero_odom_offset_);

            last_odom_pose_ = tf::Transform();
            tf::poseMsgToTF(msg_copy.pose.pose, last_odom_pose_);

            tf::Transform current_pose_odom;
            tf::poseMsgToTF(msg_copy.pose.pose, current_pose_odom);
            tf::Transform current_goal_frame_odom = goal_;
            tf::Transform current_goal_plus_lookahead_frame_odom = goal_plus_lookahead_;
            tf::Transform old_goal_frame_world = poses_[goal_index_];

            // printTfTransform(current_pose_odom);
            // printTfTransform(current_goal_plus_lookahead_frame_odom);

            tf::Transform delta_frame = current_pose_odom.inverseTimes(current_goal_plus_lookahead_frame_odom);

            if (delta_frame_in_bounds(delta_frame))
                makeNewGoal();
        // }
    } else {
        std::lock_guard<std::mutex> lock(mutex_);
        tf::poseMsgToTF(msg->pose.pose, last_odom_pose_);
    }
    last_odom_pose_received_ = true;
}


GOAL_STATE TeachRepeat::updateGoalIndex() {
    goal_index_++;

    if (goal_index_ == poses_.size()) {
        if (stop_at_end_) {
            goal_index_ = poses_.size() - 1;
            // Save image when arriving at the final goal
            // if (goal_number_ < poses_.size()) {
            //     calculateImagePoseOffset(goal_index_);
            //     goal_number_++;
            // }
            return GOAL_STATE::finished;
        } else {
            goal_index_ = 0;  // Repeat the path (loop)
            return GOAL_STATE::restart;
        }
    }

    return GOAL_STATE::normal_goal;
}


bool TeachRepeat::isTurningGoal(const tf::Transform& goal_frame, const tf::Transform& next_goal_frame) {
    tf::Transform diff = goal_frame.inverse() * next_goal_frame;
    double dist = diff.getOrigin().length();
    return dist < (TURNING_TARGET_RANGE_DISTANCE_RATIO * GOAL_DISTANCE_SPACING);
}


void TeachRepeat::updateGoal(const tf::Transform& goal_frame, bool new_goal, bool turning_goal) {
    if (new_goal == true) {
        last_goal_ = goal_;
    }

    // tf::poseTFToMsg(goal_frame, goal_);
    goal_ = goal_frame;
    // normaliseQuaternion(goal_.orientation);

    // If goal is a turning goal or final goal, don't set a virtual waypoint ahead
    bool is_final_goal = (goal_index_ == poses_.size() - 1 && stop_at_end_);
    if (turning_goal || is_final_goal) {
        publishGoal(goal_, 0.0, true);
    } else {
        publishGoal(goal_, LOOKAHEAD_DISTANCE_RATIO * GOAL_DISTANCE_SPACING, false);
    }
}


void TeachRepeat::makeNewGoal(double rotation_correction, double path_correction) {
    int old_goal_index = goal_index_;
    tf::Transform old_goal_frame_world = poses_[old_goal_index];
    tf::Transform current_goal_frame_odom = goal_;
    // tf::poseMsgToTF(goal_, current_goal_frame_odom);

    GOAL_STATE state = updateGoalIndex();

    if (state == GOAL_STATE::finished) {
        std::cout << "Localiser stopping. Reached final goal." << std::endl;
        running_ = false;
        return;
    }

    if (state == GOAL_STATE::restart) {
        old_goal_frame_world.setIdentity();  // tf equivalent of tf_conversions.Frame()
    }

    tf::Transform new_goal_frame_world = poses_[goal_index_];
    bool turning_goal = isTurningGoal(old_goal_frame_world, new_goal_frame_world);

    tf::Transform goal_offset = getCorrectedGoalOffset(
        old_goal_frame_world, new_goal_frame_world,
        rotation_correction, path_correction
    );

    tf::Transform new_goal = current_goal_frame_odom * goal_offset;

    double sum_path_correction_ratio = 
        (GOAL_DISTANCE_SPACING + sum_path_correction_) / GOAL_DISTANCE_SPACING;

    updateGoal(new_goal, true, turning_goal);
    // saveDataAtGoal(last_odom_pose_, new_goal, new_goal_frame_world,
                //    sum_theta_correction_, sum_path_correction_ratio);

    goal_number_ += 1;

    // std::cout << "[" << old_goal_index << "] theta ["
    //           << rad2deg(sum_theta_correction_) << "]\tpath ["
    //           << sum_path_correction_ratio << "]" << std::endl;

    if (turning_goal) {
        std::cout << "turning goal:" << std::endl;
    }

    sum_theta_correction_ = 0;
    sum_path_correction_ = 0.0;
}


void TeachRepeat::publishGoal(const tf::Transform& pose, double lookahead_distance, bool stop_at_goal) {
    // Prepare Goal message
    cam_pub::Goal goal;
    goal.pose.header.stamp = ros::Time::now();
    goal.pose.header.frame_id = "odom";
    goal.pose.header.seq = goal_index_;

    // Lookahead offset in base frame
    tf::Vector3 lookahead_vec(lookahead_distance, 0.0, 0.0);
    tf::Transform lookahead(tf::createIdentityQuaternion(), lookahead_vec);

    // Convert input pose to tf::Transform
    tf::Transform original_pose_frame = pose;

    // Apply odom offset and lookahead
    tf::Transform pose_frame = zero_odom_offset_ * original_pose_frame;
    tf::Transform original_pose_frame_lookahead = original_pose_frame * lookahead;
    tf::Transform pose_frame_lookahead = pose_frame * lookahead;

    // Set the goal pose
    goal.pose.pose.position.x = pose_frame_lookahead.getOrigin().x();
    goal.pose.pose.position.y = pose_frame_lookahead.getOrigin().y();
    
    geometry_msgs::Quaternion quat_msg;
    tf::quaternionTFToMsg(pose_frame_lookahead.getRotation(), quat_msg);
    goal.pose.pose.orientation = quat_msg;

    // Set stop condition
    goal.stop_at_goal.data = stop_at_goal;

    // Publish goal
    goal_pub_.publish(goal);

    // Save goal + lookahead for potential debug/use
    goal_plus_lookahead_ = original_pose_frame_lookahead;
}


void TeachRepeat::start() {
    // Wait for first odom
    if (!last_odom_pose_received_) {
        ROS_INFO("Global localisation - waiting for first odom message");
        ros::Rate rate(5); // 0.2 sec = 5Hz
        while (!last_odom_pose_received_ && ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    tf::Transform goal_pose;
    tf::Transform odom_pose = last_odom_pose_;
    zero_odom_offset_ = odom_pose;

    // Check if first pose is (0,0,0)
    tf::Transform first_pose_tf = poses_[0];
    double roll, pitch, yaw;
    first_pose_tf.getBasis().getRPY(roll, pitch, yaw);

    if (first_pose_tf.getOrigin().x() == 0 &&
        first_pose_tf.getOrigin().y() == 0 &&
        std::abs(yaw) < 1e-4) {

        ROS_INFO("Localiser: starting at goal 1, goal 0 = [0,0,0]");
        goal_index_ = 1;
    } else {
        goal_index_ = 0;
    }

    tf::Transform goal_tf;
    goal_tf = poses_[goal_index_];
    updateGoal(goal_tf);

    running_ = true;

    std::thread img_thread(&TeachRepeat::processImgData, this);
    img_thread.detach();
}

FramerateEstimator framerate_estimator;

void TeachRepeat::processImgData() {
    if (!event_file_path_.empty()) {
        camera_ = Metavision::Camera::from_file(event_file_path_);
    } else {
        camera_ = Metavision::Camera::from_first_available();
        if (!config_file_path_.empty()) {
            camera_.load(config_file_path_);
        }
    }

    const Metavision::I_Geometry &geometry_ = camera_.geometry();

    // Setup CD frame generator
    std::mutex cd_frame_generator_mutex;
    cd_frame_generator_ = std::make_unique<Metavision::CDFrameGenerator>(geometry_.get_width(), geometry_.get_height(), false);
    
    // Configure frame generation mode
    if (frame_mode_ == "count") {
        if (use_sliding_) {
            // Use sliding window mode with overlap
            cd_frame_generator_->set_sliding_event_count_mode(events_per_frame_, hop_events_);
        } else {
            cd_frame_generator_->set_event_count_mode(events_per_frame_);
        }
    } else {
        cd_frame_generator_->set_display_accumulation_time_us(accumulation_time_us_);
    }

    // setup frame receive callback
    std::mutex cd_frame_mutex;
    cv::Mat cd_frame;
    Metavision::timestamp cd_frame_ts{0};
    cd_frame_generator_->start(
        frames_per_second_, [this, &cd_frame, &cd_frame_ts](const Metavision::timestamp &ts, const cv::Mat &frame) {
            std::unique_lock<std::mutex> lock(mutex_);
            cd_frame_ts = ts;
            cd_frame = frame;
            // frame.copyTo(cd_frame);   // Potential spot to trigger an visual xCorr call
            framerate_estimator.update();
            flag = 1;
        });

    cd_events_cb_id_ = camera_.cd().add_callback(
    [this, &cd_frame_generator_mutex](const Metavision::EventCD *ev_begin, const Metavision::EventCD *ev_end) {
        std::unique_lock<std::mutex> lock(cd_frame_generator_mutex);
        cd_frame_generator_->add_events(ev_begin, ev_end);
    });

    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    SDL_Texture* texture = nullptr;

    if (!init_SDL(window, renderer, texture, geometry_.get_width() * resize_factor, geometry_.get_height() * resize_factor * 2)) {
        return;
    }

    SDL_Event event;

    // Start the camera streaming
    camera_.start();
    camera_.start_recording(record_file_path);

    std::string dirname = "/home/qcr/gokul/repeat_out/";
    if (std::filesystem::create_directory(dirname)) {
        std::cout << "Directory created: " << dirname << std::endl;
    } else {
        std::cout << "Directory already exists: " << dirname << std::endl;
    }

    double rotation_correction_sum = 0;

    while (camera_.is_running() && running_) {
        if (flag == 1) {
            // std::cout << "CD Frame received at timestamp: " << cd_frame_ts << std::endl;

            if (last_odom_pose_received_ == true && goal_index_ > 0) {
                std::lock_guard<std::mutex> lock(mutex_);
                
                auto loop_start_time = std::chrono::high_resolution_clock::now();
                
                cv::Mat qry;
                {
                    std::lock_guard<std::mutex> lock(cd_frame_mutex);
                    qry = ImgProcessing(cd_frame, resize_factor);
                    flag = 0;
                }

                tf::Transform next_goal_world   = poses_[goal_index_];
                tf::Transform last_goal_world   = poses_[goal_index_ - 1];
                tf::Transform last_goal_odom    = last_goal_;
                tf::Transform next_goal_odom    = goal_;
                tf::Transform current_pose_odom = last_odom_pose_;

                tf::Transform next_goal_offset_odom  = next_goal_odom.inverse() * current_pose_odom;
                tf::Transform last_goal_offset_odom  = last_goal_odom.inverse() * current_pose_odom;
                tf::Transform inter_goal_offset_odom = last_goal_odom.inverse() * next_goal_odom;

                double last_goal_distance = last_goal_offset_odom.getOrigin().length();
                double next_goal_distance = next_goal_offset_odom.getOrigin().length();

                tf::Vector3 vec = inter_goal_offset_odom.getOrigin();
                std::vector<double> last_goal_to_next_goal_vector = {vec.x(), vec.y(), vec.z()};

                vec = last_goal_offset_odom.getOrigin();
                std::vector<double> last_goal_to_current_pose_vector = {vec.x(), vec.y(), vec.z()};

                double roll, pitch, yaw;
                last_goal_offset_odom.getBasis().getRPY(roll, pitch, yaw);
                double last_goal_angle = yaw;

                next_goal_offset_odom.getBasis().getRPY(roll, pitch, yaw);
                double next_goal_angle = yaw;

                bool turning_goal;
                double u;
                if (isTurningGoal(last_goal_world, next_goal_world)) {
                    turning_goal = true;
                    u = last_goal_angle / (last_goal_angle - next_goal_angle);
                }
                else {
                    turning_goal = false;
                    // projection: dot(a, b) / dot(a, a)
                    double dot_product = 0.0;
                    double norm_squared = 0.0;

                    for (size_t i = 0; i < last_goal_to_next_goal_vector.size(); ++i) {
                        dot_product += last_goal_to_next_goal_vector[i] * last_goal_to_current_pose_vector[i];
                        norm_squared += last_goal_to_next_goal_vector[i] * last_goal_to_next_goal_vector[i];
                    }
                    u = dot_product / norm_squared;
                }

                int start_range = std::max(0, goal_index_ - search_range_ - 1);
                int end_range = std::min(static_cast<int>(reference_imgs_.size()), goal_index_ + search_range_ + 2);
                std::vector<cv::Mat> subset(reference_imgs_.begin() + start_range, reference_imgs_.begin() + end_range);

                // qry = qry(cv::Rect(qry.cols / 4, 0, qry.cols / 2, qry.rows));

                int num_imgs = end_range - start_range;

                std::vector<double> offsets;
                std::vector<double> correlations;
                cv::Mat full_correlations;
                offsets.reserve(num_imgs);
                correlations.reserve(num_imgs);

                for (const auto& ref_img : subset) {
                    auto [img_offsets, img_correlations, img_full_corr] =
                        concat_fft_match_images_debug(ref_img, qry, 1);

                    if (!img_offsets.empty()) {
                        offsets.push_back(img_offsets[0]);
                    }
                    if (!img_correlations.empty()) {
                        correlations.push_back(img_correlations[0]);
                    }

                    if (full_correlations.empty()) {
                        full_correlations = img_full_corr;
                    } else {
                        cv::hconcat(full_correlations, img_full_corr, full_correlations);
                    }
                }

                int corr_width = subset.empty() ? 0 : (subset[0].cols / 8);

                cv::Mat probsImg;
                std::vector<cv::Mat> probsImgVector;
                for (int i = 0; i < num_imgs; ++i) {
                    int start = i * corr_width;
                    int end = start + corr_width/2;

                    cv::Mat ind_corr = full_correlations(cv::Rect(start, 0, corr_width/2, full_correlations.rows));

                    // Normalize ind_corr to 0-80
                    cv::normalize(ind_corr, ind_corr, 0, 80, cv::NORM_MINMAX);

                    int plotHeight = 80;
                    int prob_len = corr_width/2;
                    probsImg = cv::Mat::zeros(plotHeight, prob_len, CV_8UC1);
                    for (int x = 0; x < prob_len; x++) {
                        int h = static_cast<int>(ind_corr.at<float>(0, x));
                        cv::line(probsImg, cv::Point(x, plotHeight - 1),
                                        cv::Point(x, plotHeight - h), 255, 1);
                    }
                    cv::resize(probsImg, probsImg, cv::Size(prob_len*8, plotHeight), 0, 0, cv::INTER_NEAREST);
                    cv::cvtColor(probsImg, probsImg, cv::COLOR_GRAY2BGR);
                    probsImgVector.push_back(probsImg);
                }

                // Best match
                auto max_iter = std::max_element(correlations.begin(), correlations.end());
                int max_index = std::distance(correlations.begin(), max_iter);
                double best_offset = offsets[max_index];

                std::vector<double> rotation_offsets;
                std::vector<double> rotation_correlations;

                if (goal_index_ > search_range_) {
                    int start = search_range_;
                    int end = search_range_ + 2;

                    rotation_offsets = std::vector<double>(offsets.begin() + start, offsets.begin() + end);
                    rotation_correlations = std::vector<double>(correlations.begin() + start, correlations.begin() + end);
                } else {
                    int start = offsets.size() - search_range_ - 3;
                    int end = offsets.size() - search_range_ - 1;

                    rotation_offsets = std::vector<double>(offsets.begin() + start, offsets.begin() + end);
                    rotation_correlations = std::vector<double>(correlations.begin() + start, correlations.begin() + end);
                }

                double offset = (1-u) * rotation_offsets[0] + u * rotation_offsets[1];
                double rotation_correction = rotation_correction_gain_ * offset;

                double max_corr = *std::max_element(rotation_correlations.begin(), rotation_correlations.end());
                if (turning_goal || max_corr < image_recognition_threshold_ || u < 0 || u > 1) {
                    rotation_correction = 0.0;
                }

                // Remove negative values for correlations
                for (auto& val : correlations) {
                    if (val < 0)
                        val = 0;
                }

                // Normalize the correlations array such that, sum = 1
                double corr_sum = std::accumulate(correlations.begin(), correlations.end(), 0.0);
                if (corr_sum != 0) {
                    for (auto& val : correlations) {
                        val /= corr_sum;
                    }
                }

                std::vector<double> corr;
                std::vector<double> w;
                tf::Transform corrected_pose;
                double path_correction = 1.0;
                double path_correction_distance = 0.0;
                int n = static_cast<int>(poses_.size());
                if (goal_index_ > search_range_ && goal_index_ < n - search_range_) {
                    corr.assign(correlations.begin(), correlations.begin() + 2 * (2+search_range_));
                    w = arange(-0.5 - 1 - search_range_, 0.5 + 1 + search_range_, 1);

                    double corrected_index = 0.0;
                    for (size_t i = 0; i < corr.size(); i++) { 
                        corrected_index += (corr[i] * w[i]);
                    }

                    double index = start_range + 1 + search_range_ + corrected_index;

                    double path_error = index - goal_index_;
                    path_correction_distance = -1 * path_correction_gain_ * path_error * GOAL_DISTANCE_SPACING;
                    path_correction = (next_goal_distance + path_correction_distance) / next_goal_distance;

                    // std::cout << goal_index_ << " " << index << std::endl;

                    if ( (-1*path_correction_distance) > next_goal_distance ) {
                        makeNewGoal();
                        continue;
                    }
                }
                else {
                    path_correction = 1.0;
                    path_correction_distance = 0.0;
                }

                // rotation_correction = 0.0;                          // Disable rotation corrections for now. 
                // path_correction = 1.0;                                 // Disable path corrections for now. 
                // path_correction_distance = 0.0;
                
                sum_theta_correction_ += rotation_correction;
                sum_path_correction_ += path_correction_distance;

                tf::Transform goal_offset = getCorrectedGoalOffset(
                    current_pose_odom, next_goal_odom,
                    rotation_correction, path_correction
                );

                cv::Mat best_image = subset[max_index];
                best_image = best_image(cv::Rect(best_image.cols / 4, 0, best_image.cols / 2, best_image.rows));
                best_image.convertTo(best_image, CV_8UC1);
                qry.convertTo(qry, CV_8UC1);
                cv::flip(qry, qry, -1);

                // Visualization code
                cv::Mat debug_image = visualize(qry, best_image, (best_offset*best_image.cols)/FIELD_OF_VIEW_RAD + best_image.cols / 2);
                // std::cout << "offset: " << (best_offset*best_image.cols)/FIELD_OF_VIEW_RAD + best_image.cols / 2 << std::endl;
                probsImg = probsImgVector[max_index];
                cv::vconcat(debug_image, probsImg, debug_image);

                int status = SDL_UpdateTexture(texture, NULL, debug_image.data, debug_image.step);
                if (status != 0) {
                    std::cerr << "SDL_UpdateTexture error: " << SDL_GetError() << std::endl;    
                }

                SDL_RenderClear(renderer);
                SDL_RenderCopy(renderer, texture, NULL, NULL);
                SDL_RenderPresent(renderer);

                tf::Transform new_goal = current_pose_odom * goal_offset;
                updateGoal(new_goal, false, turning_goal);
                // std::cout << framerate_estimator.getFPS() << "fps\n";

                auto now = std::chrono::high_resolution_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - logging_start_time).count();

                auto loop_end_time = std::chrono::high_resolution_clock::now();
                double loop_duration_ms = std::chrono::duration<double, std::milli>(loop_end_time - loop_start_time).count();
                
                if (elapsed >= logging_interval_ms) {
                    double timestamp_s = std::chrono::duration<double>(now.time_since_epoch()).count();
                    double current_fps = framerate_estimator.getFPS();
                    fps_log.emplace_back(timestamp_s, current_fps);
                    loop_log.emplace_back(timestamp_s, loop_duration_ms);
                    corrections_log.emplace_back(sum_theta_correction_, sum_path_correction_);
                    logging_start_time = now; // reset timer

                    cv::imwrite(dirname + "debug_image_" + std::to_string(timestamp_s) + ".png", debug_image);
                }
            }
        }
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                camera_.stop();
            }
        }
    }

    std::cout << "Camera stopping." << std::endl;
    camera_.stop_recording(record_file_path);
    camera_.stop();

    // Get current time
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    // Format time into a string (e.g., 2025-07-08_14-33-55)
    std::ostringstream oss;
    oss << "/home/qcr/gokul/fps_log_" << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S") << ".csv";
    std::string filename = oss.str();

    std::ofstream outfile(filename);
    outfile << "timestamp,fps,latency,sum_theta_correction,sum_path_correction\n";
    size_t log_size = std::min(fps_log.size(), loop_log.size());
    for (size_t i = 0; i < log_size; ++i) {
        outfile << fps_log[i].first << "," << fps_log[i].second << "," << loop_log[i].second << "," << corrections_log[i].first << "," << corrections_log[i].second << "\n";
    }
    outfile.close();
    std::cout << "FPS log saved to fps_log.csv\n";

}


std::tuple<std::vector<double>, std::vector<double>, cv::Mat> TeachRepeat::concat_fft_match_images_debug(
    const cv::Mat& image_raw, const cv::Mat& template_image_raw, int num_imgs) {

    cv::Mat image = reduceWidthBySumming8(image_raw);
    int img_width = image.cols / num_imgs;

    cv::Mat qry_img = reduceWidthBySumming8(template_image_raw);
    cv::Mat template_image_flipped;
    cv::flip(template_image_raw, template_image_flipped, -1);

    torch::Tensor corr_tensor = fftconvolve_torch_cpp(
        torch::from_blob(image.data, {image.rows, image.cols}),
        torch::from_blob(qry_img.data, {qry_img.rows, qry_img.cols}),
        "valid"
    );

    cv::Mat corr(corr_tensor.size(0), corr_tensor.size(1), CV_32F, corr_tensor.data_ptr());
    corr = corr.colRange(0, corr.cols - 1);

    std::vector<double> cor_arr;
    std::vector<double> offset_arr;

    int img_height = image.rows;

    for (int i = 0; i < num_imgs; ++i) {
        int start = i * img_width + (img_width / 8);
        int end   = i * img_width + (img_width / 2) - (img_width / 8);

        cv::Mat cor = corr.colRange(start, end);

        cv::Point max_loc;
        cv::minMaxLoc(cor, nullptr, nullptr, nullptr, &max_loc);
        int max_y = max_loc.x;

        cor_arr.push_back(cor.at<double>(max_y));
        int offset = max_y + (img_width/8) - (img_width / 4);
        double offset_radians = (offset * FIELD_OF_VIEW_RAD * 2) / img_width;
        offset_arr.push_back(offset_radians);
    }

    return {offset_arr, cor_arr, corr};
}


// Function to perform cross-correlation between two images in frequency domain
torch::Tensor TeachRepeat::fftconvolve_torch_cpp(const torch::Tensor& a, const torch::Tensor& b, const std::string& mode = "same") {
    torch::Tensor result;    

    // Ensure tensors are float32 and on the same device
    auto a_f = a.detach().to(torch::kFloat32);
    auto b_f = b.detach().to(torch::kFloat32);

    auto a_shape = a.sizes();
    auto b_shape = b.sizes();

    std::vector<int64_t> out_size;
    if (mode == "full") {
        for (size_t i = 0; i < a.dim(); ++i) {
            out_size.push_back(a_shape[i] + b_shape[i] - 1);
        }
    } else if (mode == "same") {
        out_size = a_shape.vec();
    } else if (mode == "valid") {
        for (size_t i = 0; i < a.dim(); ++i) {
            out_size.push_back(a_shape[i] - b_shape[i] + 1);
        }
    } else {
        throw std::invalid_argument("mode must be 'full', 'same', or 'valid'");
    }

    // For simplicity, assume input shapes are same or compatible
    std::vector<int64_t> fft_size(a_shape.begin(), a_shape.end());

    // Perform FFT (use rfft2 if both a and b are real)
    auto a_fft = torch::fft::rfft2(a_f, fft_size);
    auto b_fft = torch::fft::rfft2(b_f, fft_size);

    // Multiply in frequency domain
    auto result_fft = a_fft * b_fft;

    // Inverse FFT
    result = torch::fft::irfft2(result_fft, fft_size);

    // Slicing for different modes
    if (mode == "same" || mode == "valid") {
        std::vector<torch::indexing::TensorIndex> slices;
        for (size_t i = 0; i < out_size.size(); ++i) {
            int64_t start, end;
            if (mode == "same") {
                start = (fft_size[i] - out_size[i]) / 2;
                end = start + out_size[i];
            } else { // valid
                start = fft_size[i] - out_size[i];
                end = start + out_size[i];
            }
            slices.push_back(torch::indexing::Slice(start, end));
        }
        result = result.index(slices);
    }
    return result;
}


void TeachRepeat::loadReferenceImages() {
    std::vector<std::filesystem::directory_entry> ref_entries;
    // Collect all regular files
    for (const auto& entry : std::filesystem::directory_iterator(reference_imgs_filepath_)) {
        if (entry.is_regular_file())
            ref_entries.push_back(entry);
    }
    // Sort files lexicographically by filename
    std::sort(ref_entries.begin(), ref_entries.end(),
              [](const std::filesystem::directory_entry& a, const std::filesystem::directory_entry& b) {
                  return a.path().filename().string() < b.path().filename().string();
              });
    for (const auto& entry : ref_entries) {
        std::string file_path = entry.path().string();
        std::vector<cv::Mat> channels;

        cv::Mat img = cv::imread(file_path, cv::IMREAD_COLOR);
        cv::resize(img, img, cv::Size(), resize_factor, resize_factor, cv::INTER_NEAREST);
        img = img(cv::Range(0, 140), cv::Range::all());
        cv::split(img, channels);

        cv::threshold(channels[0], img, 150, 255, cv::THRESH_BINARY);
        img.convertTo(img, CV_32F);
        img -= 64;
        cv::Mat img_padded;
        cv::copyMakeBorder(img, img_padded, 0, 0, img.cols/2, img.cols/2, cv::BORDER_CONSTANT, cv::Scalar(0));
        reference_imgs_.push_back(img_padded);
    }
}
