#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>


enum class GOAL_STATE {
    normal_goal = 0,
    finished = 1,
    restart = 2
};


nav_msgs::Odometry subtract_odom(const nav_msgs::Odometry& odom, const tf::Transform& odom_frame_to_subtract) {
    tf::Transform odom_frame;
    tf::poseMsgToTF(odom.pose.pose, odom_frame);

    tf::Transform subtracted_odom = odom_frame_to_subtract.inverseTimes(odom_frame);

    nav_msgs::Odometry odom_result = odom;
    tf::poseTFToMsg(subtracted_odom, odom_result.pose.pose);
    return odom_result;
}


tf::Transform getCorrectedGoalOffset(const tf::Transform& goal1, const tf::Transform& goal2, double rotation_correction, double correction_length) {
    // Offset from goal1 to goal2
    tf::Transform goal_offset = goal1.inverse() * goal2;

    // Apply rotation correction
    tf::Transform rotation(tf::createQuaternionFromYaw(rotation_correction), tf::Vector3(0, 0, 0));
    goal_offset = rotation * goal_offset;

    // Apply length correction
    tf::Vector3 p = goal_offset.getOrigin();
    p *= correction_length;
    goal_offset.setOrigin(p);

    return goal_offset;
}


void printTfTransform(const tf::Transform& transform) {
    const tf::Vector3& origin = transform.getOrigin();
    const tf::Quaternion& rotation = transform.getRotation();

    std::cout << "Translation: ["
              << origin.x() << ", "
              << origin.y() << ", "
              << origin.z() << "]\n";

    std::cout << "Rotation (quaternion): ["
              << rotation.x() << ", "
              << rotation.y() << ", "
              << rotation.z() << ", "
              << rotation.w() << "]\n";
}



void normaliseQuaternion(geometry_msgs::Quaternion& q) {
    double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm > 0.0) {
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
        q.w /= norm;
    }
}



inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}


std::string readFile(const std::string& path) {
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}



tf::Transform jsonToPose(const nlohmann::json& j) {
    tf::Vector3 translation(
        j["position"]["x"],
        j["position"]["y"],
        j["position"]["z"]
    );

    tf::Quaternion rotation(
        j["orientation"]["x"],
        j["orientation"]["y"],
        j["orientation"]["z"],
        j["orientation"]["w"]
    );

    return tf::Transform(rotation, translation);
}


std::string expandUser(const std::string& path) {
    if (!path.empty() && path[0] == '~') {
        const char* home = std::getenv("HOME");
        if (home) {
            return std::string(home) + path.substr(1);
        }
    }
    return path;
}


cv::Mat reduceWidthBySumming8(const cv::Mat& input) {
    int rows = input.rows;
    int cols = input.cols;

    // Output image will have the same number of rows but 1/8 the number of columns
    int reducedCols = cols / 8;
    cv::Mat output(rows, reducedCols, CV_32FC1, cv::Scalar(0));

    for (int i = 0; i < rows; ++i) {
        const float* inputRow = input.ptr<float>(i);
        float* outputRow = output.ptr<float>(i);

        for (int j = 0; j < reducedCols; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 8; ++k) {
                sum += inputRow[j * 8 + k];
            }
            outputRow[j] = sum;
        }
    }

    return output;
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


cv::Mat visualize(const cv::Mat& qry, const cv::Mat& match, int match_y) {
    cv::Mat qry_color, match_color;

    // Convert grayscale to BGR if needed
    if (qry.channels() == 1) {
        cv::cvtColor(qry, qry_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(match, match_color, cv::COLOR_GRAY2BGR);
    } else {
        qry_color = qry.clone();
        match_color = match.clone();
    }

    // Draw green vertical line in query image (center)
    int qry_center_x = qry.cols / 2;
    cv::line(qry_color, cv::Point(qry_center_x, 0), cv::Point(qry_center_x, qry.rows), cv::Scalar(0, 255, 0), 2);

    // Draw green vertical line in match image at match_y
    cv::line(match_color, cv::Point(match_y, 0), cv::Point(match_y, match.rows), cv::Scalar(0, 255, 0), 2);

    // Concatenate vertically
    cv::Mat combined;
    cv::vconcat(qry_color, match_color, combined);

    return combined;
}


// Helper function to extract 2D patches from a padded image
std::vector<cv::Mat> getPatches2D(const cv::Mat& padded, const cv::Size& patch_size) {
    std::vector<cv::Mat> patches;
    int rows = padded.rows - patch_size.height + 1;
    int cols = padded.cols - patch_size.width + 1;

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            cv::Rect roi(j, i, patch_size.width, patch_size.height);
            patches.push_back(padded(roi).clone());
        }
    }
    return patches;
}

// Main function equivalent to patch_normalise_pad
cv::Mat patchNormalisePad(const cv::Mat& image, const cv::Size& patch_size) {
    CV_Assert(patch_size.width % 2 == 1 && patch_size.height % 2 == 1);
    int half_h = (patch_size.height - 1) / 2;
    int half_w = (patch_size.width - 1) / 2;

    // Convert to float64
    cv::Mat float_img;
    image.convertTo(float_img, CV_64F);

    // Pad the image with NaN
    cv::Mat padded(image.rows + 2 * half_h, image.cols + 2 * half_w, CV_64F, cv::Scalar(std::numeric_limits<double>::quiet_NaN()));
    float_img.copyTo(padded(cv::Rect(half_w, half_h, image.cols, image.rows)));

    // Extract patches
    auto patches = getPatches2D(padded, patch_size);
    int nrows = image.rows;
    int ncols = image.cols;

    // Compute mean and stddev
    cv::Mat mean_map = cv::Mat::zeros(nrows, ncols, CV_64F);
    cv::Mat std_map = cv::Mat::zeros(nrows, ncols, CV_64F);

    for (int i = 0; i < nrows * ncols; ++i) {
        const cv::Mat& patch = patches[i];
        cv::Scalar mean, stddev;
        cv::meanStdDev(patch, mean, stddev, ~cv::Mat(patch != patch)); // Mask NaNs
        mean_map.at<double>(i / ncols, i % ncols) = mean[0];
        std_map.at<double>(i / ncols, i % ncols) = stddev[0];
    }

    // Normalize
    cv::Mat out = (float_img - mean_map) / std_map;

    // Handle NaNs and clamp values
    for (int i = 0; i < out.rows; ++i) {
        for (int j = 0; j < out.cols; ++j) {
            double& val = out.at<double>(i, j);
            if (std::isnan(val))
                val = 127.0;
            else if (val < -1.0)
                val = 0.0;
            else if (val > 1.0)
                val = 255.0;
        }
    }

    out.convertTo(out, CV_8U); // Convert back to 8-bit unsigned integer
    return out;
}


cv::Mat ImgProcessing(const cv::Mat& img, double resize_factor) {
    cv::Mat res, bin;
    std::vector<cv::Mat> channels;

    // cv::cvtColor(img, res, cv::COLOR_BGR2GRAY);
    cv::resize(img, res, cv::Size(), resize_factor, resize_factor, cv::INTER_NEAREST);
    res = res(cv::Range(40, 180), cv::Range::all());  // rows 40–100 inclusive

    cv::split(res, channels); // channels[0] = Blue, channels[1] = Green, channels[2] = Red
    cv::threshold(channels[0], bin, 150, 255, cv::THRESH_BINARY);

    // cv::threshold(res, res, 150, 255, cv::THRESH_BINARY);
    bin.convertTo(bin, CV_32F);
    bin -= 64;
    return bin;
}

tf::Transform interpolatePose(const std::vector<tf::Transform>& poses, double index) {
    int i = static_cast<int>(std::floor(index));
    double frac = index - i;

    if (i < 0 || i + 1 >= static_cast<int>(poses.size())) {
        throw std::out_of_range("Index out of range for interpolation");
    }

    // Get the two poses
    const tf::Transform& pose1 = poses[i];
    const tf::Transform& pose2 = poses[i + 1];

    // Interpolate translation (linear)
    tf::Vector3 t1 = pose1.getOrigin();
    tf::Vector3 t2 = pose2.getOrigin();
    tf::Vector3 t_interp = t1 * (1.0 - frac) + t2 * frac;

    // Interpolate rotation (slerp)
    tf::Quaternion q1 = pose1.getRotation();
    tf::Quaternion q2 = pose2.getRotation();
    tf::Quaternion q_interp = q1.slerp(q2, frac);

    // Combine
    tf::Transform result;
    result.setOrigin(t_interp);
    result.setRotation(q_interp);
    return result;
}

// Helper to mimic np.arange(start, stop, step=1)
std::vector<double> arange(double start, double stop, double step = 1.0) {
    std::vector<double> result;
    for (double val = start; val < stop + 1e-9; val += step) {
        result.push_back(val);
    }
    return result;
}