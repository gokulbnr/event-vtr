#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <libcaer/libcaer.h>
#include <libcaer/devices/davis.h>

#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <mutex>

struct Event {
    int x, y;
    bool polarity;
    int64_t timestamp;

    Event(int x_, int y_, bool polarity_, int64_t timestamp_)
        : x(x_), y(y_), polarity(polarity_), timestamp(timestamp_) {}
};

class DVSFramePublisher {
private:
    caerDeviceHandle davis;
    int WIDTH = 346, HEIGHT = 260, BINNING_US = 10000;

    std::vector<Event> currentEvents;
    std::mutex eventMutex;

    ros::NodeHandle nh;
    ros::Publisher imgPub;

    cv::Mat eventFrame;
    bool running = false;

public:
    DVSFramePublisher();
    ~DVSFramePublisher();
    void run();
    void readEvents();
    void publishFrame();
};

DVSFramePublisher::DVSFramePublisher() {
    davis = caerDeviceOpen(1, CAER_DEVICE_DAVIS, 0, 0, NULL);
    if (davis == NULL) {
        ROS_FATAL("Could not open DAVIS camera.");
        ros::shutdown();
    }

    caerDeviceSendDefaultConfig(davis);
    caerDeviceConfigSet(davis, DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, false);
    caerDeviceConfigSet(davis, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, false);
    caerDeviceConfigSet(davis, DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE, false);
    caerDeviceConfigSet(davis, DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, true);
    caerDeviceDataStart(davis, NULL, NULL, NULL, NULL, NULL);

    imgPub = nh.advertise<sensor_msgs::Image>("/dvs/event_frame", 1);
    eventFrame = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3);
}

DVSFramePublisher::~DVSFramePublisher() {
    running = false;
    caerDeviceDataStop(davis);
    caerDeviceClose(&davis);
}

void DVSFramePublisher::run() {
    running = true;
    std::thread reader(&DVSFramePublisher::readEvents, this);
    ros::Rate loopRate(100);  // 100 Hz

    while (ros::ok()) {
        publishFrame();
        ros::spinOnce();
        loopRate.sleep();
    }

    running = false;
    reader.join();
}

void DVSFramePublisher::readEvents() {
    while (running) {
        caerEventPacketContainer packetContainer = caerDeviceDataGet(davis);
        if (packetContainer == NULL) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        int32_t numPackets = caerEventPacketContainerGetEventPacketsNumber(packetContainer);
        for (int32_t i = 0; i < numPackets; i++) {
            caerEventPacketHeader header = caerEventPacketContainerGetEventPacket(packetContainer, i);
            if (header == NULL || caerEventPacketHeaderGetEventType(header) != POLARITY_EVENT) continue;

            caerPolarityEventPacket packet = (caerPolarityEventPacket)header;
            int32_t numEvents = caerEventPacketHeaderGetEventNumber(header);

            std::lock_guard<std::mutex> lock(eventMutex);
            for (int j = 0; j < numEvents; j++) {
                caerPolarityEvent e = caerPolarityEventPacketGetEvent(packet, j);
                currentEvents.emplace_back(
                    caerPolarityEventGetX(e),
                    caerPolarityEventGetY(e),
                    caerPolarityEventGetPolarity(e),
                    caerPolarityEventGetTimestamp64(e, packet)
                );
            }
        }
        caerEventPacketContainerFree(packetContainer);
    }
}

void DVSFramePublisher::publishFrame() {
    static int64_t lastTimestamp = -1;
    int64_t currentTimestamp = -1;

    std::vector<Event> eventsToProcess;

    {
        std::lock_guard<std::mutex> lock(eventMutex);
        if (currentEvents.empty()) return;

        if (lastTimestamp == -1) {
            lastTimestamp = currentEvents.front().timestamp;
        }

        currentTimestamp = currentEvents.back().timestamp;

        auto it = std::remove_if(currentEvents.begin(), currentEvents.end(),
            [&](const Event& e) {
                if (e.timestamp < lastTimestamp + BINNING_US) {
                    eventsToProcess.push_back(e);
                    return true;
                }
                return false;
            });
        currentEvents.erase(it, currentEvents.end());
    }

    if (eventsToProcess.empty()) return;

    eventFrame.setTo(cv::Scalar(0, 0, 0));
    for (const auto& e : eventsToProcess) {
        if (e.x >= 0 && e.x < WIDTH && e.y >= 0 && e.y < HEIGHT) {
            eventFrame.at<cv::Vec3b>(e.y, e.x) = e.polarity ?
                cv::Vec3b(255, 255, 255) :  // ON - Red
                cv::Vec3b(255, 255, 255);   // OFF - Blue
        }
    }

    auto msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", eventFrame).toImageMsg();
    msg->header.stamp = ros::Time::now();
    imgPub.publish(msg);

    lastTimestamp += BINNING_US;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dvs_frame_publisher");
    DVSFramePublisher publisher;
    publisher.run();
    return 0;
}
