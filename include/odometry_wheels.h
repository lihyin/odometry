/**********************************************
 * @file odometry_wheels.h
 * @brief Candidate interview for Farmwise Embedded and
 * Robotics Software team. DO NOT COPY.
 * Copyright 2022 FarmWise Labs Inc.
 **********************************************/

#pragma once

#include <boost/lockfree/queue.hpp>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <ctime>
#include <thread>
#include <iostream>

namespace farmwise_odometry
{

/**************
 * Supplied code.
 **************/

struct Timestamp
{
    uint32_t secs;
    uint32_t nsecs;
};

struct EncoderValue
{
    static constexpr int64_t max_tick = (uint32_t(1) << 24) - 1;
    int64_t tick;
    Timestamp timestamp;
};

struct OdometryValue
{
    float speed;
    Timestamp timestamp;
};

class OdometryWheels
{
public:
    ~OdometryWheels()
    {
        stop_threads_ = true;
        for (auto& internal_thread : internal_threads_)
        {
            internal_thread.join();
        }
    };

    /**
     * Non-blocking.
     * To be called to start processing encoder data and producing odometry updates.
     */
    void start(void)
    {
        internal_threads_.push_back(std::thread(&OdometryWheels::callbackLeftEncoder, this));
        internal_threads_.push_back(std::thread(&OdometryWheels::callbackRightEncoder, this));
        internal_threads_.push_back(std::thread(&OdometryWheels::callbackOdometry, this));
    };

    /**
     * Non-blocking. Called when a new update on the left/right encoder
     * position is available.
     * @return false if the new update is discarded.
     */
    bool newEncoderUpdate(const EncoderValue& encoder_value, const bool is_left)
    {
        if (is_left)
        {
            return left_encoder_queue_.push(encoder_value);
        }
        else
        {
            return right_encoder_queue_.push(encoder_value);
        }
    };

    /**
     * Non-blocking. Fetch new odometry update if available.
     * @return true if a new update is available, in which case new_update gets populated.
     */
    bool getOdometryUpdate(OdometryValue& new_update) {
        // return odom_queue_.pop(new_update);

        bool ret = odom_queue_.pop(new_update);
        std::cout << "[getOdometryUpdate] speed: " << new_update.speed 
            << ", new_update.timestamp: " << new_update.timestamp.secs << ":" << new_update.timestamp.nsecs 
            << ", return: " << ret
            << ", odom_queue_ is empty: " << odom_queue_.empty()
            << std::endl;

        return ret;
    };

protected:
    /**
     * Must be called to instantiate subclasses. Subclasses should choose
     * an appropriate encoder_queue_size.
     */
    OdometryWheels(int encoder_queue_size, int odometry_queue_size)
          : left_encoder_queue_(encoder_queue_size)
          , right_encoder_queue_(encoder_queue_size)
          , odom_queue_(odometry_queue_size)
          , stop_threads_(false){};

    virtual bool updateOdometry(OdometryValue& odometry_value) = 0;
    virtual void processLeftEncoder(const EncoderValue& encoder_value) = 0;
    virtual void processRightEncoder(const EncoderValue& encoder_value) = 0;

    // Queues
    boost::lockfree::queue<EncoderValue, boost::lockfree::fixed_sized<true>>
        left_encoder_queue_, right_encoder_queue_;
    boost::lockfree::queue<OdometryValue, boost::lockfree::fixed_sized<true>>
        odom_queue_;

private:
    // Threads
    std::vector<std::thread> internal_threads_;
    bool stop_threads_;

    void callbackLeftEncoder(void)
    {
        EncoderValue left_encoder_update;
        while (true)
        {
            if (stop_threads_)
            {
                return;
            }
            bool is_available = left_encoder_queue_.pop(left_encoder_update);
            if (is_available)
            {
                std::cout << "[callbackLeftEncoder] pop succeed, left_encoder_queue_ is empty: " 
                    << left_encoder_queue_.empty() << std::endl;
                processLeftEncoder(left_encoder_update);
            }
            else
            {
                // std::cout << "[callbackLeftEncoder] pop not succeed, left_encoder_queue_ is empty: " 
                //    << left_encoder_queue_.empty() << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    };

    void callbackRightEncoder(void)
    {
        EncoderValue right_encoder_update;
        while (true)
        {
            if (stop_threads_)
            {
                return;
            }
            bool is_available = right_encoder_queue_.pop(right_encoder_update);
            if (is_available)
            {
                processRightEncoder(right_encoder_update);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    };

    void callbackOdometry(void)
    {
        while (true)
        {
            if (stop_threads_)
            {
                return;
            }
            OdometryValue odometry_value;
            bool is_available = updateOdometry(odometry_value);
            if (is_available)
            {
                std::cout << "[callbackOdometry] updateOdometry available" << std::endl;
                odom_queue_.push(odometry_value);
            }
            else
            {
                std::cout << "[callbackOdometry] updateOdometry not available" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    };
};

/**************
 * End of supplied code
 **************/

class FarmwiseOdometryWheels : public OdometryWheels
{
public:
    FarmwiseOdometryWheels(int ticks_per_meter);
    bool updateOdometry(OdometryValue& odometry_value);
    void processLeftEncoder(const EncoderValue &encoder_value);
    void processRightEncoder(const EncoderValue &encoder_value);

private:
    int ticks_per_meter_;                       // Ticks per meter calibration for the wheels
    int64_t last_left_tick_, last_right_tick_;  // Last tick values for the left and right wheels
    std::chrono::steady_clock::time_point last_left_update_, last_right_update_;  // Last update times for the left and right wheels
    float left_speed_, right_speed_;            // Speeds of the left and right wheels
    std::mutex mutex_;                          // Mutex for thread safety
    std::chrono::steady_clock::time_point last_update_;  // Last update time for the odometry value
};
}  // namespace farmwise_odometry

