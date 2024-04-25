#include "odometry_wheels.h"

namespace farmwise_odometry
{
FarmwiseOdometryWheels::FarmwiseOdometryWheels(int ticks_per_meter)
    : OdometryWheels(1000, 1), 
        ticks_per_meter_(ticks_per_meter), 
        last_left_tick_(0), last_right_tick_(0),
        last_left_update_(std::chrono::steady_clock::time_point::min()), 
        last_right_update_(std::chrono::steady_clock::time_point::min()), 
        last_update_(std::chrono::steady_clock::time_point::min()), 
        left_speed_(0), right_speed_(0)
{
}

bool FarmwiseOdometryWheels::updateOdometry(OdometryValue& odometry_value) {
    std::lock_guard<std::mutex> lock(mutex_);

    // if (left_encoder_queue_.empty() && right_encoder_queue_.empty()) {
    //     return false;
    // }

    // If it's the first read for either left or right
    // TODO: use a special value for initial speed state instead of 0
    if (left_speed_ == 0 || right_speed_ == 0) {  
        return false;
    }

    // Calculate the average speed of the left and right wheels
    float speed = (left_speed_ + right_speed_) / 2.0;

    // Update the odometry value
    odometry_value.speed = speed;
    last_update_ = last_left_update_ > last_right_update_ ? last_left_update_ : last_right_update_;

    odometry_value.timestamp.secs = 
        std::chrono::duration_cast<std::chrono::seconds>(last_update_.time_since_epoch()).count();
    odometry_value.timestamp.nsecs = 
        std::chrono::nanoseconds(last_update_.time_since_epoch()).count() % 1000000000;

    
    OdometryValue t;
    odom_queue_.pop(t);

    std::cout << "[updateOdometry] odometry_value.speed: " << odometry_value.speed 
        << ", odometry_value.timestamp: " << odometry_value.timestamp.secs << ":" << odometry_value.timestamp.nsecs
        << std::endl;

    return true;
}

void FarmwiseOdometryWheels::processLeftEncoder(const EncoderValue& encoder_value) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::time_point() 
            + std::chrono::seconds(encoder_value.timestamp.secs) 
            + std::chrono::nanoseconds(encoder_value.timestamp.nsecs);

    // If it's the first read
    if (last_left_update_ == std::chrono::steady_clock::time_point::min()) {
        last_left_tick_ = encoder_value.tick;
        last_left_update_ = current_time;

        return;
    }

    // Calculate the tick difference
    int64_t tick_diff = encoder_value.tick - last_left_tick_;

    // Calculate the time difference since the last update
    auto elapsed = 
        std::chrono::duration_cast<std::chrono::seconds>(current_time - last_left_update_).count();

    // Check for overflow/underflow of the encoder tick
    if (tick_diff > EncoderValue::max_tick / 2)
    {
        tick_diff -= EncoderValue::max_tick + 1;
    }
    else if (tick_diff < (-EncoderValue::max_tick / 2))
    {
        tick_diff += EncoderValue::max_tick + 1;
    }

    // Calculate the speed of the left wheel
    left_speed_ = static_cast<float>(tick_diff) / (ticks_per_meter_ * elapsed);

    // Update the last tick and update time
    last_left_tick_ = encoder_value.tick;
    last_left_update_ = current_time;

    std::cout << "[processLeftEncoder] left_speed_: " << left_speed_
        << ", encoder_value.tick: " << encoder_value.tick
        << ", elapsed: " << elapsed
        << std::endl;
}

void FarmwiseOdometryWheels::processRightEncoder(const EncoderValue& encoder_value) {
    std::lock_guard<std::mutex> lock(mutex_);

    std::chrono::steady_clock::time_point current_time = 
        std::chrono::steady_clock::time_point() 
        + std::chrono::seconds(encoder_value.timestamp.secs) 
        + std::chrono::nanoseconds(encoder_value.timestamp.nsecs);

    // If it's the first read
    if (last_right_update_ == std::chrono::steady_clock::time_point::min()) {
        last_right_tick_ = encoder_value.tick;
        last_right_update_ = current_time;

        return;
    }

    // Calculate the tick difference
    int64_t tick_diff = encoder_value.tick - last_right_tick_;

    // Calculate the time difference since the last update
    auto elapsed = 
        std::chrono::duration_cast<std::chrono::seconds>(current_time - last_right_update_).count();

    // Check for overflow/underflow of the encoder tick
    if (tick_diff > EncoderValue::max_tick / 2)
    {
        tick_diff -= EncoderValue::max_tick + 1;
    }
    else if (tick_diff < (-EncoderValue::max_tick / 2))
    {
        tick_diff += EncoderValue::max_tick + 1;
    }

    // Calculate the speed of the right wheel
    right_speed_ = static_cast<float>(tick_diff) / (ticks_per_meter_ * elapsed);

    // Update the last tick and update time
    last_right_tick_ = encoder_value.tick;
    last_right_update_ = current_time;
};

}  // namespace farmwise_odometry

