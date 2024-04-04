#include "odometry_wheels.h"

namespace farmwise_odometry
{


class FarmwiseOdometryWheels : public OdometryWheels
{
public:
    FarmwiseOdometryWheels(int ticks_per_meter)
        : OdometryWheels(1000, 1000), ticks_per_meter_(ticks_per_meter), last_left_tick_(0), last_right_tick_(0),
          last_left_update_(0), last_right_update_(0), left_speed_(0), right_speed_(0)
    {
    }

    bool updateOdometry(OdometryValue& odometry_value) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Calculate the elapsed time since the last update
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_update_).count();
        if (elapsed == 0)
        {
            return false; // Not enough time has elapsed
        }
        last_update_ = now;

        // Calculate the average speed of the left and right wheels
        float speed = (left_speed_ + right_speed_) / 2.0;

        // Calculate the distance traveled by the center of the wheels
        float distance = speed * elapsed;

        // Update the odometry value
        odometry_value.speed = speed;
        odometry_value.timestamp.secs = std::chrono::seconds(now.time_since_epoch()).count();
        odometry_value.timestamp.nsecs = std::chrono::nanoseconds(now.time_since_epoch()).count() % 1000000000;

        return true;
    }

    void processLeftEncoder(const EncoderValue& encoder_value) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Calculate the tick difference
        int64_t tick_diff = encoder_value.tick - last_left_tick_;

        // Calculate the time difference since the last update
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_left_update_).count();

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
        last_left_update_ = now;
    }

    void processRightEncoder(const EncoderValue& encoder_value) override
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Calculate the tick difference
        int64_t tick_diff = encoder_value.tick - last_right_tick_;

        // Calculate the time difference since the last update
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_right_update_).count();

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
        last_right_update_ = now;
    }

private:
    int ticks_per_meter_;                       // Ticks per meter calibration for the wheels
    int64_t last_left_tick_, last_right_tick_;  // Last tick values for the left and right wheels
    std::chrono::steady_clock::time_point last_left_update_, last_right_update_;  // Last update times for the left and right wheels
    float left_speed_, right_speed_;            // Speeds of the left and right wheels
    std::mutex mutex_;                          // Mutex for thread safety
    std::chrono::steady_clock::time_point last_update_;  // Last update time for the odometry value
};

}  // namespace farmwise_odometry

