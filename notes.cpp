void test_1()
{
    start();
    for (size_t i = 0; i < 1; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick = i;
        odometry_wheels->newEncoderUpdate(encoder_value, true);
        odometry_wheels->newEncoderUpdate(encoder_value, false);
    }
    usleep(1e5);
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
}

void test_2()
{
    start();
    for (size_t i = 0; i < 20; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick = (farmwise_odometry::EncoderValue::max_tick - 10 + i) % (farmwise_odometry::EncoderValue::max_tick + 1);
        odometry_wheels->newEncoderUpdate(encoder_value, true);
        odometry_wheels->newEncoderUpdate(encoder_value, false);
        usleep(1e5);
        if (i == 0)
        {
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }

        assert(odometry_wheels->getOdometryUpdate(odometry_value));
        assert(is_same_float(odometry_value.speed, 1 / static_cast<float>(TICKS_PER_METER)));
    }
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
}

class OdometryWheels
{
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

    bool getOdometryUpdate(OdometryValue& new_update) { 
        return odom_queue_.pop(new_update); 
    };

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
                processLeftEncoder(left_encoder_update);
            }
            else
            {
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
                odom_queue_.push(odometry_value);
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    };
}
