#include "odometry_wheels.h"
#include <cassert>
#include <memory>
#include <iostream>


farmwise_odometry::EncoderValue encoder_value;
farmwise_odometry::OdometryValue odometry_value;
std::shared_ptr<farmwise_odometry::FarmwiseOdometryWheels> odometry_wheels;

#define TICKS_PER_METER 300

void start()
{
    odometry_wheels = std::make_shared<farmwise_odometry::FarmwiseOdometryWheels>(TICKS_PER_METER);
    odometry_wheels->start();
}

bool is_same_float(float float1, float float2)
{
    return (std::abs(float1 - float2) < 1e-6);
}

// Test one value pair, no speed available
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

// Test multiple value pairs, constant speed, overflowed
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

// Test multiple value pairs, speeding up
void test_3()
{
    start();
    for (size_t i = 0; i < 10; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick += i;
        odometry_wheels->newEncoderUpdate(encoder_value, true);
        odometry_wheels->newEncoderUpdate(encoder_value, false);
        usleep(1e5);
        if (i == 0)
        {
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        assert(odometry_wheels->getOdometryUpdate(odometry_value));
        assert(is_same_float(odometry_value.speed, i / static_cast<float>(TICKS_PER_METER)));
    }
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
}

// Test multiple value pairs, going backwards, constant speed, underflowed
void test_4()
{
    start();
    for (size_t i = 0; i < 20; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick = (10 - i + farmwise_odometry::EncoderValue::max_tick) % (farmwise_odometry::EncoderValue::max_tick + 1);
        odometry_wheels->newEncoderUpdate(encoder_value, true);
        odometry_wheels->newEncoderUpdate(encoder_value, false);
        usleep(1e5);
        if (i == 0)
        {
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        assert(odometry_wheels->getOdometryUpdate(odometry_value));
        assert(is_same_float(odometry_value.speed, -1 / static_cast<float>(TICKS_PER_METER)));
    }
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
}

// Constant speed, delayed right encoder updates
void test_5()
{
    start();
    for (size_t i = 0; i < 10; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick = i;
        odometry_wheels->newEncoderUpdate(encoder_value, true);
        usleep(1e5);
        assert(!odometry_wheels->getOdometryUpdate(odometry_value));
    }

    for (size_t i = 0; i < 10; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick = i;
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

// Constant speed, with some dropped encoder updates
void test_6()
{
    start();
    for (size_t i = 0; i < 10; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick = i;

        if (i == 0)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        else if (i == 5 || i == 6)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            usleep(1e5);
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
        }
        else
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(odometry_wheels->getOdometryUpdate(odometry_value));
            assert(is_same_float(odometry_value.speed, 1 / static_cast<float>(TICKS_PER_METER)));
        }
    }
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
}

// Speeding up, with some dropped encoder updates, unit sampling period
void test_7()
{
    start();
    for (size_t i = 0; i < 10; i++)
    {
        encoder_value.timestamp.secs = 2 * i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick += i;
        if (i == 0)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        else if (i == 5 || i == 6)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            usleep(1e5);
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        else if (i == 7)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        else
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(odometry_wheels->getOdometryUpdate(odometry_value));
            assert(is_same_float(odometry_value.speed, i / static_cast<float>(TICKS_PER_METER) / 2.0));
        }
    }
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
    odometry_wheels.reset();
}

// Speeding up, with some dropped encoder updates, non-integer sampling period
void test_8()
{
    start();
    for (size_t i = 0; i < 10; i++)
    {
        encoder_value.timestamp.secs = 2 * i + (i / 2);
        encoder_value.timestamp.nsecs = (i % 2) ? 500000000 : 0;
        encoder_value.tick += i;
        if (i == 0)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        else if (i == 5 || i == 6)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            usleep(1e5);
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        else if (i == 7)
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        else
        {
            odometry_wheels->newEncoderUpdate(encoder_value, true);
            odometry_wheels->newEncoderUpdate(encoder_value, false);
            usleep(1e5);
            assert(odometry_wheels->getOdometryUpdate(odometry_value));
            assert(is_same_float(odometry_value.speed, i / static_cast<float>(TICKS_PER_METER) / 2.5));
        }
    }
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
    odometry_wheels.reset();
}

// Test multiple value pairs but not reading odo regularly, speeding up
void test_9()
{
    start();
    for (size_t i = 0; i < 10; i++)
    {
        encoder_value.timestamp.secs = i;
        encoder_value.timestamp.nsecs = 0;
        encoder_value.tick += i;
        odometry_wheels->newEncoderUpdate(encoder_value, true);
        odometry_wheels->newEncoderUpdate(encoder_value, false);
        usleep(1e5);
        if (i == 0)
        {
            assert(!odometry_wheels->getOdometryUpdate(odometry_value));
            continue;
        }
        if (i % 2 == 0)
        {
            assert(odometry_wheels->getOdometryUpdate(odometry_value));
            assert(is_same_float(odometry_value.speed, i / static_cast<float>(TICKS_PER_METER)));
        }
    }
    // i == 9 was not read
    assert(odometry_wheels->getOdometryUpdate(odometry_value));
    assert(!odometry_wheels->getOdometryUpdate(odometry_value));
}

int main(int argc, char** argv)
{
    std::cout << "Test 1 "; test_1(); std::cout << "✔️" << std::endl;
    std::cout << "Test 2 "; test_2(); std::cout << "✔️" << std::endl;
    std::cout << "Test 3 "; test_3(); std::cout << "✔️" << std::endl;
    std::cout << "Test 4 "; test_4(); std::cout << "✔️" << std::endl;
    std::cout << "Test 5 "; test_5(); std::cout << "✔️" << std::endl;
    std::cout << "Test 6 "; test_6(); std::cout << "✔️" << std::endl;
    std::cout << "Test 7 "; test_7(); std::cout << "✔️" << std::endl;
    std::cout << "Test 8 "; test_8(); std::cout << "✔️" << std::endl;
    std::cout << "Test 9 "; test_9(); std::cout << "✔️" << std::endl;
}
