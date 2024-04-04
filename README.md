# Instruction

The solution should be implemented in C++11+, and passing tests in `test_drive_straight.cpp`. We are expecting **a subclass of _OdometryWheels_ named _FarmwiseOdometryWheels_** that implements all virtual/abstract methods that have not been implemented yet, as well as the constructor. **Note that all methods should be thread-safe.** Any new code should conform to the same style as the provided codebase. The _OdometryWheels_ and the test suits are validated in linux/unix.

The encoders for the left and right wheels are set up to output **absolute rotational positions** in units of 'ticks' at 50 Hz as messages on a CAN bus. These messages are parsed and then relayed to the _OdometryWheels_ class through calls to the `newEncoderUpdate()` class function. Note left and right encoders are not updated simultaneously: they are managed by different entities, and thus the left odometry updates may be out of sync in timestamps with right odometry updates. Nevertheless, within the same update stream of encoder values (i.e. left update streams and right update stream), the data is guaranteed to come in strictly increasing time order.

As the wheel turns forward, the position in units of ticks as reported by the encoder will increase. On the other hand, as the wheel turns backward, the position will decrease. The ticks-per-meter calibration of each wheel will be supplied, and it should be specified as parameters at instantiation of the subclass.

the data range of the encoder is [0 - 16777215], with the value wrapping between the min and max in the case of overflow / underflow. For example, if the encoder reports 16777200 and then increments by 20 ticks, the next value reported by the encoder is 4.

The `getOdometryUpdate` function may be called at any point in time to get the latest estimate of the instantaneous speed of the robot reported at the center of the two wheels. To simplify this problem, we will make two simplifying assumptions:
- Timestamps between the left and right wheel encoder will match up (e.g. latched to the nearest every 20ms if running at 50hz).
- To calculate the instantaneous speed at time **t** for the robot is the average of the instantaneous speeds of the left and right wheels at time **t**. (i.e. _(speed_left_wheel + speed_right_wheel) / 2_). As mentioned above, left and right streams may be out of sync in timestamps. Be sure to take care of lagging updates.
