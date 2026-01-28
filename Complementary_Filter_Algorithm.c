    const float dt = 1.0f / 200.0f;
    const float tau = 0.1f;
    const float alpha = tau / (tau + dt);
    gx = sensor->wx * DEG_TO_RAD;
    gy = sensor->wy * DEG_TO_RAD;
    gz = sensor->wz * DEG_TO_RAD;
    ax = sensor->ax;
    ay = sensor->ay;
    az = sensor->az;
    //计算加速度计欧拉角
    accel_roll=atan2f(ay,az);
    accel_pitch=atan2f(-ax, sqrt(ay*ay + az*az));

    float s = sin(gyro_roll);
    float c = cos(gyro_roll);
    float t = tan(gyro_pitch);
    float sec = 1.0f / cos(gyro_pitch);
    //计算变化率
    float roll_rate = gx + s * t * gy + c * t * gz;
    float pitch_rate = c * gy - s * gz;
    float yaw_rate = (s * sec) * gy + (c * sec) * gz;
    //计算新的陀螺仪欧拉角
    float roll_new = gyro_roll + roll_rate * dt;
    float pitch_new = gyro_pitch + pitch_rate * dt;
    float yaw_new = gyro_yaw + yaw_rate * dt;

    gyro_roll=roll_new;
    gyro_pitch=pitch_new;
    gyro_yaw=yaw_new;
    atti->roll=alpha*roll_new+(1-alpha)*accel_roll;
    atti->pitch=alpha*pitch_new+(1-alpha)*accel_pitch;
    atti->yaw=yaw_new;