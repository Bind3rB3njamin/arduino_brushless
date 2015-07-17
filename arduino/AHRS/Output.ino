/* This file is part of the Razor AHRS Firmware */

// Output angles: yaw, pitch, roll
void output_angles()
{
  if (output_format == OUTPUT__FORMAT_BINARY)
  {
    float ypr[3];  
    ypr[0] = TO_DEG(yaw);
    ypr[1] = TO_DEG(pitch);
    ypr[2] = TO_DEG(roll);
    BufferedUART::write_s((byte*) ypr, 12);  // No new-line
  }
  else if (output_format == OUTPUT__FORMAT_TEXT)
  {
    BufferedUART::write_s("#YPR=");
    BufferedUART::write_f(TO_DEG(yaw),2); BufferedUART::write_s(",");
    BufferedUART::write_f(TO_DEG(pitch),2); BufferedUART::write_s(",");
    BufferedUART::write_f(TO_DEG(roll),2); BufferedUART::write_c('\n');
  }
}

void output_calibration(int calibration_sensor)
{
  if (calibration_sensor == 0)  // Accelerometer
  {
    // Output MIN/MAX values
    BufferedUART::write_s("accel x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (accel[i] < accel_min[i]) accel_min[i] = accel[i];
      if (accel[i] > accel_max[i]) accel_max[i] = accel[i];
      BufferedUART::write_f(accel_min[i],2);
      BufferedUART::write_s("/");
      BufferedUART::write_f(accel_max[i],2);
      if (i < 2) BufferedUART::write_s("  ");
      else BufferedUART::write_c('\n');
    }
  }
  else if (calibration_sensor == 1)  // Magnetometer
  {
    // Output MIN/MAX values
    BufferedUART::write_s("magn x,y,z (min/max) = ");
    for (int i = 0; i < 3; i++) {
      if (magnetom[i] < magnetom_min[i]) magnetom_min[i] = magnetom[i];
      if (magnetom[i] > magnetom_max[i]) magnetom_max[i] = magnetom[i];
      BufferedUART::write_f(magnetom_min[i],2);
      BufferedUART::write_s("/");
      BufferedUART::write_f(magnetom_max[i],2);
      if (i < 2) BufferedUART::write_s("  ");
      else BufferedUART::write_c('\n');
    }
  }
  else if (calibration_sensor == 2)  // Gyroscope
  {
    // Average gyro values
    for (int i = 0; i < 3; i++)
      gyro_average[i] += gyro[i];
    gyro_num_samples++;
      
    // Output current and averaged gyroscope values
    BufferedUART::write_s("gyro x,y,z (current/average) = ");
    for (int i = 0; i < 3; i++) {
      BufferedUART::write_f(gyro[i],2);
      BufferedUART::write_s("/");
      BufferedUART::write_f(gyro_average[i] / (float) gyro_num_samples,2);
      if (i < 2) BufferedUART::write_s("  ");
      else BufferedUART::write_c('\n');
    }
  }
}

void output_sensors_text(char raw_or_calibrated)
{
  BufferedUART::write_s("#A-"); BufferedUART::write_c(raw_or_calibrated); BufferedUART::write_c('=');
  BufferedUART::write_f(accel[0],2); BufferedUART::write_s(",");
  BufferedUART::write_f(accel[1],2); BufferedUART::write_s(",");
  BufferedUART::write_f(accel[2],2); BufferedUART::write_c('\n');

  BufferedUART::write_s("#M-"); BufferedUART::write_c(raw_or_calibrated); BufferedUART::write_c('=');
  BufferedUART::write_f(magnetom[0],2); BufferedUART::write_s(",");
  BufferedUART::write_f(magnetom[1],2); BufferedUART::write_s(",");
  BufferedUART::write_f(magnetom[2],2); BufferedUART::write_c('\n');

  BufferedUART::write_s("#G-"); BufferedUART::write_c(raw_or_calibrated); BufferedUART::write_c('=');
  BufferedUART::write_f(gyro[0],2); BufferedUART::write_s(",");
  BufferedUART::write_f(gyro[1],2); BufferedUART::write_s(",");
  BufferedUART::write_f(gyro[2],2); BufferedUART::write_c('\n');
}

void output_both_angles_and_sensors_text()
{
  BufferedUART::write_s("#YPRAG=");
  BufferedUART::write_f(TO_DEG(yaw),2); BufferedUART::write_s(",");
  BufferedUART::write_f(TO_DEG(pitch),2); BufferedUART::write_s(",");
  BufferedUART::write_f(TO_DEG(roll),2); BufferedUART::write_s(",");
  
  BufferedUART::write_f(Accel_Vector[0],2); BufferedUART::write_s(",");
  BufferedUART::write_f(Accel_Vector[1],2); BufferedUART::write_s(",");
  BufferedUART::write_f(Accel_Vector[2],2); BufferedUART::write_s(",");

  BufferedUART::write_f(Gyro_Vector[0],2); BufferedUART::write_s(",");
  BufferedUART::write_f(Gyro_Vector[1],2); BufferedUART::write_s(",");
  BufferedUART::write_f(Gyro_Vector[2],2); BufferedUART::write_c('\n');
}

void output_sensors_binary()
{
  BufferedUART::write_s((byte*) accel, 12);
  BufferedUART::write_s((byte*) magnetom, 12);
  BufferedUART::write_s((byte*) gyro, 12);
}

void output_sensors()
{
  if (output_mode == OUTPUT__MODE_SENSORS_RAW)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('R');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_CALIB)
  {
    // Apply sensor calibration
    compensate_sensor_errors();
    
    if (output_format == OUTPUT__FORMAT_BINARY)
      output_sensors_binary();
    else if (output_format == OUTPUT__FORMAT_TEXT)
      output_sensors_text('C');
  }
  else if (output_mode == OUTPUT__MODE_SENSORS_BOTH)
  {
    if (output_format == OUTPUT__FORMAT_BINARY)
    {
      output_sensors_binary();
      compensate_sensor_errors();
      output_sensors_binary();
    }
    else if (output_format == OUTPUT__FORMAT_TEXT)
    {
      output_sensors_text('R');
      compensate_sensor_errors();
      output_sensors_text('C');
    }
  }
}

