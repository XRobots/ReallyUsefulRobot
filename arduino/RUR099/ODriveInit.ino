// init hips

void OdriveInit1() {

          //Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 60000.0f << '\n';
          //Serial1 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
          //Serial1 << "w axis" << axis << ".motor.config.calibration_current " << 10.0f << '\n';

          // *** current limit and other parameers are set on the ODrive using the Odrive tool as above. *** 

          // init ODrive axis

          //axis 0

          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          odrive1.run_state(0, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          odrive1.run_state(0, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          odrive1.run_state(0, requested_state, false); // don't wait 

          //axis 1

          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          odrive1.run_state(1, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          odrive1.run_state(1, requested_state, true);
    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          odrive1.run_state(1, requested_state, false); // don't wait 
               
}


