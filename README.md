Sensors

Sensor fusion pipeline running at 100 Hz via shared memory. Reads pitch and angular rate from a BNO085 IMU over I2C, wheel velocity from two AMT10 quadrature encoders, and obstacle distance from two HC-SR04 ultrasonic sensors. Includes IMU calibration, drift compensation, and a fallback data chain (SHM, file cache, SQLite).

Motor Controller

Interface layer between the Raspberry Pi and the Sabertooth 2x12 dual motor driver. Sends PWM servo signals to independently control left and right DC motors at 0 to 24V. Handles safe-stop on fault detection within one control cycle.
