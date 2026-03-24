package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

public class FeederConstants {
    public static final int MOTOR_ID = 23;
    public static final CANBus CANBUS = new CANBus("canivore");

    public static final double FEED_SPEED = 1;
}
