package frc.robot.adapters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SparkMaxController implements MotorController {
    private final SparkMax motor;
    SparkClosedLoopController controller;

    public SparkMaxController(int id) {
        motor = new SparkMax(id, MotorType.kBrushless);
        controller = motor.getClosedLoopController();
    }

    public void set(double power) {
        controller.setSetpoint(power, ControlType.kDutyCycle);
    }

    public void setPosition(double setpoint) {
        controller.setSetpoint(setpoint, ControlType.kPosition);
    }

    public void setVelocity(double setpoint) {
        controller.setSetpoint(setpoint, ControlType.kVelocity);
    }

    public void stop() {
        motor.stopMotor();
    }
}
