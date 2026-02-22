package frc.robot.adapters;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXController implements MotorController {
    private final TalonFX motor;

    public TalonFXController(int id) {
        motor = new TalonFX(id);        
    }

    public TalonFXController(int id, String bus) {
        motor = new TalonFX(id, new CANBus(bus));
    }

    public void set(double power) {
        motor.setControl(new DutyCycleOut(power));
    }

    public void setPosition(double setpoint) {
        motor.setControl(new PositionVoltage(setpoint));
    }

    public void setVelocity(double setpoint) {
        motor.setControl(new VelocityVoltage(setpoint));
    }

    public void stop() {
        motor.stopMotor();
    }
}
