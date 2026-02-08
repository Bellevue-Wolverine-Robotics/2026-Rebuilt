package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        ClimberConstants.FORWARD_CHANNEL,
        ClimberConstants.REVERSE_CHANNEL
    );

    public Command stop() {
        return runOnce(() -> solenoid.set(DoubleSolenoid.Value.kOff));
    }

    public Command extend() {
        return runOnce(() -> solenoid.set(DoubleSolenoid.Value.kForward));
    }

    public Command retract() {
        return runOnce(() -> solenoid.set(DoubleSolenoid.Value.kReverse));
    }
}
