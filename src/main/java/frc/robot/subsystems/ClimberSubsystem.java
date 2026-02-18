package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ClimberConstants;

/* Represents the climber mechanism, allowing the robot to climb on the tower. */
public class ClimberSubsystem extends SubsystemBase {
    private final DoubleSolenoid solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        ClimberConstants.FORWARD_CHANNEL,
        ClimberConstants.REVERSE_CHANNEL
    );

    /**
     * Opens the forward side of the solenoid valves, to extend the intake arms.
     * 
     * @return Command that extends the arms.
     */
    public Command extendCommand() {
        return runOnce(() -> solenoid.set(DoubleSolenoid.Value.kForward));
    }

    /**
     * Opens the reverse side of the solenoid valves, to retract the arms.
     * 
     * @return Command that retracts the arms.
     */
    public Command retractCommand() {
        return runOnce(() -> solenoid.set(DoubleSolenoid.Value.kReverse));
    }
}
