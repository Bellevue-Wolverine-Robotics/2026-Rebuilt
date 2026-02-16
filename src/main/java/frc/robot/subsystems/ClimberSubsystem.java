package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ClimberConstants;

/* Represents the climber mechanism, allowing the robot to climb on the tower. */
public class ClimberSubsystem extends SubsystemBase {
    private final DoubleSolenoid leftSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        ClimberConstants.LEFT_FORWARD_CHANNEL,
        ClimberConstants.LEFT_REVERSE_CHANNEL
    );

    private final DoubleSolenoid rightSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        ClimberConstants.RIGHT_FORWARD_CHANNEL,
        ClimberConstants.RIGHT_REVERSE_CHANNEL
    );

    /**
     * Sets the valves on both sides to a certain posiiton.
     * 
     * @param valve The position of both valves.
    */
    private void set(DoubleSolenoid.Value value) {
        leftSolenoid.set(value);
        rightSolenoid.set(value);
    }

    /**
     * Opens the forward side of the solenoid valves, to extend the intake arms.
     * 
     * @return Command that extends the arms.
     */
    public Command extendCommand() {
        return runOnce(() -> set(DoubleSolenoid.Value.kForward));
    }

    /**
     * Opens the reverse side of the solenoid valves, to retract the arms.
     * 
     * @return Command that retracts the arms.
     */
    public Command retractCommand() {
        return runOnce(() -> set(DoubleSolenoid.Value.kReverse));
    }
}
