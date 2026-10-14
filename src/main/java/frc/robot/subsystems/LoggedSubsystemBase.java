package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LoggedSubsystemBase extends SubsystemBase {
    public LoggedSubsystemBase() {
        SmartDashboard.putData(this);
    }
}
