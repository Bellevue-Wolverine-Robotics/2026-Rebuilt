package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeederConstants;

public class FeederSubsystem extends SubsystemBase {
    // Todo: Change to one TalonFX on the competition robot
    private SparkMax motor = new SparkMax(FeederConstants.MOTOR_ID, MotorType.kBrushless);

    public FeederSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void run() {
        motor.set(FeederConstants.FEED_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }
}
