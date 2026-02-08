package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FeedingConstants;

public class FeedingSubsystem extends SubsystemBase {
    // Both motors are linked, controlling the conveyor belt feeder and the gateway roller when moving.
    // Might need to be changed to 1 TalonFX in the future, uncertain
    private SparkMax masterMotor = new SparkMax(FeedingConstants.FEED_MOTOR_1_ID, MotorType.kBrushless);
    private SparkMax followerMotor = new SparkMax(FeedingConstants.FEED_MOTOR_2_ID, MotorType.kBrushless);

    public FeedingSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);

        masterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(masterMotor, false);

        followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Command FeedFuelToShooter() {
        return new RunCommand(
            () -> masterMotor.set(FeedingConstants.FEEDING_MOTOR_VELOCITY), this)
            .finallyDo(() -> masterMotor.stopMotor());
    }
}
