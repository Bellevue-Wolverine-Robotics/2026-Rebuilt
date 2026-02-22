package frc.robot.adapters;

public interface MotorController {
    /**
     * Directly sets the output of the motor to a given voltage.
     * 
     * @param power The percentage of the maximum voltage to set the motor to, from -1.0 to 1.0.
     */
    public void set(double power);

    /**
     * Sets the position of the motor using the internal PID controller.
     * 
     * @param setpoint The position of the shaft in relative rotations.
     */ 
    public void setPosition(double setpoint);

    /**
     * Sets the velocity of the motor using the internal PID controller.
     * 
     * @param setpoint The velocity of the shaft in rotations per minute.
     */
    public void setVelocity(double setpoint);

    /**
     * Cuts all power to the motor.
     */
    public void stop();
}
