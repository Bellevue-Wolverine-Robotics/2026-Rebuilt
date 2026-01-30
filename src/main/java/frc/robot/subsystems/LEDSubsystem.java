package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants.LEDConstants;

public class LEDSubsystem {
    private final AddressableLED LED;
    private final AddressableLEDBuffer LEDBuffer;
    private LEDPattern pattern;
    public LEDSubsystem() {
        LED = new AddressableLED(LEDConstants.PWM);
        LEDBuffer = new AddressableLEDBuffer(LEDConstants.LED_BUFFER);
        LED.setLength(LEDBuffer.getLength());
        LED.setData(LEDBuffer);
        pattern = LEDPattern.solid(Color.kRed);
        LED.start();
    }
    public void isAligned (boolean isAlign) {
        if(isAlign){
            pattern = LEDPattern.solid(Color.kGreen);
        }else{
            pattern = LEDPattern.solid(Color.kRed);
        }
    }

    private void setBlueYellow() {
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        gradient.applyTo(LEDBuffer);
        LED.setData(LEDBuffer);
    }
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setBlueYellow();
        }else{
            pattern.applyTo(LEDBuffer);
            LED.setData(LEDBuffer);
        }
    }
}
