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
    public LEDSubsystem() {
        LED = new AddressableLED(LEDConstants.PWM);
        LEDBuffer = new AddressableLEDBuffer(LEDConstants.LED_BUFFER);
        LED.setLength(LEDBuffer.getLength());
        LED.setData(LEDBuffer);
        LED.start();
    }
    public void isAligned(boolean isAlign){
        if(isAlign){
            setGreeen();
        }else{
            setRed();
        }
    }
    private void setGreeen(){
        LEDPattern green = LEDPattern.solid(Color.kGreen);
        green.applyTo(LEDBuffer);
        LED.setData(LEDBuffer);
    }
    private void setRed(){
        LEDPattern red = LEDPattern.solid(Color.kRed);
        red.applyTo(LEDBuffer);
        LED.setData(LEDBuffer);
    }
    private void setBlueYellow(){
        LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kBlue);
        gradient.applyTo(LEDBuffer);
        LED.setData(LEDBuffer);
    }
    public void periodic() {
        if (DriverStation.isDisabled()) {
            setBlueYellow();
        }
    }
}
