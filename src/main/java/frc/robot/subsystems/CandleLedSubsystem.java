package frc.robot.subsystems;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer; //Timer.getFPGATimestamp()
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.Color;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;


public class CandleLedSubsystem extends SubsystemBase {

    public enum LedMode {
        OFF,
        SOLID,
        RAINBOW,
        BLINK
    }

    private final int canID = 33;
    private final int ledCount = 72 ;

    private CANdle candle;

    private LedMode currentMode = LedMode.OFF;

    //solid color
    private RGBWColor solidColor = OFF;

    //BLINK-SPECIFIC
    private double blinkInterval = 0.5;
    private boolean blinkState = false;
    private double lastBlinkTime = 0;
    private RGBWColor blinkColor = OFF;

    //pre-defined colors
    public static final RGBWColor RED = new RGBWColor(255, 0, 0);
    public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
    public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
    public static final RGBWColor YELLOW = new RGBWColor(255, 180, 0);
    public static final RGBWColor PURPLE = new RGBWColor(160, 0, 255);
    public static final RGBWColor ORANGE = new RGBWColor(255, 80, 0);
    public static final RGBWColor WHITE = new RGBWColor(255, 255, 255);
    public static final RGBWColor OFF = new RGBWColor(0, 0, 0);

    //other subsystems we can access to get their states
    RebuiltShooterSubsystem shooter;

    public CandleLedSubsystem(RebuiltShooterSubsystem shooter) {
        candle = new CANdle(canID);

        this.shooter=shooter;

        //set up our config
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.7; //global brightness modifier

        //If lights aren't being set to the right colors, change the StripType
        config.LED.StripType = com.ctre.phoenix6.signals.StripTypeValue.GRB;

        //apply the config
        candle.getConfigurator().apply(config);

    }

    private void setSolidColor(int r, int g, int b) {
        currentMode = LedMode.SOLID;
        RGBWColor color = new RGBWColor(r, g, b);

        SolidColor c = new SolidColor(0,ledCount-1).withColor(color);
        candle.setControl(c);
    }

    private void setSolidColor(RGBWColor color){
        currentMode = LedMode.SOLID;
        SolidColor c = new SolidColor(0, ledCount -1).withColor(color);
        candle.setControl(c);
    }

    private void turnOff() {
        currentMode = LedMode.OFF;
        setSolidColor(OFF);
    }

    


    @Override
    public void periodic() {

        if(DriverStation.isAutonomousEnabled()) return;

        if(shooter.closeToTargetRPM()){
            RainbowAnimation rainbow = new RainbowAnimation(0, ledCount);
            candle.setControl(rainbow);
        }

        if(DriverStation.isTeleopEnabled()){
            if(isRedAlliance()){
                setSolidColor(RED);
            }else{
                setSolidColor(BLUE);
            }
        }else if(DriverStation.isTeleop()){
            setSolidColor(GREEN);
        }




        //handle states
    }



    
    /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    //return false;
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  private void handleSolidColor(){
    SolidColor c = new SolidColor(0, ledCount -1).withColor(solidColor);
    candle.setControl(c);
  }

  private void handleOff(){
    SolidColor c = new SolidColor(0, ledCount -1).withColor(OFF);
    candle.setControl(c);
  }

  private void handleBlinkingAnimation(){

    //update timers
    if(Timer.getFPGATimestamp() > lastBlinkTime + blinkInterval){
        blinkState = !blinkState;
        lastBlinkTime = Timer.getFPGATimestamp();
    }

    //turn the light on or off
    if(blinkState){
        setSolidColor(blinkColor);
    }else{
        setSolidColor(OFF);
    }

  }




}