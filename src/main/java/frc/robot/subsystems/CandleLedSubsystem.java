package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer; //Timer.getFPGATimestamp()
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.awt.Color;

import org.w3c.dom.css.RGBColor;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
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

    private CANdle candle;

    private LedMode currentMode = LedMode.OFF;

    private final int ledCount = 72 ;

    private double blinkInterval = 0.5;
    private boolean blinkState = false;
    private double lastBlinkTime = 0;

    public CandleLedSubsystem() {
        candle = new CANdle(canID);

        //set up our config
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.BrightnessScalar = 0.7;
        config.LED.StripType = com.ctre.phoenix6.signals.StripTypeValue.GRB;

        //rgb
        //BRG


        //apply the config
        candle.getConfigurator().apply(config);

    }

    // =============================
    // Public Control Methods
    // =============================

    public void setSolidColor(int r, int g, int b) {
        currentMode = LedMode.SOLID;
        RGBWColor color = new RGBWColor(r, g, b);

        SolidColor c = new SolidColor(0,ledCount-1).withColor(color);
        candle.setControl(c);
        
    }


    public void turnOff() {
        currentMode = LedMode.OFF;
        setSolidColor(0,0,0);
    }



    @Override
    public void periodic() {

        if(DriverStation.isTeleopEnabled()){
            setSolidColor(0, 50, 0);
        }else if(DriverStation.isTeleop()){
            if(isRedAlliance()){
                setSolidColor(50, 0, 0);
            }else{
                setSolidColor(0, 0, 50);
            }
        }

        
        

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



}