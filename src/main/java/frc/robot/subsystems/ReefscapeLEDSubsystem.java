package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Random;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ReefscapeLEDSubsystem extends SubsystemBase {
    
    private AddressableLED ledStrip;  
    private int ledStripPWMportNumber = 1;
    private int numPixels = 64;    // The number of addressable LEDs in the strip
    private AddressableLEDBuffer buffer;


    Random r = new Random();

    // Constructor to initialize the LED strip
    public ReefscapeLEDSubsystem() {
        this.ledStrip = new AddressableLED(ledStripPWMportNumber);
        this.ledStrip.setLength(numPixels);
        this.buffer = new AddressableLEDBuffer(numPixels);

        

        //set the lights on startup
        for(int i=0;i<buffer.getLength();i++){
            buffer.setLED(i, Color.kBlue);
        }
        this.ledStrip.setData(buffer);

    }


    @Override
    public void periodic() {

        /* 
        if(DriverStation.getMatchTime() > 0 && DriverStation.getMatchTime() < 19){
            //last 19 seconds of match
            setColor(Color.kPurple);


        }else if(DriverStation.getMatchTime() >= 19 && DriverStation.getMatchTime() <20){
            //the second that the 20 second horn blows
            randomizeAllColors();

        }else{
            //all other times
            setColorByElevatorLevel();
        }
        */


        //apply the buffer data to the led strip
        this.ledStrip.setData(buffer);
    }


    

    private void randomizeAllColors(){
        for(int i=0;i<buffer.getLength();i++){
            buffer.setRGB(i, r.nextInt(200), r.nextInt(200), r.nextInt(200));
        }
    }

    private void setColor(Color color){
        for(int i=0;i<buffer.getLength();i++){
            buffer.setLED(i, color);
        }
    }


}
