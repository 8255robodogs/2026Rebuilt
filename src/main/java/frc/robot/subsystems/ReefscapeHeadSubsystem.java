package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefscapeHeadSubsystem extends SubsystemBase{

    //Motor settings
    private VictorSPX motor;
    private int motorControllerCanID = 14;
    private boolean invertedMotor = false;
    
    //light beam
    private int lightBeamSensorDIOportNumber = 1;
    private DigitalInput lightBeamSensor;

    

    //Constructor
    public ReefscapeHeadSubsystem(){
        motor = new VictorSPX(motorControllerCanID);
        motor.setNeutralMode(NeutralMode.Brake);
        lightBeamSensor = new DigitalInput(lightBeamSensorDIOportNumber);
    }

    public void setMotorSpeed(double speed){
        double newSpeed = speed;
        if(invertedMotor == true){
            newSpeed = newSpeed *-1;
        }
        motor.set(VictorSPXControlMode.PercentOutput, newSpeed);
    }


    @Override
    public void periodic(){

    }

    public boolean getBeamBroken(){
        return lightBeamSensor.get();
    }

    public Command setHeadSpeedDefault(double speed){
        return Commands.run(() -> setMotorSpeed(speed),this);
    }

    public Command setHeadSpeed(double speed){
        return Commands.runOnce(() -> setMotorSpeed(speed),this);
    }



}



