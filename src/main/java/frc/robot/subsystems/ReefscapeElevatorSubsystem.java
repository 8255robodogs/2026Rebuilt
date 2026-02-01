package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Value;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ReefscapeElevatorSubsystem extends SubsystemBase{

    private int level = 1;

    //Motor settings
    private SparkMax motor;
    private int motorControllerCanID = 9;
    private boolean invertedMotor = true;

    //limit switch settings
    private int limitSwitchDIOportNumber = 0;
    private DigitalInput limitSwitch = new DigitalInput(limitSwitchDIOportNumber);
    

    //PID settings
    private PIDController pid;
    private final double p = 0.1;
    private final double i = 0.0;
    private final double d = 0.01;
    private final double pidErrorTolerance = 0.2;

    //Elevator Height Presets
    private final double heightL4 = 180;
    private final double heightL3 = 78;
    private final double heightL2 = 25.5;
    private final double heightL1 = 0;

    


    //Constructor
    public ReefscapeElevatorSubsystem(){
        motor = new SparkMax(motorControllerCanID,MotorType.kBrushless);
        
        pid = new PIDController(p, i, d);
        pid.setSetpoint(heightL1);
        pid.setTolerance(pidErrorTolerance);
    }

    public void setMotorSpeed(double speed){
        double newSpeed = speed;
        if(invertedMotor == true){
            newSpeed = newSpeed *-1;
        }
        motor.set(newSpeed);
    }

    private double getHeight(){
        double heightToReturn = motor.getEncoder().getPosition();
        if(invertedMotor){
            heightToReturn = heightToReturn * -1;
        }
        return heightToReturn;
    }

    private void resetEncoder(){
        motor.getEncoder().setPosition(0);
    }

    public boolean getLimitSwitchHit(){
        if(limitSwitch.get()){
            return true;
        }else{
            return false;
        }
    }

    


    @Override
    public void periodic(){
        
        //when we are at the bottom, we reset our encoder to let it know our true zero
        if(getLimitSwitchHit()){
            resetEncoder();
        }

        //handle movement.
        if(level == 1){
            moveTowardsBottom();
        }else{
            moveTowardsSetpoint();
        }

        
        //update values for shuffleboard
        SmartDashboard.putNumber("elevatorLevel", level);
        SmartDashboard.putNumber("elevatorHeight", getHeight());
        SmartDashboard.putBoolean("elevatorLimitHit", getLimitSwitchHit());
    }

    private void moveTowardsBottom(){
        if(getLimitSwitchHit()){
            //we are already at the bottom, apply a small voltage to hold the elevator in place
            setMotorSpeed(-0.05);
        }else{
            //we are trying to move to the bottom. we do the last bit a little slower.
            if(getHeight() > 10){
                setMotorSpeed(-1);
            }else{
                setMotorSpeed(-0.5);
            }
        }
    }

    private void moveTowardsSetpoint(){
        //if(pid.atSetpoint() == false){
            setMotorSpeed(pid.calculate(getHeight()));
        //}else{
            //setMotorSpeed(0);
        //}
    }

    public int getLevel(){
        return this.level;
    }

    
    private void setSetPoint(int level1to4){
        switch (level1to4){
            case 1:
                this.level = 1;
                pid.setSetpoint(heightL1);
                break;
            case 2:
                this.level = 2;
                pid.setSetpoint(heightL2);
                break;
            case 3:
                this.level = 3;
                pid.setSetpoint(heightL3);
                break;
            case 4:
                this.level = 4;    
                pid.setSetpoint(heightL4);
                break;
            default:
                System.out.println("Invalid level!");
                break;
        }
    }

    public Command setLevel(int level1to4){    
        return Commands.runOnce(()-> setSetPoint(level1to4));
    }

    


}



