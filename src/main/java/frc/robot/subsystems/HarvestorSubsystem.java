package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Value;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
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

public class HarvestorSubsystem extends SubsystemBase{

    //Motor settings
    private SparkFlex motorLeft;
    private SparkFlex motorRight;
    private int motorLeftCanId = 4;
    private int motorRightCanId = 7;
    
    private double motorSpeed = 0.75;
    

    //Constructor
    public HarvestorSubsystem(){
        motorLeft = new SparkFlex(motorLeftCanId,MotorType.kBrushless);
        motorRight = new SparkFlex(motorRightCanId,MotorType.kBrushless);
        
    }

    public void setMotorSpeed(double speed){
        motorLeft.set(speed);
        motorRight.set(speed * -1);
    }


    @Override
    public void periodic(){

    }

    

    public Command EnableHarvester(){
        return Commands.runOnce(() -> setMotorSpeed(motorSpeed),this);
    }

    public Command DisableHarvester(){
        return Commands.runOnce(() -> setMotorSpeed(0),this);
    }



}
