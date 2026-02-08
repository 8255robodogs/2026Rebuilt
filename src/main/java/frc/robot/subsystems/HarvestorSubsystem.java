package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Value;

import java.util.Map;
import java.util.function.BooleanSupplier;

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
    private SparkMax motor;
    private int motorControllerCanID = 4;
    private boolean invertedMotor = true;
    

    

    //Constructor
    public HarvestorSubsystem(){
        motor = new SparkMax(motorControllerCanID,MotorType.kBrushless);
    }

    public void setMotorSpeed(double speed){
        double newSpeed = speed;
        if(invertedMotor == true){
            newSpeed = newSpeed *1;
        }
        motor.set(newSpeed);
    }


    @Override
    public void periodic(){

    }

    public Command setHeadSpeedDefault(double speed){
        return Commands.run(() -> setMotorSpeed(speed),this);
    }

    public Command setHeadSpeed(double speed){
        return Commands.runOnce(() -> setMotorSpeed(speed),this);
    }



}
