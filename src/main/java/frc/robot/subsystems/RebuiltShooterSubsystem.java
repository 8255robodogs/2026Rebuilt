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

public class RebuiltShooterSubsystem extends SubsystemBase{


    //Motor settings
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private int leftMotorCanId = 17;
    private int rightMotorCanId = 19;


    //PID settings
    private PIDController pid;
    private final double p = 0.1;
    private final double i = 0.0;
    private final double d = 0.01;
    private final double pidErrorTolerance = 0.2;

    //Shooter Tuning
    private int rpmCurrent = 0;
    private int rpmTarget = 0;

    //Constructor
    public RebuiltShooterSubsystem(){
        leftMotor = new SparkMax(leftMotorCanId,MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorCanId,MotorType.kBrushless);

        pid = new PIDController(p, i, d);
        pid.setSetpoint(rpmTarget);
        pid.setTolerance(pidErrorTolerance);
    }



    @Override
    public void periodic(){



        //update values for shuffleboard
        SmartDashboard.putNumber("RPM Target", rpmTarget);


    }

   private int getCurrentRpm(){
    rightMotor


    return 0;
   }
    


    public Command setRpmTarget(int target){    
        return Commands.runOnce(()-> SetRpmTarget(target));
    }

    private void SetRpmTarget(int target){
        rpmTarget = target;
    }








    //remove later
    public Command setLevel(int level1to4){    
        return Commands.runOnce(()-> SetRpmTarget(level1to4));
    }

    


}



