package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Value;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;
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


    //CAN IDs
    private final int leftShooterMotorCanId = 17;
    private final int rightShooterMotorCanId = 19;
    private final int conveyorMotorCanId = 5;
    private final int kickerMotorCanId = 11;

    //Tuning
    private double conveyorSpeed = 0.8; //This can be anywhere from -1 to 1. Decimals work, such as 0.8
    private double kickerSpeed = -0.8;


    //PID settings
    private PIDController pid;
    private final double p = 0.0001;
    private final double i = 0.0;
    private final double d = 0.00;
    private final double pidErrorTolerance = 100;
    private final double kV = 0.00015;  // initial guess


    //Motor objects
    private SparkMax leftShooterMotor;
    private SparkMax rightShooterMotor;
    private RelativeEncoder shooterEncoder;
    private VictorSPX conveyorMotor;
    private SparkMax kickerMotor;

    //Shooter data
    private double rpmCurrent = 0;
    private double rpmTarget = 3800; //temporary hardcode

    //Mode
    private Boolean shooting = false;
    private Boolean feeding = false;



    //Constructor
    public RebuiltShooterSubsystem(){
        leftShooterMotor = new SparkMax(leftShooterMotorCanId,MotorType.kBrushless);
        rightShooterMotor = new SparkMax(rightShooterMotorCanId,MotorType.kBrushless);
        shooterEncoder = leftShooterMotor.getEncoder();
        conveyorMotor = new VictorSPX(conveyorMotorCanId);
        kickerMotor = new SparkMax(kickerMotorCanId, MotorType.kBrushless);

        pid = new PIDController(p, i, d);
        pid.setSetpoint(rpmTarget);
        pid.setTolerance(pidErrorTolerance);

    }



    @Override
    public void periodic(){

        //update internal values
        rpmCurrent = shooterEncoder.getVelocity();
        

        //update rpmTarget based on distance to the goal
        //TODO after limelight is working




        //update values for dashboard
        SmartDashboard.putNumber("Shooter RPM Current", rpmCurrent);
        SmartDashboard.putNumber("Shooter RPM Target", rpmTarget);


        //calculate the shooter motor power needed with pidf
        if(shooting){
            double pidOutput = 0;
            double feedforward = 0;
            pid.setSetpoint(rpmTarget);

            pidOutput = pid.calculate(rpmCurrent);
            feedforward = kV * rpmTarget;
            double totalOutput = pidOutput + feedforward;
            totalOutput = MathUtil.clamp(totalOutput, -1, 1);

            //apply the power to the shooter motors
            leftShooterMotor.set(totalOutput);
            rightShooterMotor.set(-totalOutput);
        
            if (shooting && pid.atSetpoint() ){
                
            }

        }else{
            leftShooterMotor.set(0);
            rightShooterMotor.set(0); 
        }
        

        if(feeding){
            conveyorMotor.set(VictorSPXControlMode.PercentOutput, conveyorSpeed);
            kickerMotor.set(kickerSpeed);
        }else{
            conveyorMotor.set(VictorSPXControlMode.PercentOutput, 0);
            kickerMotor.set(0);
        }


        

    }

   
    public Command BeginShooting(){
        pid.reset();
        return Commands.runOnce(()->beginShooting());
    }

    private void beginShooting(){
        shooting = true;
    }



    
    public Command StopShooting(){
        pid.reset();
        return Commands.runOnce(()->stopShooting());
    }

    private void stopShooting(){
        shooting = false;
    }

    


    public Command BeginFeedingShooter(){
        return Commands.runOnce(()->beginFeedingShooter());
    }

    private void beginFeedingShooter(){
        feeding = true;
    }


    public Command StopFeedingShooter(){
        
        return Commands.runOnce(()->stopFeedingShooter());
    }

    private void stopFeedingShooter(){
        feeding = false;
    }







    

    


}



