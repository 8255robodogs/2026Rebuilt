package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Value;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkClosedLoopController;
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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HarvestorSubsystem extends SubsystemBase{

    //motor settings
    private final SparkFlex motorLeft;
    private final SparkFlex motorRight;
    private final int motorLeftCanId = 4;
    private final int motorRightCanId = 7;

    //pid settings for maintaining rotation speed
    private static final double BASE_RPM = 2000;
    private double targetRPM = BASE_RPM;
    private double rpmErrorTolerance = 50;
    private final PIDController velocityPID = new PIDController(0.0004, 0.0, 0.0);
    
    //pressure control module (PCM)
    private int pcmCanId = Constants.pcmCanId;

    //piston
    public DoubleSolenoid piston;
    private int pistonForwardPort = 2;
    private int pistonReversePort = 3;

    //Constructor
    public HarvestorSubsystem(){
        //motors
        motorLeft = new SparkFlex(motorLeftCanId, MotorType.kBrushless);
        motorRight = new SparkFlex(motorRightCanId, MotorType.kBrushless);
        velocityPID.setTolerance(rpmErrorTolerance);

        //piston
        piston = new DoubleSolenoid(
            pcmCanId,
            PneumaticsModuleType.CTREPCM,
            pistonForwardPort,
            pistonReversePort );
    }




    //use this to set the motor speeds at once. notice the right motor is inverted
    private void setMotorSpeeds(double speed){
        motorLeft.set(speed);
        motorRight.set(speed*-1);
    }


    //stop both motors
    private void stopMotor() {
    setMotorSpeeds(0);
    velocityPID.reset();
    }


    private double getRPM(){
        return motorLeft.getEncoder().getVelocity();
    }


    @Override
    public void periodic(){
        //write information to the dashboard
        SmartDashboard.putNumber("Intake set RPM", targetRPM);
        SmartDashboard.putNumber("Intake real RPM", getRPM());
        SmartDashboard.putBoolean("Intake Extended", piston.get() == DoubleSolenoid.Value.kForward);
    }

    


    //Wheel commands

    public Command StartIntake() {
    return run(
        () -> {
            double output = velocityPID.calculate(getRPM(), targetRPM);
            output = MathUtil.clamp(output, -1.0, 1.0);
            setMotorSpeeds(output);
        }
    );
    }

    public Command StartIntakeBackwards(){
        return run(() -> {
            setMotorSpeeds(-0.3);
        }
        );
    }

    public Command StopIntake(){
        return Commands.runOnce(() -> {
            stopMotor();
        });
    }


    public Command ModifyIntakeTargetRPMs(double rpmModification){
    return runOnce(() -> {
        targetRPM = MathUtil.clamp(targetRPM + rpmModification, 0, 6000);
    });
}




    




    //piston commands

    public Command Extend(){
        return runOnce(() ->{
            piston.set(DoubleSolenoid.Value.kForward);
        });
    }

    public Command Retract(){
        return runOnce(() ->{
            piston.set(DoubleSolenoid.Value.kReverse);
        });
    }

    public Command TogglePistonState(){
        return runOnce(()->{
            piston.toggle();
        });
    }






}
