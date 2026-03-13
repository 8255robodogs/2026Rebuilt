package frc.robot.subsystems;



import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class HarvestorSubsystem extends SubsystemBase{

    //motor settings
    private final SparkMax motorLeft;
    private final SparkMax motorRight;
    private final int motorLeftCanId = 4;
    private final int motorRightCanId = 3;
    private boolean harvestorRunning = false;

    //pid settings for maintaining rotation speed
    private double harvesterFixedSpeed = .8;
    private double harvesterReverseSpeed = -.8;

    private static final double BASE_RPM = 1600;
    private double targetRPM = BASE_RPM;
    private double rpmErrorTolerance = 200;
    private final PIDController velocityPID = new PIDController(0.0001, 0.0, 0.0);
    
    //pressure control module (PCM)
    private int pcmCanId = Constants.pcmCanId;

    //piston
    public DoubleSolenoid piston;
    private int pistonForwardPort = 1;
    private int pistonReversePort = 0;

    //Constructor
    public HarvestorSubsystem(){
        //motors
        motorLeft = new SparkMax(motorLeftCanId, MotorType.kBrushless);
        motorRight = new SparkMax(motorRightCanId, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(motorLeftCanId, true);
        //motorRight.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);


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
        //set both motors to the same. we will handle reversing one in the motor controller's configuration
        motorLeft.set(speed);
        motorRight.set(speed);
        harvestorRunning = true;
    }


    //stop both motors. call this when other commands end so we have a consistent way of stopping things.
    private void stopMotor() {
        setMotorSpeeds(0);
        velocityPID.reset();
        harvestorRunning = false;
    }


    private double getRPM(){
        return motorLeft.getEncoder().getVelocity();
    }


    @Override
    public void periodic(){
        //write information to the dashboard
        SmartDashboard.putNumber("Intake real RPM", getRPM());

        SmartDashboard.putNumber("Int L RPM", motorLeft.getEncoder().getVelocity());
        SmartDashboard.putNumber("Int R RPM", motorRight.getEncoder().getVelocity());

        SmartDashboard.putNumber("Int L Current", motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("Int R Current", motorRight.getOutputCurrent());

        

        SmartDashboard.putBoolean("Intake Extended", piston.get() == DoubleSolenoid.Value.kForward);
        SmartDashboard.putBoolean("Harvester", harvestorRunning);
        SmartDashboard.putNumber("HarvesterLeftCurrent", motorLeft.getOutputCurrent());
        SmartDashboard.putNumber("HarvesterRightCurrent", motorRight.getOutputCurrent());
        
    }

    


    //Wheel commands

    public Command IntakeWithFixedPower() {
    return run(
        () -> {
            intakeWithFixedPower();
        }
    ).finallyDo(()-> stopMotor());
    }

    public Command IntakeWithPID(){
        return run(()->{
            intakeWithPID();
        }).finallyDo(()->stopMotor());

    }

    public Command ReverseHarvester(){

        return run(
            ()->{
                setMotorSpeeds(harvesterReverseSpeed);
            }
        ).finallyDo(()->{
            stopMotor();
        });
    }
    
    private void intakeWithFixedPower(){
        setMotorSpeeds(harvesterFixedSpeed);
    }

    private void intakeWithPID(){
        //calcualte values with pid
        double output = velocityPID.calculate(getRPM(), targetRPM);
            
        //apply feed forward
        double feedforward = .6;
        output += feedforward;

        //clamp values
        output = MathUtil.clamp(output, 0, 1);
        SmartDashboard.putNumber("Harvester PID Output", output);
        setMotorSpeeds(output);
    }



    public Command PathPlannerStartIntake(){
        return runOnce(()->{
            setMotorSpeeds(harvesterFixedSpeed);
        });
    }

    public Command PathPlannerStopIntake(){
        return runOnce(()->{
            setMotorSpeeds(0);
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