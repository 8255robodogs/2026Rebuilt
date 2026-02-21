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
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;
import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;


public class RebuiltShooterSubsystem extends SubsystemBase{


    //CAN IDs
    private final int leftShooterMotorCanId = 17;
    private final int rightShooterMotorCanId = 19;
    private final int conveyorMotorCanId = 5;
    private final int kickerMotorCanId = 11;

    //conveyor tuning
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
    private double rpmTarget = 0; //temporary hardcode
    private double autoRpmTarget = 2000;
    private double lowRpmTarget = 3000;
    private double highRpmTarget = 4500;
    private double manualRpmTarget = 2000;

    //Mode
    private Boolean shooterPowered = false;
    private Boolean feeding = false;

    //simulation
    private FlywheelSim shooterSim;
    private SparkMaxSim leftShooterSim;
    private SparkMaxSim rightShooterSim;
    private final double gearRatio = 1.0;          
    private final double momentOfInertia = 0.002; 


    //reference to the drivebase so we can check our position
    SwerveSubsystem drivebase;


    //Constructor
    public RebuiltShooterSubsystem(SwerveSubsystem drivebase){
        leftShooterMotor = new SparkMax(leftShooterMotorCanId,MotorType.kBrushless);
        rightShooterMotor = new SparkMax(rightShooterMotorCanId,MotorType.kBrushless);
        shooterEncoder = leftShooterMotor.getEncoder();
        conveyorMotor = new VictorSPX(conveyorMotorCanId);
        kickerMotor = new SparkMax(kickerMotorCanId, MotorType.kBrushless);

        pid = new PIDController(p, i, d);
        pid.setSetpoint(rpmTarget);
        pid.setTolerance(pidErrorTolerance);



        this.drivebase = drivebase;




        //simulation
        LinearSystem<N1, N1, N1> flywheelPlant =
            LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(2),   // 2 motors
            momentOfInertia,
            gearRatio
        );

        leftShooterSim = new SparkMaxSim(leftShooterMotor, DCMotor.getNEO(1));
        rightShooterSim = new SparkMaxSim(rightShooterMotor, DCMotor.getNEO(1));

        shooterSim = new FlywheelSim(
            flywheelPlant,
            DCMotor.getNEO(2)
        );



        


    }



    @Override
    public void periodic(){

        //update internal values
        rpmCurrent = shooterEncoder.getVelocity();
        

        //update rpmTarget based on distance to the goal
        if(drivebase.distanceToMyTarget() > 0){
            autoRpmTarget = 1000;
        }
        if(drivebase.distanceToMyTarget() > 1){
            autoRpmTarget = 1500;
        }
        if(drivebase.distanceToMyTarget() > 2){
            autoRpmTarget = 1800;
        }
        //TODO add additional values. eventually, change this to a formula.
        //TODO CLAMP VALUES



        //update values for dashboard
        SmartDashboard.putNumber("Shooter RPM Current", rpmCurrent);
        SmartDashboard.putNumber("Shooter RPM Target", rpmTarget);
        SmartDashboard.putBoolean("Conveyor", feeding);
        SmartDashboard.putNumber("Kicker RPM Current", kickerMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Low RPM for shooter", lowRpmTarget);
        SmartDashboard.putNumber("High RPM for shooter", highRpmTarget);
        SmartDashboard.putNumber("Auto RPM for shooter", autoRpmTarget);
        SmartDashboard.putNumber("Distance To Target", drivebase.distanceToMyTarget());
        SmartDashboard.putBoolean("Shooter Power", shooterPowered);
        SmartDashboard.putBoolean("RPM READY", pid.atSetpoint());
        SmartDashboard.putNumber("Manual RPM for Shooter", manualRpmTarget);

    }




    //simulation only
    @Override
    public void simulationPeriodic() {

        // Use left motor as leader reference
        double appliedVoltage = leftShooterMotor.getAppliedOutput() * 12.0;

        // Feed voltage into physics model
        shooterSim.setInputVoltage(appliedVoltage);

        // Advance simulation by 20ms
        shooterSim.update(0.02);

        // Convert rad/s to RPM
        double simRPM = Units.radiansPerSecondToRotationsPerMinute(
            shooterSim.getAngularVelocityRadPerSec()
        );

        // Push velocity into SparkMax simulated encoders
        leftShooterSim.setVelocity(simRPM);
        rightShooterSim.setVelocity(simRPM);
    }






     //calculate the shooter motor power needed with pidf
        private void getShooterToRpm(double rpm){
            rpmTarget = rpm;
            shooterPowered = true;
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

        }
        
        private void stopShooter(){
            shooterPowered = false;
            leftShooterMotor.set(0);
            rightShooterMotor.set(0); 
        }












    //shooting commands
   
    public Command ShootAtAutoRpm(){
        pid.reset();
        return Commands.run(()->{
            getShooterToRpm(autoRpmTarget);
        })
        .finallyDo(()->{
            stopShooter();
        });
    }

    public Command ShootAtLowRpm(){
        pid.reset();
        return Commands.run(()->{
            getShooterToRpm(lowRpmTarget);
        })
        .finallyDo(()->{
            stopShooter();
        });
    }

    public Command ShootAtHighRpm(){
        pid.reset();
        return Commands.run(()->{
            getShooterToRpm(highRpmTarget);
        })
        .finallyDo(()->{
            stopShooter();
        });
    }

    public Command ShootAtManualRpm(){
        pid.reset();
        return Commands.run(()->{
            getShooterToRpm(manualRpmTarget);
        })
        .finallyDo(()->{
            stopShooter();
        });
    }
    
    public Command StopShooting(){
        pid.reset();
        return Commands.runOnce(()->{
            stopShooter();
        });
    }

    public Command ModifyManualShootingRPM(double modifier){
        pid.reset();
        return Commands.runOnce(()->{
            manualRpmTarget += modifier;
            manualRpmTarget = MathUtil.clamp(manualRpmTarget, -1000, 6000);
        });
    }


    public Command GetShooterUpToAutoRpmSpeed(){
        pid.reset();
        return Commands.run(()-> {
            getShooterToRpm(autoRpmTarget);
        }
        ).until(pid::atSetpoint);
    }




    //conveyor commands
    
    public Command FeedShooter(){
        return Commands.run(()->{
            feeding = true;
            conveyorMotor.set(VictorSPXControlMode.PercentOutput, conveyorSpeed);
            kickerMotor.set(kickerSpeed);
        }
        ).finallyDo(()->{
            feeding=false;
            conveyorMotor.set(VictorSPXControlMode.PercentOutput, 0);
            kickerMotor.set(0);
        });    
    }



    


}