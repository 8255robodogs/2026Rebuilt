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

public class RebuiltClimberSubsystem extends SubsystemBase{

    //pressure control module (PCM)
    private int pcmCanId = Constants.pcmCanId;

    //piston
    public DoubleSolenoid piston;
    private int pistonForwardPort = 3;
    private int pistonReversePort = 2;

    //Constructor
    public RebuiltClimberSubsystem(){
        piston = new DoubleSolenoid(
            pcmCanId,
            PneumaticsModuleType.CTREPCM,
            pistonForwardPort,
            pistonReversePort );
    }






    @Override
    public void periodic(){
        //write information to the dashboard
        SmartDashboard.putBoolean("Climber Up", piston.get() == DoubleSolenoid.Value.kForward);
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