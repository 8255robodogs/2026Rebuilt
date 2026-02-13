package frc.robot;

import frc.robot.Constants.OperatorConstants;


//import subsystems
import frc.robot.subsystems.RebuiltShooterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

//import yagsl
import swervelib.SwerveInputStream;

//import pathplanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

//misc imports
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;




public class RobotContainer {
  

  //used for DPAD driving
  public double creepSpeed = 0.2;


  // The robot's subsystems and commands are defined here...
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(limelight);
  private final RebuiltShooterSubsystem shooter = new RebuiltShooterSubsystem();
  private final HarvestorSubsystem harvestor = new HarvestorSubsystem();

  
  //declare the controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  //Setting up swerve drive
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
    () -> m_driverController.getLeftY() *-1,
    () -> m_driverController.getLeftX() *-1)
    .withControllerRotationAxis(m_driverController::getRightX)
    .deadband(OperatorConstants.kDriverStickDeadband)
    .scaleTranslation(1)
    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(m_driverController::getRightX, 
    m_driverController::getRightY)
    .headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  
  CameraServer cameraServer;
  
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();




  public RobotContainer() {
    //Explains the robot to pathplanner so it can be used to drive paths.
    drivebase.setupPathPlanner();
    
    //create the camera
    //CameraServer.addServer("cams");
    //CameraServer.startAutomaticCapture();

    
    //registers controls
    configureBindings();
    configureAutos();
  }
  
  private void configureBindings() {
    
    //-----DRIVER CONTROLLER (CONTROLLER ZERO)-----

    //DRIVING CONTROLS

    //this makes the drivebase drive. Very important.
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    //hold b to lock the wheels and resist being pushed
    m_driverController.b().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    //press start to zero your heading
    m_driverController.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
    

    //DPAD CREEPING
    m_driverController.povRight().whileTrue(drivebase.driveRobotRelativeCommand(0.0, creepSpeed, 0.0));
    m_driverController.povLeft().whileTrue(drivebase.driveRobotRelativeCommand(0.0, -creepSpeed, 0.0));
    m_driverController.povUp().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, 0, 0.0));
    m_driverController.povDown().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, 0, 0.0));
    
    m_driverController.povDownRight().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, creepSpeed, 0.0));
    m_driverController.povDownLeft().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, -creepSpeed, 0.0));
    m_driverController.povUpRight().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, creepSpeed, 0.0));
    m_driverController.povUpLeft().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, -creepSpeed, 0.0));




    //-----OPERATOR CONTROLLER CONTROLLER (CONTROLLER ONE)-----

    //Shooter
    m_operatorController.y().onTrue(shooter.BeginShooting());
    m_operatorController.y().onFalse(shooter.StopShooting());
    
    //Harvestor
    m_driverController.a().onTrue(harvestor.setHeadSpeed(.4));
    m_driverController.a().onFalse(harvestor.setHeadSpeed(0));


  } 

  private void configureAutos(){
    
    m_autoChooser.setDefaultOption("Left", "Left");
    m_autoChooser.addOption("Middle", "Middle");
    m_autoChooser.addOption("MiddleLeft", "MiddleLeft");
    m_autoChooser.addOption("MiddleRight", "MiddleRight");
    m_autoChooser.addOption("BlueLeft","BlueLeft");
    SmartDashboard.putData("Auto Choices", m_autoChooser);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
/*
    if("Left".equals(m_autoChooser.getSelected())){
      return 
      AutoBuilder.buildAuto("Test Auto")
      .alongWith(elevator.setLevel(4))
      .alongWith(head.setHeadSpeed(0))
      
      .andThen(head.setHeadSpeed(.7))
      .andThen(new WaitCommand(1))
      .andThen(head.setHeadSpeed(0))
      
      .andThen(elevator.setLevel(1))
      .andThen(AutoBuilder.buildAuto("Human Left"))
      
      .andThen(new WaitCommand(1))
      
      .andThen(AutoBuilder.buildAuto("Second Score"))
  
      //.andThen(head.setHeadSpeed(.2))
      
      ;
    }


*/

   
    return Commands.none();
    

  }
}
