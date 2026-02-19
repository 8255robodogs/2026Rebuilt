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
import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.button.Trigger;





public class RobotContainer {
  

  //used for DPAD driving
  public double creepSpeed = 0.2;


  // The robot's subsystems and commands are defined here...
  private final LimelightSubsystem limelight = new LimelightSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem(limelight);
  private final RebuiltShooterSubsystem shooter = new RebuiltShooterSubsystem(drivebase);
  private final HarvestorSubsystem harvestor = new HarvestorSubsystem();

  
  //declare the controllers
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
  //Setting up swerve drive
  SwerveInputStream driveAngularVelocity = 
    SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> -m_driverController.getLeftY(),
      () -> -m_driverController.getLeftX()
    )
    .withControllerRotationAxis(() -> MathUtil.applyDeadband(-m_driverController.getRightX(), 0.15))
    .deadband(OperatorConstants.kDriverStickDeadband)
    .scaleTranslation(1)
    .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
    .withControllerHeadingAxis(
    () -> MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriverStickDeadband),
    () -> MathUtil.applyDeadband(m_driverController.getRightY(), OperatorConstants.kDriverStickDeadband)
    ).headingWhile(true);

  Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  //used for camera
  CameraServer cameraServer;
  
  //used for selecting autos
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
    Trigger t_wheelLock = m_driverController.b();
    Trigger t_harvester = m_driverController.rightTrigger(0.5);
    Trigger t_conveyorAndKicker = m_driverController.leftTrigger(0.5);
    Trigger t_zeroGyro = m_driverController.start();
    Trigger t_resetPose = m_driverController.x();
    Trigger t_autoAim = m_driverController.rightStick();

    //-----OPERATOR CONTROLLER CONTROLLER (CONTROLLER ONE)-----
    Trigger t_shooterAutoSpeed = m_operatorController.x();
    Trigger t_shooterHighSpeed = m_operatorController.y();
    Trigger t_shooterLowSpeed = m_operatorController.a();
    Trigger t_shooterManualSpeed = m_operatorController.b();
    Trigger t_increaseShooterManualSpeed = m_operatorController.povUp();
    Trigger t_decreaseShooterManualSpeed = m_operatorController.povDown();

    Trigger t_increaseHarvesterRPM = m_operatorController.povRight();
    Trigger t_decreaseHarvesterRPM = m_operatorController.povLeft();
    Trigger t_extendHarvester = m_operatorController.rightBumper();
    Trigger t_retractHarvester = m_operatorController.leftBumper();


    //debug - testing auto drive to locations
    /*
    final Pose2d blueRight = new Pose2d(2,2,new Rotation2d(0));
    final Pose2d blueLeft = new Pose2d(2,4,new Rotation2d(0));
    m_driverController.rightStick().onTrue(drivebase.driveToPose(blueRight));
    m_driverController.leftStick().onTrue(drivebase.driveToPose(blueLeft));
    */

    //this makes the drivebase drive. Very important. This is the default command, a different drivebase command can override it.
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    //lock the wheels and resist being pushed
    t_wheelLock.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    //Lock onto the "ideal" target.
    //logic for that target is in the swervedrive subsystem.
    //if we are in our scoring area, its our hub. If we are outside our scoring area, its the nearest shuttle point.
    t_autoAim.whileTrue(
        drivebase.driveFacingTarget(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX()
        )
    );


    //zero your heading
    t_zeroGyro.onTrue(Commands.runOnce(drivebase::zeroGyroWithAlliance));
    

    //Harvestor
    t_harvester.whileTrue(harvestor.StartIntake());
    t_harvester.whileFalse(harvestor.StopIntake());
    t_increaseHarvesterRPM.onTrue(harvestor.ModifyIntakeTargetRPMs(250));
    t_decreaseHarvesterRPM.onTrue(harvestor.ModifyIntakeTargetRPMs(-250));
    t_extendHarvester.onTrue(harvestor.Extend());
    t_retractHarvester.onTrue(harvestor.Retract());


    //Conveyor & kicker
    t_conveyorAndKicker.whileTrue(shooter.BeginFeedingShooter());
    t_conveyorAndKicker.whileFalse(shooter.StopFeedingShooter());
    t_conveyorAndKicker.whileTrue(harvestor.AssistConveyor());
    t_conveyorAndKicker.whileFalse(harvestor.StopIntake());

    //DPAD CREEPING
    m_driverController.povRight().whileTrue(drivebase.driveRobotRelativeCommand(0.0, creepSpeed, 0.0));
    m_driverController.povLeft().whileTrue(drivebase.driveRobotRelativeCommand(0.0, -creepSpeed, 0.0));
    m_driverController.povUp().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, 0, 0.0));
    m_driverController.povDown().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, 0, 0.0));
    
    m_driverController.povDownRight().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, creepSpeed, 0.0));
    m_driverController.povDownLeft().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, -creepSpeed, 0.0));
    m_driverController.povUpRight().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, creepSpeed, 0.0));
    m_driverController.povUpLeft().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, -creepSpeed, 0.0));

    //set the robot's pose (its idea of where it is on the field) to 0,0 for debugging
    t_resetPose.onTrue(drivebase.SetPose(new Pose2d(2,2, new Rotation2d(0))));


    //Shooter
    t_shooterAutoSpeed.whileTrue(shooter.BeginShootingAutoRpm().repeatedly());
    t_shooterHighSpeed.whileTrue(shooter.BeginShootingHighRpm());
    t_shooterLowSpeed.whileTrue(shooter.BeginShootingLowRpm());
    t_shooterManualSpeed.whileTrue(shooter.BeginShootingManualRpm().repeatedly());
    t_increaseShooterManualSpeed.onTrue(shooter.ModifyManualShootingRPM(250));
    t_decreaseShooterManualSpeed.onTrue(shooter.ModifyManualShootingRPM(-250));

    //create a trigger for when any shooting button is pressed, then make sure when that trigger isn't happening, we stop shooting
    Trigger anyShooterButton =
    t_shooterAutoSpeed
        .or(t_shooterHighSpeed)
        .or(t_shooterLowSpeed)
        .or(t_shooterManualSpeed);
    anyShooterButton.whileFalse(shooter.StopShooting());
    

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
