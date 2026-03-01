package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RebuiltAutoShoot;
//import subsystems
import frc.robot.subsystems.RebuiltShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RebuiltClimberSubsystem;
import frc.robot.subsystems.CandleLedSubsystem;
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
import edu.wpi.first.math.geometry.Translation2d;
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
  //private final RebuiltClimberSubsystem climber = new RebuiltClimberSubsystem();
  //private final CandleLedSubsystem candle = new CandleLedSubsystem();

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
    registerCommandsToBeUsableInPathPlanner();
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
    Trigger t_driver_extend_harvester = m_driverController.rightBumper();
    Trigger t_driver_retract_harvester = m_driverController.leftBumper();

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

    Trigger t_climber_up = m_operatorController.rightTrigger(0.5);
    Trigger t_climber_down = m_operatorController.leftTrigger(0.5);

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
    t_harvester.whileTrue(harvestor.Intake());
    t_increaseHarvesterRPM.onTrue(harvestor.ModifyIntakeTargetRPMs(250));
    t_decreaseHarvesterRPM.onTrue(harvestor.ModifyIntakeTargetRPMs(-250));
    t_extendHarvester.onTrue(harvestor.Extend());
    t_retractHarvester.onTrue(harvestor.Retract());
    t_driver_extend_harvester.onTrue(harvestor.Extend());
    t_driver_retract_harvester.onTrue(harvestor.Retract());


    //Conveyor & kicker
    t_conveyorAndKicker.whileTrue(shooter.FeedShooter());
    //t_conveyorAndKicker.whileTrue(harvestor.Intake());

    //DPAD CREEPING
    m_driverController.povRight().whileTrue(drivebase.driveRobotRelativeCommand(0.0, -creepSpeed, 0.0));
    m_driverController.povLeft().whileTrue(drivebase.driveRobotRelativeCommand(0.0, creepSpeed, 0.0));
    m_driverController.povUp().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, 0, 0.0));
    m_driverController.povDown().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, 0, 0.0));
    
    m_driverController.povDownRight().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, -creepSpeed, 0.0));
    m_driverController.povDownLeft().whileTrue(drivebase.driveRobotRelativeCommand(-creepSpeed, creepSpeed, 0.0));
    m_driverController.povUpRight().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, -creepSpeed, 0.0));
    m_driverController.povUpLeft().whileTrue(drivebase.driveRobotRelativeCommand(creepSpeed, creepSpeed, 0.0));

    //set the robot's pose (its idea of where it is on the field) to 0,0 for debugging
    t_resetPose.onTrue(drivebase.SetPose(new Pose2d(2,2, new Rotation2d(0))));


    //Shooter
    t_shooterAutoSpeed.whileTrue(shooter.ShootAtAutoRpm());
    t_shooterHighSpeed.whileTrue(shooter.ShootAtHighRpm());
    t_shooterLowSpeed.whileTrue(shooter.ShootAtLowRpm());
    t_shooterManualSpeed.whileTrue(shooter.ShootAtManualRpm());
    t_increaseShooterManualSpeed.onTrue(shooter.ModifyManualShootingRPM(100));
    t_decreaseShooterManualSpeed.onTrue(shooter.ModifyManualShootingRPM(-100));

    //Climber
    //t_climber_up.onTrue(climber.Extend());
    //t_climber_down.onTrue(climber.Retract());


    //create a trigger for when any shooting button is pressed, then make sure when that trigger isn't happening, we stop shooting
    Trigger anyShooterButton =
    t_shooterAutoSpeed
        .or(t_shooterHighSpeed)
        .or(t_shooterLowSpeed)
        .or(t_shooterManualSpeed);
    anyShooterButton.whileFalse(shooter.StopShooting());
    

  } 

  private void registerCommandsToBeUsableInPathPlanner(){
    
    NamedCommands.registerCommand("ExtendHarvester", harvestor.Extend());

    NamedCommands.registerCommand("RetractHarvester", harvestor.Retract());

    NamedCommands.registerCommand("StartIntake", harvestor.PathPlannerStartIntake());

    NamedCommands.registerCommand("StopIntake", harvestor.PathPlannerStopIntake());

    NamedCommands.registerCommand("Shoot", new RebuiltAutoShoot(shooter, harvestor, drivebase).withTimeout(5));
    
    

  }

  private void configureAutos(){
    
    //m_autoChooser is the component on our dashboard that lets us choose an auto routine we want to run
    
    //First, we add names for our routines that we can select
    m_autoChooser.setDefaultOption("1Outpost", "1Outpost");
    m_autoChooser.addOption("2Depot", "2Depot");
    m_autoChooser.addOption("Harvestor Test","Harvestor Test");

    //Second, we add that data to the dashboard all at once
    SmartDashboard.putData("Auto Choices", m_autoChooser);

    //This ties in with the getAutonomousCommand() function below

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {    

    //This is where the logic and commands inside our autonomous routines goes.
    //We simply check if the autonomous chooser matches one of these names, then do those actions.

    if("1Outpost".equals(m_autoChooser.getSelected())){
      return 
      AutoBuilder.buildAuto("1Outpost");
    }

    if("1Outpost".equals(m_autoChooser.getSelected())){
      return 
      AutoBuilder.buildAuto("1Outpost");
    }

    
    if("Harvestor Test".equals(m_autoChooser.getSelected())){
      return Commands.sequence(
        harvestor.Extend(),
        new WaitCommand(2),
        harvestor.Retract()
      );
    }

    if("Shooter Test".equals(m_autoChooser.getSelected())){
      return Commands.sequence(
        new RebuiltAutoShoot(shooter, harvestor, drivebase).withTimeout(5)
      );
    }

    //default if there is no match
    return Commands.none();
    

  }
}
