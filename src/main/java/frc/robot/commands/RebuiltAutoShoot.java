package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.HarvestorSubsystem;
import frc.robot.subsystems.RebuiltShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;

public class RebuiltAutoShoot extends SequentialCommandGroup {
    
    public RebuiltAutoShoot(
        RebuiltShooterSubsystem shooter,
        HarvestorSubsystem harvester,
        SwerveSubsystem drivebase
    ){
        addCommands(
               shooter.GetShooterUpToAutoRpmSpeed().withTimeout(1.5),
               
               Commands.parallel(
                shooter.ShootAtAutoRpm(),
                shooter.FeedShooter(),
                harvester.AssistConveyor(),
                drivebase.driveFacingTarget(()->0.0, ()->0.0)
               )
        );
    }
}
