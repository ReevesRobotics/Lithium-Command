package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoMode;

public class AutoSubsystem {
  private ShooterSubsystem shooter;
  private IntakeSubsystem intake;
  private DriveSubsystem drive;

  public AutoSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem drive) {
    this.shooter = shooter;
    this.intake = intake;
    this.drive = drive;
  }

  public SequentialCommandGroup startAutos(AutoMode mode) {
    double DRIVE_SPEED_X = 0.1;
    double DRIVE_SPEED_Y = 0.15;
    double DRIVE_RUNTIME = 3;

    SequentialCommandGroup group = new SequentialCommandGroup();

    switch (mode) {
      case LEFT:
        DRIVE_SPEED_X = 0.2;
        DRIVE_SPEED_Y = 0.2;
        DRIVE_RUNTIME = 1.8;

        group.addCommands(
            shooter.ShooterForwardSpeaker(),
            new WaitCommand(1.5),
            shooter.FeederForward(),
            new WaitCommand(1),
            shooter.ShooterStop(),
            shooter.FeederStop(),
            new WaitCommand(7),
            drive.AutoDrive(-DRIVE_SPEED_X, -DRIVE_SPEED_Y, 0, true, DRIVE_RUNTIME));
        break;
      case CENTER:
        DRIVE_SPEED_Y = 0.15;
        DRIVE_RUNTIME = 3;

        group.addCommands(
            shooter.ShooterForwardSpeaker(),
            new WaitCommand(1.5),
            shooter.FeederForward(),
            new WaitCommand(1),
            shooter.ShooterStop(),
            shooter.FeederStop(),
            new WaitCommand(0.5),
            intake.IntakeForward(),
            drive.AutoDrive(0, -DRIVE_SPEED_Y, 0, true, DRIVE_RUNTIME),
            new WaitCommand(1),
            intake.IntakeStop(),
            drive.AutoDrive(0, DRIVE_SPEED_Y, 0, true, DRIVE_RUNTIME),
            shooter.ShooterForwardSpeaker(),
            new WaitCommand(1.5),
            shooter.FeederForward(),
            new WaitCommand(1),
            shooter.ShooterStop(),
            shooter.FeederStop());
        break;
      case MORECENTER:
        DRIVE_SPEED_Y = 0.15;
        DRIVE_RUNTIME = 3;

        group.addCommands(
            shooter.ShooterForwardSpeaker(),
            new WaitCommand(1.5),
            shooter.FeederForward(),
            new WaitCommand(1),
            shooter.ShooterStop(),
            shooter.FeederStop(),
            new WaitCommand(0.5),
            intake.IntakeForward(),
            drive.AutoDrive(0, -DRIVE_SPEED_Y, 0, true, DRIVE_RUNTIME),
            new WaitCommand(1),
            intake.IntakeStop(),
            drive.AutoDrive(0, DRIVE_SPEED_Y, 0, true, DRIVE_RUNTIME),
            shooter.ShooterForwardSpeaker(),
            new WaitCommand(1.5),
            shooter.FeederForward(),
            new WaitCommand(1),
            shooter.ShooterStop(),
            shooter.FeederStop());
        break;
      case RIGHT:
        DRIVE_SPEED_X = 0.2;
        DRIVE_SPEED_Y = 0.2;
        DRIVE_RUNTIME = 1.8;

        group.addCommands(
            shooter.ShooterForwardSpeaker(),
            new WaitCommand(1.5),
            shooter.FeederForward(),
            new WaitCommand(1),
            shooter.ShooterStop(),
            shooter.FeederStop(),
            new WaitCommand(5),
            drive.AutoDrive(DRIVE_SPEED_X, -DRIVE_SPEED_Y, 0, true, DRIVE_RUNTIME),
            new WaitCommand(2),
            drive.AutoDrive(0, 0, 0.5, true, 0.8));
        break;
      default:
        System.out.println("Red default");
        break;
    }

    return group;
  }
}
