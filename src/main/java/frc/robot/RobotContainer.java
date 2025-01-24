// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
//import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoMode;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.AutoSubsystem;
import frc.robot.subsystems.ColourSensorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer extends SubsystemBase {

  // The driver's controller
  public static CommandXboxController driverController =
      new CommandXboxController(OIConstants.DRIVER_CONTROLLER_PORT);
  public static CommandXboxController operatorController =
      new CommandXboxController(OIConstants.OPERATOR_CONTROLLER_PORT);
  public static SparkMax feederMotor =
      new SparkMax(DriveConstants.FEEDER_MOTOR_PORT, MotorType.kBrushless);

  // The robot's subsystems
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(feederMotor);
  public final DriveSubsystem robotDrive = new DriveSubsystem();
  // public final CameraSubsystem cameraSubsystem = new CameraSubsystem();
  public final ColourSensorSubsystem colourSensorSubsystem = new ColourSensorSubsystem();
  //  public final LimitSwitchSubsystem limitSwitchSubsystem = new LimitSwitchSubsystem();
  public final IntakeSubsystem robotIntake = new IntakeSubsystem(feederMotor);
  public final ElevatorSubsystem robotElevator = new ElevatorSubsystem(operatorController);
  public final AutoSubsystem robotAuto =
      new AutoSubsystem(shooterSubsystem, robotIntake, robotDrive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    SparkMaxConfig feederMotorConfig = new SparkMaxConfig();
    feederMotor.configure(feederMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotorConfig.smartCurrentLimit(80);
    feederMotorConfig.idleMode(IdleMode.kBrake);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> {
              double xInput = driverController.getLeftX();
              double yInput = driverController.getLeftY();
              double thetaInput = driverController.getRawAxis(4);

              // Apply a round deadband, based on the x/y distance from the origin
              double distanceFromZero =
                  Math.sqrt(Math.pow(xInput, 2) + Math.pow(yInput, 2)); // Pythagoras
              if (distanceFromZero < ModuleConstants.DEADBAND) {
                xInput = 0;
                yInput = 0;
              }

              robotDrive.drive(
                  Math.pow(-yInput, 3) * Math.abs(yInput),
                  Math.pow(-xInput, 3) * Math.abs(xInput),
                  Math.pow(MathUtil.applyDeadband(-thetaInput, ModuleConstants.DEADBAND), 3)
                      * Math.abs(thetaInput),
                  true);
            },
            robotDrive));

    /* Configure default commands */
    // cameraSubsystem.setDefaultCommand(
    //     /* Prints estimated pose to SmartDashboard */
    //     new RunCommand(
    //         () -> {
    //           Pose3d estimatedPose = cameraSubsystem.getLastEstimatedRobotPose(false);
    //           SmartDashboard.putNumberArray("Camera Estimated Position",
    //               new double[] { estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getZ()
    // });
    //           SmartDashboard.putNumberArray("Camera Estimated Rotation",
    //               new double[] { Math.toDegrees(estimatedPose.getRotation().getX()),
    //                   Math.toDegrees(estimatedPose.getRotation().getY()),
    //                   Math.toDegrees(estimatedPose.getRotation().getZ()) });
    //         },
    //         cameraSubsystem));
    // /* Prints Colour Sensor information to SmartDashboard */

    /* Prints current status of limit switches to SmartDashboard */
    // limitSwitchSubsystem.setDefaultCommand(
    //   new RunCommand(
    //     () -> {
    //       for (int i = 0; i < Constants.ModuleConstants.LIMIT_SWITCHES.length; i++) {
    //         SmartDashboard.putBoolean("Limit Switch #" + i, limitSwitchSubsystem.isClosed(i));
    //       }
    //     },
    //     limitSwitchSubsystem
    //   )
    // );

    robotElevator.setDefaultCommand(
        new RunCommand(
            () -> {
              robotElevator.elevatorControl();
            },
            robotElevator));

    //  colourSensorSubsystem.setDefaultCommand(
    //      new RunCommand(
    //          () -> {
    //            SmartDashboard.putNumber("Colour Sensor Proximity",
    // colourSensorSubsystem.getProximity());
    //           SmartDashboard.putString("Colour Sensor Detected Colour",
    //                colourSensorSubsystem.getDetectedColour().toHexString());
    //          },
    //         colourSensorSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * XboxController}), and then calling passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController
        .button(7)
        .and(driverController.button(8))
        .whileTrue(robotDrive.getZeroHeadingCommand());
    /// driverController.leftBumper().whileTrue(robotDrive.SlowSwerve());
    // driverController.leftBumper().whileFalse(robotDrive.NormalSwerve());

    operatorController.rightBumper().whileTrue(robotIntake.IntakeReverse());
    operatorController.leftBumper().whileTrue(robotIntake.IntakeForward());

    operatorController.leftBumper().whileFalse(robotIntake.IntakeStop());
    operatorController.rightBumper().whileFalse(robotIntake.IntakeStop());

    operatorController.axisGreaterThan(3, 0.75).whileTrue(shooterSubsystem.FeederReverse());
    operatorController.axisGreaterThan(2, 0.75).whileTrue(shooterSubsystem.FeederForward());

    operatorController.axisGreaterThan(3, 0.75).whileFalse(shooterSubsystem.FeederStop());
    operatorController.axisGreaterThan(2, 0.75).whileFalse(shooterSubsystem.FeederStop());

    operatorController.x().whileTrue(shooterSubsystem.ShooterReverse());
    operatorController.x().whileFalse(shooterSubsystem.ShooterStop());

    operatorController.y().whileTrue(shooterSubsystem.ShooterForwardSpeaker());
    operatorController.y().whileFalse(shooterSubsystem.ShooterStop());

    operatorController.a().whileTrue(shooterSubsystem.ShooterForwardAmp());
    operatorController.a().whileFalse(shooterSubsystem.ShooterStop());

    operatorController.button(7).whileTrue(robotElevator.ResetEncoders());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   return new PathPlannerAuto("RavenCompetition");
  // }

  public Command auto(AutoMode mode) {
    return this.runOnce(
        () -> {
          robotAuto.startAutos(mode).schedule();
        });
  }
}
