package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax shooterLeft;
  private SparkMax shooterRight;
  private SparkMax feederMotor;

  Pose3d robotPositonToApril;

  public ShooterSubsystem(SparkMax feederMotor) {
    this.feederMotor = feederMotor;
    shooterLeft = new SparkMax(DriveConstants.LEFT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    shooterRight = new SparkMax(DriveConstants.RIGHT_SHOOTER_MOTOR_PORT, MotorType.kBrushless);
    // robotPositonToApril = new Pose3d();

    SparkMaxConfig shooterLeftConfig = new SparkMaxConfig();
    SparkMaxConfig shooterRightConfig = new SparkMaxConfig();


    shooterLeftConfig.smartCurrentLimit(80);
    shooterRightConfig.smartCurrentLimit(80);
  }

  // @Override
  // public void periodic() {
  //     checkAprilTag();
  // }

  // private Command checkAprilTag() {
  //     return this.runOnce(() -> robotPositonToApril =
  // cameraSubsystem.getLastEstimatedRobotPose(false));
  // }

  // public Command cameraAutoAlign() {
  //     return this.run(() -> {
  //         if (gcontroller.getRawButton(1)) {
  //             System.out.println("Hi");
  //         }
  //     });

  // }

  public Command FeederForward() {
    return this.runOnce(
        () -> {
          feederMotor.set(ShooterConstants.FEEDER_FORWARD_SPEED);
        });
  }

  public Command FeederReverse() {
    return this.runOnce(
        () -> {
          feederMotor.set(-ShooterConstants.FEEDER_REVERSE_SPEED);
        });
  }

  public Command FeederReverseSlow() {
    return this.runOnce(
        () -> {
          feederMotor.set(-0.15);
        });
  }

  public Command FeederStop() {
    return this.runOnce(
        () -> {
          feederMotor.set(0);
        });
  }

  public Command ShooterReverse() {
    return this.runOnce(
        () -> {
          shooterLeft.set(-ShooterConstants.SHOOTER_REVERSE_SPEED);
          shooterRight.set(ShooterConstants.SHOOTER_REVERSE_SPEED);
        });
  }

  public Command ShooterForwardSpeaker() {
    return this.runOnce(
        () -> {
          System.out.println("Running");
          // Setting a min to make sure the motor speed doesn't go over 1.0 or under -1.0
          // Applying half of the delta value to equally change the motor speeds to balance out the
          // shooting of the note
          shooterLeft.set(
              Math.min(
                  1.0,
                  ShooterConstants.SHOOTER_SPEAKER_SPEED_BASE
                      + ShooterConstants.SHOOTER_SPEAKER_SPEED_DELTA / 2));
          shooterRight.set(
              -(Math.min(
                  1.0,
                  ShooterConstants.SHOOTER_SPEAKER_SPEED_BASE
                      - ShooterConstants.SHOOTER_SPEAKER_SPEED_DELTA / 2)));
        });
  }

  public Command ShooterForwardAmp() {
    return this.runOnce(
        () -> {
          shooterLeft.set(ShooterConstants.SHOOTER_AMP_SPEED);
          shooterRight.set(-ShooterConstants.SHOOTER_AMP_SPEED);
        });
  }

  public Command ShooterStop() {
    return this.runOnce(
        () -> {
          System.out.println("Shooter Stop");
          shooterLeft.set(0);
          shooterRight.set(0);
        });
  }
}
