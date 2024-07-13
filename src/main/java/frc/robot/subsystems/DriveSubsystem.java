// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {

  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(DriveConstants.SPEED_RATE_LIMIT);
  SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(DriveConstants.SPEED_RATE_LIMIT);

  // Robot swerve modules
  public final SwerveModule frontLeft =
      new SwerveModule(
          DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
          DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
          DriveConstants.LEFT_FRONT_ENCODER_OFFSET,
          true);

  public final SwerveModule rearLeft =
      new SwerveModule(
          DriveConstants.REAR_LEFT_DRIVE_MOTOR_PORT,
          DriveConstants.REAR_LEFT_TURNING_MOTOR_PORT,
          DriveConstants.LEFT_REAR_ENCODER_OFFSET,
          true);

  public final SwerveModule frontRight =
      new SwerveModule(
          DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
          DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
          DriveConstants.RIGHT_FRONT_ENCODER_OFFET,
          true);

  public final SwerveModule rearRight =
      new SwerveModule(
          DriveConstants.REAR_RIGHT_DRIVE_MOTOR_PORT,
          DriveConstants.REAR_RIGHT_TURNING_MOTOR_PORT,
          DriveConstants.RIGHT_REAR_ENCODER_OFFET,
          true);

  // The gyro sensor
  public final PigeonIMU gyro = new PigeonIMU(30);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          DriveConstants.DRIVE_KINEMATICS,
          new Rotation2d(gyro.getYaw()),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
          });

  // rolling second omega rotation speed, radians
  private double lastYaw = 0;
  private double[] rollingDeltaYaw = new double[50];
  private int rollingDeltaYawIndex = 0;
  private double omega = 0; // Angluar Velocity

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    gyro.setYaw(0);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetPose,
        this::getChassisSpeed,
        this::setChassisSpeed,
        new HolonomicPathFollowerConfig(
            AutoConstants.MAX_SPEED_METERS_PER_SECOND,
            0.5
                * Math.sqrt(
                    Math.pow(DriveConstants.WHEEL_BASE, 2)
                        + Math.pow(
                            DriveConstants.TRACK_WIDTH, 2)), /* TODO: test / increase accuracy */
            new ReplanningConfig(true, true)),
        this::switchAlliance,
        this);
  }

  public Command getZeroHeadingCommand() {
    return this.runOnce(
        () -> {
          this.zeroHeading();
        });
  }

  @Override
  public void periodic() {
    // Update omega rotation
    double yaw = gyro.getYaw();
    rollingDeltaYaw[rollingDeltaYawIndex] = yaw - lastYaw;
    lastYaw = yaw;
    rollingDeltaYawIndex++;
    if (rollingDeltaYawIndex == 50) {
      rollingDeltaYawIndex = 0;
    }
    omega = 0;
    for (double i : rollingDeltaYaw) {
      omega += i;
    }
    omega /= 50;

    // Update the odometry in the periodic block
    odometry.update(
        new Rotation2d(gyro.getYaw()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        });
  }

  public boolean switchAlliance() {
    return AutoConstants.SWITCH_ALLIANCE;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current velocity of the robot as a ChassisSpeed object.
   *
   * @return ChassisSpeed
   */
  public ChassisSpeeds getChassisSpeed() {
    double yaw = gyro.getYaw();
    double speedMPS =
        (frontLeft.getState().speedMetersPerSecond
                + frontRight.getState().speedMetersPerSecond
                + rearLeft.getState().speedMetersPerSecond
                + rearRight.getState().speedMetersPerSecond)
            / 4;

    return new ChassisSpeeds(speedMPS * Math.cos(yaw), speedMPS * Math.sin(yaw), omega);
  }

  /**
   * Sets the desired chassisspeeds of the robot, primarily for Autonomous mode.
   *
   * @param chassisSpeeds Desired state of the robot.
   */
  public void setChassisSpeed(ChassisSpeeds chassisSpeeds) {
    drive(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond,
        true);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        new Rotation2d(gyro.getYaw()),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    xSpeed = xSpeedLimiter.calculate(xSpeed);
    ySpeed = ySpeedLimiter.calculate(ySpeed);

    xSpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    ySpeed *= DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    rot /= DriveConstants.ROTATION_DIVISOR;

    SwerveModuleState[] swerveModuleStates =
        DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, new Rotation2d(getGyroValue()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ModuleConstants.MAX_MODULE_METERS_PER_SECOND);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command AutoDrive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double runTime) {
    return this.runOnce(
        () -> {
          drive(xSpeed, ySpeed, rot, fieldRelative);
          try {
            Thread.sleep(Double.valueOf(runTime * 1000).longValue());
          } catch (InterruptedException e) {
            e.printStackTrace();
          }

          drive(0, 0, 0, true);
        });
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ModuleConstants.MAX_MODULE_METERS_PER_SECOND);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return gyro.getYaw();
  }

  private double getGyroValue() {
    return gyro.getYaw() * Math.PI / 180;
  }

  public double getFrontLeft() {
    return frontLeft.getValue();
  }

  public double getRearLeft() {
    return rearLeft.getValue();
  }

  public double getFrontRight() {
    return frontRight.getValue();
  }

  public double getRearRight() {
    return rearRight.getValue();
  }

  public double[] getDesiredState() {
    return new double[] {
      frontLeft.getDesiredState(),
      frontRight.getDesiredState(),
      rearLeft.getDesiredState(),
      rearRight.getDesiredState()
    };
  }

  public void stop() {
    frontLeft.stop();
    frontRight.stop();
    rearLeft.stop();
    rearRight.stop();
  }
}
