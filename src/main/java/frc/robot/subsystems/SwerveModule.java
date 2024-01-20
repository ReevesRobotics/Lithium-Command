// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

import org.opencv.core.Mat;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;


public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final SparkAbsoluteEncoder absoluteEncoder;
  // private final SparkMaxAlternateEncoder.Type absoluteEncoderType = SparkMaxAlternateEncoder.Type.kQuadrature;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  public double absoluteEnocderOffet;

  private final PIDController drivePIDController =
      new PIDController(ModuleConstants.P_MODULE_DRIVE_CONTROLLER, 0, 0);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final PIDController turningPIDController =
      new PIDController(
          ModuleConstants.P_MODULE_TURNING_CONTROLLER,
          0,
          0);

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      boolean IsRevsred,
      double offset) 
    {

    driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

    absoluteEncoder = turningMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    //absoluteEncoder = turningMotor.getAlternateEncoder(absoluteEncoderType, Constants.ModuleConstants.ENCODER_CPR);

    this.driveEncoder = driveMotor.getEncoder();
    this.turningEncoder = turningMotor.getEncoder();

    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
    turningEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
   // absoluteEncoder.setVelocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);
   // absoluteEncoder.setPositionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setPosition(ModuleConstants.DRIVE_ENCODER_DISTANCE_PER_PULSE);

    // Set whether drive encoder should be reversed or not
    // driveEncoder.setInverted(driveEncoderReversed);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    turningEncoder.setPosition(ModuleConstants.TURNING_ENCODER_DISTANCE_PER_PULSE);

    // Set whether turning encoder should be reversed or not
    // turningEncoder.setInverted(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    if (IsRevsred)
    {
      turningMotor.setInverted(true);
      turningMotor.burnFlash();
    }

    absoluteEnocderOffet = offset;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turningEncoder.getPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    XboxController drivController = new XboxController(0);
    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.angle = desiredState.angle.plus(new Rotation2d(absoluteEnocderOffet));
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getAbsoluteEncoderRad()));
    System.out.println(state.angle);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond * 2);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        turningPIDController.calculate(absoluteEncoder.getVelocity(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller
    driveMotor.set(drivController.getLeftY());
    turningMotor.set(drivController.getLeftX());
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(0);
  }

  public SwerveModulePosition getSwerveModulePosition()
  {
    return new SwerveModulePosition
            (absoluteEncoder.getVelocity() / ModuleConstants.GEAR_RATIO * 2 * Math.PI + ModuleConstants.WHEEL_DIAMETER_METERS / 60,
          new Rotation2d(absoluteEncoder.getPosition()));

    // return new SwerveModulePosition(100, new Rotation2d(absoluteEncoder.getPosition()));
  }

  public double getAbsoluteEncoderRad() {
    return (absoluteEncoder.getPosition() * (2 * Math.PI));
  }

  public double getValue()
  {
    return (absoluteEncoder.getPosition());
  }

  public void stop()
  {
    driveMotor.set(0);
    turningMotor.set(0);
  }

}
