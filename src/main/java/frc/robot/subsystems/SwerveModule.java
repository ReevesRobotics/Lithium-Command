// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final PIDController m_drivingPIDController;
  private final PIDController m_turningPIDController;
  private final PIDController m_feedForwardPIDController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor, encoder, and PID
   * controller. This configuration is specific to the REV MAXSwerve Module built with NEOs, SPARKS
   * MAX, and a Through Bore Encoder.
   */
  public SwerveModule(
    int drivingCANId, int turningCANId, double chassisAngularOffset, boolean isReversed) {
    m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    SparkMaxConfig turningConfig = new SparkMaxConfig();
    m_drivingSparkMax.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    m_turningSparkMax.configure(turningConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    if (isReversed) {
      // m_turningEncoder.setInverted(true);
      // turningConfig.encoder.inverted(true);
      driveConfig.inverted(true);
    }

    // m_turningSparkMax = new CANSparkMax(0, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // the configure above handles this
    // m_drivingSparkMax.restoreFactoryDefaults();
    // m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder();
    m_drivingPIDController = new PIDController(0, 0, 0);
    m_feedForwardPIDController = new PIDController(1, 0, 0);
    m_turningPIDController = new PIDController(0, 0, 0);
  

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    // m_drivingEncoder.setPositionConversionFactor(ModuleConstants.DrivingEncoderPositionFactor);
    // m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.DrivingEncoderVelocityFactor);

    driveConfig.encoder.positionConversionFactor(ModuleConstants.DrivingEncoderPositionFactor);
    driveConfig.encoder.velocityConversionFactor(ModuleConstants.DrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    // m_turningEncoder.setPositionConversionFactor(ModuleConstants.TurningEncoderPositionFactor);
    // m_turningEncoder.setVelocityConversionFactor(ModuleConstants.TurningEncoderVelocityFactor);
    turningConfig.encoder.positionConversionFactor(ModuleConstants.TurningEncoderPositionFactor);
    turningConfig.encoder.velocityConversionFactor(ModuleConstants.TurningEncoderVelocityFactor);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    // m_turningPIDController.setPositionPIDWrappingEnabled(true);
    // m_turningPIDController.setPositionPIDWrappingMinInput(
    //     ModuleConstants.TurningEncoderPositionPIDMinInput); // Min 0
    // m_turningPIDController.setPositionPIDWrappingMaxInput(
    //     ModuleConstants.TurningEncoderPositionPIDMaxInput); // Max 2 * PI

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.P_MODULE_DRIVE_CONTROLLER);
    m_drivingPIDController.setI(0);
    m_drivingPIDController.setD(0);
    // m_drivingPIDController.setFF(1 / ModuleConstants.kDriveWheelFreeSpeedRps); // FF Value -
    // 0.20875544852568829440720072304831
    m_drivingPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.P_MODULE_TURNING_CONTROLLER);
    m_turningPIDController.setI(0);
    m_turningPIDController.setD(0);
    // m_turningPIDController.setFF(1 / ModuleConstants.kDriveWheelFreeSpeedRps);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    driveConfig.idleMode(IdleMode.kBrake);
    turningConfig.idleMode(IdleMode.kBrake);
    // driveConfig.smartCurrentLimit(60);
    // turningConfig.smartCurrentLimit(20);

    // driveConfig.signals.primaryEncoderPositionPeriodMs(55);
    // driveConfig.signals.primaryEncoderPositionPeriodMs(15);
    // driveConfig.signals.primaryEncoderPositionPeriodMs(15);
    // driveConfig.signals.primaryEncoderPositionPeriodMs(65535);
    // driveConfig.signals.primaryEncoderPositionPeriodMs(65535);
    // driveConfig.signals.primaryEncoderPositionPeriodMs(65535);
    // driveConfig.signals.primaryEncoderPositionPeriodMs(65565);

    // turningConfig.signals.primaryEncoderPositionPeriodMs(55);
    // turningConfig.signals.primaryEncoderPositionPeriodMs(65535);
    // turningConfig.signals.primaryEncoderPositionPeriodMs(65535);
    // turningConfig.signals.primaryEncoderPositionPeriodMs(65535);
    // turningConfig.signals.primaryEncoderPositionPeriodMs(15);
    // turningConfig.signals.primaryEncoderPositionPeriodMs(65535);
    // turningConfig.signals.primaryEncoderPositionPeriodMs(65565);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  /**
   * Returns the current state (angle and speed) of the module.
   *
   * @return The current state (angle and speed) of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position (angle and distance) of the module.
   *
   * @return The current position (angle and distance) of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // get current state, already corrected by offset
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
		correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
		correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    System.out.println(m_turningEncoder.getPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    final double driveOutput = m_drivingPIDController.calculate(
        m_drivingEncoder.getVelocity(), correctedDesiredState.speedMetersPerSecond);

    // Add the offset back as we set the PIDController, because the PIDController
    // cares about the encoder's reference frame
    final double turnOutput = m_turningPIDController.calculate(
      m_turningEncoder.getPosition(),
      correctedDesiredState.angle.getRadians());

    final double driveFeedforward = m_feedForwardPIDController.calculate(correctedDesiredState.speedMetersPerSecond);
    m_feedForwardPIDController.reset();

    m_drivingSparkMax.set(driveOutput + driveFeedforward / 3);
    m_turningSparkMax.set(turnOutput / 3);

    m_desiredState = correctedDesiredState; // still chassis reference frame
  }

  public double getDesiredState() {
    return m_desiredState.angle.getRadians();
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getValue() {
    return m_turningEncoder.getPosition();
  }

  public void stop() {
    m_drivingSparkMax.set(0);
    m_turningSparkMax.set(0);
  }
}
