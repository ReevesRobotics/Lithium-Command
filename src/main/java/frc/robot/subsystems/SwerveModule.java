// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.spark.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
  private final SparkMax m_drivingSparkMax;
  private final SparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;

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

    // m_turningSparkMax = new CANSparkMax(0, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.DrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.DrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.TurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.TurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    if (isReversed) {
      m_turningEncoder.setInverted(true);
    }

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(
        ModuleConstants.TurningEncoderPositionPIDMinInput); // Min 0
    m_turningPIDController.setPositionPIDWrappingMaxInput(
        ModuleConstants.TurningEncoderPositionPIDMaxInput); // Max 2 * PI

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.P_MODULE_DRIVE_CONTROLLER);
    m_drivingPIDController.setI(0);
    m_drivingPIDController.setD(0);
    // m_drivingPIDController.setFF(1 / ModuleConstants.kDriveWheelFreeSpeedRps); // FF Value -
    // 0.20875544852568829440720072304831
    m_drivingPIDController.setOutputRange(-1, 1);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.P_MODULE_TURNING_CONTROLLER);
    m_turningPIDController.setI(0);
    m_turningPIDController.setD(0);
    // m_turningPIDController.setFF(1 / ModuleConstants.kDriveWheelFreeSpeedRps);
    m_turningPIDController.setOutputRange(-1, 1);

    m_drivingSparkMax.setIdleMode(IdleMode.kBrake);
    m_turningSparkMax.setIdleMode(IdleMode.kBrake);
    m_drivingSparkMax.setSmartCurrentLimit(60);
    m_turningSparkMax.setSmartCurrentLimit(20);

    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 55);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    m_drivingSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65565);

    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 55);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 65535);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);
    m_turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65565);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

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
    SwerveModuleState currentState = getState();

    // From this point, everything is done in the chassis reference frame.

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState =
        SwerveModuleState.optimize(desiredState, currentState.angle);

    // Move slower on harder turns
    optimizedDesiredState.speedMetersPerSecond *=
        optimizedDesiredState.angle.minus(currentState.angle).getCos();

    // System.out.println("Desired State - " + optimizedDesiredState);

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(
        optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    // Add the offset back as we set the PIDController, because the PIDController
    // cares about the encoder's reference frame
    m_turningPIDController.setReference(
        optimizedDesiredState.angle.getRadians() + m_chassisAngularOffset,
        SparkMax.ControlType.kPosition);

    m_desiredState = optimizedDesiredState; // still chassis reference frame
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
