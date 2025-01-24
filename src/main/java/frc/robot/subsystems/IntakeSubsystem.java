package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private SparkMax intakingMotor;
  private SparkMax positioningMotor;
  private SparkMax feederMotor;

  public IntakeSubsystem(SparkMax feederMotor) {
    this.feederMotor = feederMotor;
    intakingMotor = new SparkMax(DriveConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    positioningMotor = new SparkMax(DriveConstants.POSITION_MOTOR_PORT, MotorType.kBrushless);
  }

  @Override
  public void periodic() {}

  public Command IntakeReverse() {
    return this.runOnce(
        () -> {
          intakingMotor.set(IntakeConstants.INTAKING_SPEED);
          positioningMotor.set(IntakeConstants.POSITIONING_SPEED);
          feederMotor.set(-IntakeConstants.FEEDER_FORWARD_SPEED);
        });
  }
  ;

  public Command IntakeForward() {
    return this.runOnce(
        () -> {
          intakingMotor.set(-IntakeConstants.INTAKING_SPEED);
          positioningMotor.set(-IntakeConstants.POSITIONING_SPEED);
          feederMotor.set(IntakeConstants.FEEDER_SPEED);
        });
  }

  public Command IntakeStop() {
    return this.runOnce(
        () -> {
          intakingMotor.set(0);
          positioningMotor.set(0);
          feederMotor.set(0);
        });
  }
}
