package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ModuleConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private SparkMax leftElevatorMotor;
  private SparkMax rightElevatorMotor;
  public RelativeEncoder rightElevatorEncoder;
  private RelativeEncoder leftElevatorEncoder;

  private CommandXboxController gController;

  public ElevatorSubsystem(CommandXboxController controller) {
    leftElevatorMotor =
        new SparkMax(DriveConstants.LEFT_ELEVATOR_MOTOR_PORT, MotorType.kBrushless);
    rightElevatorMotor =
        new SparkMax(DriveConstants.RIGHT_ELEVATOR_MOTOR_PORT, MotorType.kBrushless);

    gController = controller;

    SparkMaxConfig leftElevConfig = new SparkMaxConfig();
    leftElevConfig.idleMode(IdleMode.kBrake);
    leftElevatorMotor.configure(leftElevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    SparkMaxConfig rightElevConfig = new SparkMaxConfig();
    rightElevConfig.idleMode(IdleMode.kBrake);
    rightElevatorMotor.configure(leftElevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightElevatorEncoder = rightElevatorMotor.getEncoder();
    leftElevatorEncoder = leftElevatorMotor.getEncoder();
  }

  @Override
  public void periodic() {}

  public Command ResetEncoders() {
    return this.runOnce(
        () -> {
          leftElevatorEncoder.setPosition(0);
          rightElevatorEncoder.setPosition(0);
        });
  }

  public void elevatorControl() {
    // System.out.println("Left Elevator Rounded Position: " +
    // Math.round(leftElevatorEncoder.getPosition()));
    // Up
    if ((Math.round(rightElevatorEncoder.getPosition()) > -ElevatorConstants.TOP_ELEVATOR_LIMIT)
        && gController.getLeftY() < -ModuleConstants.DEADBAND) {
      leftElevatorMotor.set(ElevatorConstants.LEFT_MOTOR_SPEED);
      rightElevatorMotor.set(-ElevatorConstants.RIGHT_MOTOR_SPEED);
    }
    // Down
    else if ((Math.round(leftElevatorEncoder.getPosition()) > ElevatorConstants.BOT_ELEVATOR_LIMIT)
        && gController.getLeftY() > ModuleConstants.DEADBAND) {
      leftElevatorMotor.set(-ElevatorConstants.LEFT_MOTOR_SPEED);
      rightElevatorMotor.set(ElevatorConstants.RIGHT_MOTOR_SPEED);
    } else {
      leftElevatorMotor.set(0);
      rightElevatorMotor.set(0);
    }
  }
}
