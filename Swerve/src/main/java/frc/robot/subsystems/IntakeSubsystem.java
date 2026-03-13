// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class IntakeSubsystem extends SubsystemBase {
  public SparkMax intakeSpin = new SparkMax(51, MotorType.kBrushed);
  public SparkMax intakeAngle = new SparkMax(52, MotorType.kBrushless);
  private SparkClosedLoopController pidController;
  public RelativeEncoder intakeAngleEncoder;
  private ArmFeedforward feedforward = new ArmFeedforward(
    0.0,   // kS
    0.6,   // kG (gravity)
    0.0,   // kV
    0.0    // kA
  );
  // PID controller for position
  private PIDController pid = new PIDController(5, 0, 0);
  private double targetAngle = 0;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder.positionConversionFactor(2.4);

    intakeAngle.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters
    );

    intakeAngleEncoder = intakeAngle.getEncoder();
    intakeAngleEncoder.setPosition(0);
  }

  public void setIntakeSpeed(double speed) {
    intakeSpin.set(speed);
  }

  public void setIntakeAngle(double angle) {

    double currentAngle = intakeAngleEncoder.getPosition();

    double ff = feedforward.calculate(
        Math.toRadians(currentAngle),
        0
    );

    pidController.setReference(
        angle,
        ControlType.kPosition,
        0,
        ff
    );
  }
  
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    double currentAngle = intakeAngleEncoder.getPosition();

    // PID correction
    double pidOutput = pid.calculate(currentAngle, targetAngle);

    // Feedforward to fight gravity
    double ffOutput = feedforward.calculate(
        Math.toRadians(currentAngle),
        0
    );

    double output = pidOutput + ffOutput;

    intakeAngle.setVoltage(output);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
