// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

public class IntakeSubsystem extends SubsystemBase {

  //public SparkMax intakeSpin = new SparkMax(51, MotorType.kBrushed);
  //public SparkMax intakeAngle = new SparkMax(52, MotorType.kBrushless);
  private TalonFX intakeSpin = new TalonFX(51);
  private TalonFX intakeAngle = new TalonFX(52);
  private VelocityVoltage velocityRequest = new VelocityVoltage(0);

  //private SparkClosedLoopController pidController;
  //public RelativeEncoder intakeAngleEncoder;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    CurrentLimitsConfigs configs = new CurrentLimitsConfigs();
    configs.StatorCurrentLimitEnable = true;
    configs.StatorCurrentLimit = 30;
    intakeSpin.getConfigurator().apply(configs);
    /*
    // Reset Encoder
    SparkMaxConfig config = new SparkMaxConfig();

    config.encoder.positionConversionFactor(2.4);

    // Changed 0.5 to 1
    config.closedLoop.pid(5, 0, 4);

    config.idleMode(IdleMode.kBrake);

    intakeAngle.configure(
        config,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters
    );

    intakeAngleEncoder = intakeAngle.getEncoder();
    pidController = intakeAngle.getClosedLoopController();

    intakeAngleEncoder.setPosition(0);
     */

    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = 1;
    slot0.kI = 0.0;
    slot0.kD = 0.0;

    slot0.kV = 0.5; // feedforward ----------------------------------------------------------------------------------------------------------------------------------------------------------1q
    slot0.kS = 0.2;
    // intakeAngle.getConfigurator().apply(slot0);
    intakeSpin.getConfigurator().apply(slot0);

  }

  public void setIntakeSpeed(double speed) {
    intakeSpin.set(-speed);
    /* 
    if (speed > 0) {
      intakeAngle.set(0.5);
    } else {
      intakeAngle.set(0);
    }
    */
  }

  public boolean isIntakeActive() {
    return Math.abs(intakeSpin.get()) < 0.05;
  }

  public void setIntakeAngleSpeed(double speed) {
    //intakeAngle.set(speed);
  }

  public void setIntakeAngle(double angle) {
    //intakeSpin.set(0);
    //pidController.setSetpoint(angle, ControlType.kPosition);
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
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
