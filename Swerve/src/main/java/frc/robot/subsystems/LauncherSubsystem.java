// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class LauncherSubsystem extends SubsystemBase {
  private TalonFX launcherLeft = new TalonFX(53);
  private TalonFX launcherRight = new TalonFX(54);
  private SparkMax indexer = new SparkMax(50, MotorType.kBrushed);
  private double launcherNonAutoMotorSpeed = 0;

  /** Creates a new ExampleSubsystem. */
  public LauncherSubsystem() {}

  public void setSpeedLauncher(double speed) {
    launcherLeft.set(-speed);
    launcherRight.set(speed);
    if (speed == 0) {
      indexer.set(0);
    } else {
      indexer.set(0.75);
    }
  }

  public double getNonAutoMotorSpeed() {
    return launcherNonAutoMotorSpeed;
  }

  public void cycleMotorSpeed() {
    launcherNonAutoMotorSpeed += 0.2;
    launcherNonAutoMotorSpeed %= 1.0;
  }

  public double launcherVelocityToMotorSpeed(double vl) {
    return -0.706 + 1.33 * vl + -0.245 * vl * vl;
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
