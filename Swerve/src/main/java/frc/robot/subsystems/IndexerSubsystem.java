// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

public class IndexerSubsystem extends SubsystemBase {
  private SparkMax indexer = new SparkMax(50, MotorType.kBrushed);
  private int indexerState = 0;

  /** Creates a new ExampleSubsystem. */
  public IndexerSubsystem() {}

  public void setIndexerSpeed(double speed) {
    indexer.set(speed);
  }

  public boolean isIndexerActive() {
    return Math.abs(indexer.get()) > 0.05;
  }

  //#region Unused Code

  public void cycleIndexerState() {
    indexerState += 1;
    indexerState %= 3;
    switch (indexerState) {
    case 0:
        indexer.set(0);
        break;
    case 1:
        indexer.set(1);
        break;
    case 2:
        indexer.set(-1);
        break;
    default:
        indexer.set(0);
        break;
    }
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
  //#endregion
}
