// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class LauncherSubsystem extends SubsystemBase {
  private TalonFX launcherLeft = new TalonFX(53);
  private TalonFX launcherRight = new TalonFX(54);
  private double launcherNonAutoMotorSpeed = 0;
  private double launchVelocity = 0;
  public final double launcherAngle = 1.317724;
  public final double relativeHubHeight = 2.148;

  /** Creates a new ExampleSubsystem. */
  public LauncherSubsystem() {}

  public void setSpeedLauncher(double speed) {
    launcherLeft.set(-speed);
    launcherRight.set(speed);
  }

  public double getNonAutoMotorSpeed() {
    return launcherNonAutoMotorSpeed;
  }

  public void cycleMotorSpeed() {
    launcherNonAutoMotorSpeed += 0.2;
    launcherNonAutoMotorSpeed %= 1.0;
  }

  public double launchVelocityToMotorSpeed(double vl) {
    return -0.706 + 1.33 * vl + -0.245 * vl * vl;
  }

  public double calculateLaunchVelocity(CommandSwerveDrivetrain drivetrain, Translation2d hubPosition) {
    Translation2d robotToTarget = hubPosition.minus(drivetrain.getState().Pose.getTranslation());
    double relativeHubDistance = robotToTarget.getY() * robotToTarget.getY() 
                                    + robotToTarget.getX() * robotToTarget.getX();
    relativeHubDistance = Math.sqrt(relativeHubDistance);
    double g = 9.81;
    double theta = Math.toRadians(75.5);

    if (relativeHubDistance * Math.tan(theta) <= relativeHubHeight) {
      return 0;
    }

    double x = relativeHubDistance;
    // Hub Height
    double y = 2.148;

    double velocity = Math.sqrt(
        (g * x * x) /
        (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y))
    );
    return velocity;
  }

  /* 
  public double calculateLaunchVelocity(CommandSwerveDrivetrain drivetrain, Translation2d hubPosition) {
    double offsetAngle = drivetrain.getOffsetAngle();
    Translation2d robotToTarget = hubPosition.minus(drivetrain.getState().Pose.getTranslation());
    double relativeHubDistance = robotToTarget.getY() * robotToTarget.getY() 
                                + robotToTarget.getX() * robotToTarget.getX();
    relativeHubDistance = Math.sqrt(relativeHubDistance);
    var state = drivetrain.getState();
    ChassisSpeeds speeds = state.Speeds;
    double vx = speeds.vxMetersPerSecond;
    double vy = speeds.vyMetersPerSecond;
    double vxPrime = vx * Math.cos(offsetAngle) + vy * Math.sin(offsetAngle);
    double g = 10;
    double a = Math.sin(launcherAngle) * relativeHubDistance;
    double b = vxPrime;
    double c = Math.cos(launcherAngle);
    double d = (1/2) * relativeHubDistance * relativeHubDistance * g;
    double e = relativeHubHeight;

    launchVelocity = -(2*e*b*c - a*b);
    launchVelocity += Math.sqrt((2*e*b*c) * (2*e*b*c) - 4*(e*c*c - a*c) * (e*b*b - d));
    launchVelocity /= 2*(e*c*c - a*c);

    return launchVelocity;
  }
  */

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
