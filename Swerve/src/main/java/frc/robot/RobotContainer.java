// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentricFacingAngle autoAimDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity).withHeadingPID(5, 0.1, 0.7);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final IntakeSubsystem intake = new IntakeSubsystem();

    private final LauncherSubsystem launcher = new LauncherSubsystem();

    private final IndexerSubsystem indexer = new IndexerSubsystem();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /*
     * Robot Initial Starting Points
     */

    // Blue
    public final Translation2d blueRight = new Translation2d(3.519, 0.651);
    public final Translation2d blueMiddle = new Translation2d(3.519, 4.021);
    public final Translation2d blueLeft = new Translation2d(3.519, 7.392);

    // Red
    public final Translation2d redLeft = new Translation2d(12.893, 0.651);
    public final Translation2d redMiddle = new Translation2d(12.893, 4.021);
    public final Translation2d redRight = new Translation2d(12.893, 7.392);

    /*
     * Hub Positions
     */

    public final Translation2d blueHubPosition = new Translation2d(4.612, 4.021);
    public final Translation2d redHubPosition = new Translation2d(11.901, 4.021);

    public final Translation2d hubPosition = new Translation2d(2, 2);



    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.b().whileTrue(drivetrain.applyRequest(() -> brake));
        
        joystick.x().onFalse(
            Commands.runOnce(() -> intake.intakeAngleEncoder.setPosition(0))
        );

        joystick.x().toggleOnTrue(
            new StartEndCommand(
                () -> intake.setIntakeAngleSpeed(1),
                () -> intake.setIntakeAngle(0)
            )
        );

        joystick.leftTrigger().toggleOnTrue(
            new StartEndCommand(
                () -> intake.setIntakeSpeed(1),
                () -> intake.setIntakeSpeed(0),
                intake
            )
        );

        joystick.rightTrigger().and(joystick.rightBumper().negate()).whileTrue(
            Commands.run(() -> launcher.setSpeedLauncher(launcher.getNonAutoMotorSpeed()), launcher)
        );

        joystick.rightTrigger().and(joystick.rightBumper()).whileTrue(
            Commands.run(() ->  launcher.setSpeedLauncher(
                launcher.launchVelocityToMotorSpeed(launcher.calculateLaunchVelocity(drivetrain, hubPosition))),
                launcher)
        );

        joystick.rightTrigger().whileFalse(
            Commands.run(() -> launcher.setSpeedLauncher(0), launcher)
        );

        joystick.y().onTrue(
            Commands.runOnce(() -> launcher.cycleMotorSpeed(), launcher)
        );

        joystick.a().onTrue(
            Commands.runOnce(() -> indexer.cycleIndexerState(), indexer)
        );

        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> {
                double vl = launcher.calculateLaunchVelocity(drivetrain, hubPosition);

                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(drivetrain.calculateAutoAimRot(hubPosition, vl));
            })
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        String selectedAuto = SmartDashboard.getString("Auto Selector", "None");
        final var idle = new SwerveRequest.Idle();

        
        switch (selectedAuto) {
            case "Blue Left":
                // Simple drive forward auton
                return Commands.sequence(
                    // Reset our field centric heading to match the robot
                    // facing away from our alliance station wall (0 deg).
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    // Then slowly drive forward (away from us) for 5 seconds.
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(0.5)
                            .withVelocityY(0)
                            .withRotationalRate(0)
                    )
                    .withTimeout(5.0),
                    // Finally idle for the rest of auton
                    drivetrain.applyRequest(() -> idle)
                );
            case "Blue Middle":
                // Simple drive forward auton
                return Commands.sequence(
                    // Reset our field centric heading to match the robot
                    // facing away from our alliance station wall (0 deg).
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    // Then slowly drive forward (away from us) for 5 seconds.
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(0.5)
                            .withVelocityY(0)
                            .withRotationalRate(0)
                    )
                    .withTimeout(5.0),
                    // Finally idle for the rest of auton
                    drivetrain.applyRequest(() -> idle)
                );
            case "Blue Right":
                return null;
            case "Red Left":
                return null;
            case "Red Middle":
                return null;
            case "Red Right":
                return null;
            case "None":
                return null;
            default:
                return null;
        }
    }
    
}
