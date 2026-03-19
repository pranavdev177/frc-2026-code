// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    public Translation2d hubPosition = redHubPosition;

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

        // Intake Clamp (Hold)
        /*
        joystick.x().whileFalse(
            Commands.run(() -> intake.setIntakeAngleSpeed(1), intake)
        );

        joystick.x().whileFalse(
            Commands.run(() -> intake.setIntakeAngleSpeed(1), intake)
        );
         */

        // Intake Spin (toggle)

        /* 
        joystick.leftTrigger().toggleOnTrue(
            new StartEndCommand(
                () -> intake.setIntakeSpeed(1),
                () -> intake.setIntakeSpeed(0)
            )
        );
        */

        /*
        joystick.leftTrigger().whileFalse(
            Commands.run(() -> intake.setIntakeSpeed(0))
        );
        */

        // Shoot (hold)

        joystick.rightTrigger().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherMain(0.7))
                //Commands.waitSeconds(3),
                //Commands.runOnce(() -> indexer.setIndexerSpeed(1))
            )
        );

        joystick.rightTrigger().onFalse(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherMain(0))
                //Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            )
        );

        joystick.leftTrigger().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(0.7))
                //Commands.waitSeconds(3),
                //Commands.runOnce(() -> indexer.setIndexerSpeed(1))
            )
        );

        joystick.leftTrigger().onFalse(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(0))
                //Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            )
        );

        /* 
        joystick.rightTrigger().and(joystick.rightBumper()).whileTrue(
            Commands.run(() ->  launcher.setSpeedLauncher(
                launcher.launchVelocityToMotorSpeed(launcher.calculateLaunchVelocity(drivetrain, hubPosition))),
                launcher)
        );
        */

        /*
        joystick.y().onTrue(
            Commands.runOnce(() -> launcher.cycleMotorSpeed(), launcher)
        );
        */

        /* 
        joystick.a().onTrue(
            Commands.runOnce(() -> indexer.cycleIndexerState(), indexer)
        );
        */

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
        // Simple drive forward auton
        String selectedAuto = SmartDashboard.getString("Auto Selector", "None");
        final var idle = new SwerveRequest.Idle();
        switch (selectedAuto) {
            case "Blue Left":
                Pose2d bl = new Pose2d(blueLeft, Rotation2d.fromDegrees(0));
                hubPosition = blueHubPosition;
                return Commands.sequence(
                    // Intialize Positions
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    drivetrain.runOnce(() -> drivetrain.resetPose(bl)),

                    // Auto Aim
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(drivetrain.calculateAutoAimRot(
                            hubPosition, launcher.calculateLaunchVelocity(drivetrain, hubPosition)
                            ));
                    }).withTimeout(3),

                    // Shoot pre-loaded balls for 12 seconds
                    launcher.runOnce(() -> launcher.setSpeedLauncherMain(0.75)),

                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(2),

                    launcher.runOnce(() -> launcher.setSpeedLauncherFeed(0.75)),
                    
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(12),

                    // Off and idle for the rest of auton
                    launcher.runOnce(() -> launcher.setSpeedLauncher(0)),
                    drivetrain.applyRequest(() -> idle)
                );
            case "Blue Middle":
                Pose2d bm = new Pose2d(blueMiddle, Rotation2d.fromDegrees(0));
                hubPosition = blueHubPosition;
                return Commands.sequence(
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    drivetrain.runOnce(() -> drivetrain.resetPose(bm)),

                    // Spin up the launcher for 3 seconds
                    launcher.runOnce(() -> launcher.setSpeedLauncherMain(0.72)),

                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(3),

                    // Shoot Balls for 15 seconds
                    launcher.runOnce(() -> launcher.setSpeedLauncherFeed(0.72)),

                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(15),

                    // Off and idle for the rest of auton
                    launcher.runOnce(() -> launcher.setSpeedLauncher(0)),
                    drivetrain.applyRequest(() -> idle)
                );
            case "Blue Right":
                Pose2d br = new Pose2d(blueRight, Rotation2d.fromDegrees(0));
                hubPosition = blueHubPosition;
                return Commands.sequence(
                    // Initialize position
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    drivetrain.runOnce(() -> drivetrain.resetPose(br)),

                    // Go back to human output in 2 seconds
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(1.5)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(2),
                    
                    // Spin up the launcher while the robot auto aims for 3 seconds
                    launcher.runOnce(() -> launcher.setSpeedLauncherMain(0.92)),

                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(drivetrain.calculateAutoAimRot(
                            hubPosition, launcher.calculateLaunchVelocity(drivetrain, hubPosition)
                            ));
                    }).withTimeout(3),

                    launcher.runOnce(() -> launcher.setSpeedLauncherFeed(0.92)),

                    // Humans dump balls in to the hopper while the bot is auto-aimed for 12 seconds
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(12),

                    // Off and idle for the rest of auton
                    launcher.runOnce(() -> launcher.setSpeedLauncher(0)),
                    drivetrain.applyRequest(() -> idle)
                );
            case "Red Left":
                Pose2d rl = new Pose2d(redLeft, Rotation2d.fromDegrees(180));
                hubPosition = redHubPosition;
                return Commands.sequence(
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    drivetrain.runOnce(() -> drivetrain.resetPose(rl)),
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(0.5)
                            .withVelocityY(0)
                            .withRotationalRate(0)
                    )
                    .withTimeout(5.0),
                    drivetrain.applyRequest(() -> idle)
                );
            case "Red Middle":
                Pose2d rm = new Pose2d(redMiddle, Rotation2d.fromDegrees(180));
                hubPosition = redHubPosition;
                return Commands.sequence(
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    drivetrain.runOnce(() -> drivetrain.resetPose(rm)),
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(0.5)
                            .withVelocityY(0)
                            .withRotationalRate(0)
                    )
                    .withTimeout(5.0),
                    drivetrain.applyRequest(() -> idle)
                );
            case "Red Right":
                Pose2d rr = new Pose2d(redRight, Rotation2d.fromDegrees(180));
                hubPosition = redHubPosition;
                return Commands.sequence(
                    drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                    drivetrain.runOnce(() -> drivetrain.resetPose(rr)),

                    // Jerk to release intake
                    //intake.runOnce(() -> intake.setIntakeAngleSpeed(1)),
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(-1.5)
                            .withVelocityY(0)
                            .withRotationalRate(0)
                    )
                    .withTimeout(1.0),
                    drivetrain.applyRequest(() ->
                        drive.withVelocityX(1.5)
                            .withVelocityY(0)
                            .withRotationalRate(0)
                    )
                    .withTimeout(1.0),
                    //intake.runOnce(() -> intake.setIntakeAngleSpeed(0)),
                    
                    // Shoot preloaded balls
                    //indexer.runOnce(() -> indexer.setIndexerSpeed(1)),

                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0d)
                        .withVelocityY(0)
                        .withRotationalRate(drivetrain.calculateAutoAimRot(
                            hubPosition, launcher.calculateLaunchVelocity(drivetrain, hubPosition)
                            ));
                    }).withTimeout(3),

                    launcher.runOnce(() -> launcher.setSpeedLauncher(
                            launcher.launchVelocityToMotorSpeed(
                                launcher.calculateLaunchVelocity(drivetrain, blueHubPosition)
                            )
                        )
                    ),
                    
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(6),

                    launcher.runOnce(() -> launcher.setSpeedLauncher(0)),

                    // Rotate to face forward
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(-1.047);
                    }).withTimeout(1),

                    // Clear the trench
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(1.75)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                    }).withTimeout(2),

                    // Rotate toward balls
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(-1.571);
                    }).withTimeout(1),

                    // Turn on intake, clamp it, and move into balls
                    //intake.runOnce(() -> intake.setIntakeAngleSpeed(0.5)),
                    //intake.runOnce(() -> intake.setIntakeSpeed(1)),
                    drivetrain.applyRequest(() -> {
                        return drive.withVelocityX(0)
                        .withVelocityY(1.6855)
                        .withRotationalRate(0);
                    }).withTimeout(2),

                    // Off and idle for the rest of auton
                    //intake.runOnce(() -> intake.setIntakeAngleSpeed(0)),
                    //intake.runOnce(() -> intake.setIntakeSpeed(0)),
                    drivetrain.applyRequest(() -> idle)
                );
            case "None":
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero));
                return drivetrain.applyRequest(() -> idle);
            default:
                return null;
        }
    }
    
}
