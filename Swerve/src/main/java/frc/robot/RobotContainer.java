// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
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
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double LauncherSpeed = 0.6;

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

    private Timer timer = new Timer();
    // MUST be from highest to lowest
    private int[] roundEndTimes = {140, 105, 80, 55, 30, 0};
    private SendableChooser<String> m_chooser = new SendableChooser<String>();
    private SendableChooser<String> m_chooserColor = new SendableChooser<String>();
    private final Field2d field = new Field2d();

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
        setupPathplannerCommands();

        SmartDashboard.putData("Field", field);

        timer.reset();
        timer.start();
    }

    private void setupPathplannerCommands() {
        NamedCommands.registerCommand("ShootFuel", ShootFuel(5, 0.6));
        NamedCommands.registerCommand("ShootFuelFar", ShootFuel(5, 1));
        NamedCommands.registerCommand("ShootFuelKeepingOn", ShootFuelKeepingOn(3, 0.6));
        NamedCommands.registerCommand("ShootFuelLong", ShootFuel(8, 0.8));
        NamedCommands.registerCommand("TurnOnIntake",  Commands.sequence(                  
            /*Commands.runOnce(() -> intake.setIntakeSpeed(1)),
            Commands.runOnce(() -> indexer.setIndexerSpeed(-1))*/
        ));

        NamedCommands.registerCommand("TurnOffIntake",  Commands.sequence(                  
            Commands.runOnce(() -> intake.setIntakeSpeed(0)),
            Commands.runOnce(() -> indexer.setIndexerSpeed(0))
        ));

        new EventTrigger("DropIntake").onTrue(Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeAngle(1), intake).withTimeout(2)
        ));
        
        new EventTrigger("TurnOnLauncherShooter").onTrue(Commands.sequence(
            Commands.runOnce(() -> launcher.setSpeedLauncherMain(0.7))
        ));
    }

    /*public Command PathOnTheFly() {
        Pose2d namedPoint = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(0));
        return AutoBuilder.pathfindToPose(
            namedPoint,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI),
            0.0
        );
    }*/

    private Command ShootFuel(double seconds, double launcherSpeed) {
        /*if (!launcher.isLauncherActive()) {
            return Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherMain(0.7)),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> indexer.setIndexerSpeed(1)),
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(0.7)),
                Commands.waitSeconds(seconds),
                Commands.runOnce(() -> launcher.setSpeedLauncher(0)),
                Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            );
        }*/

        return Commands.sequence(
            //Commands.runOnce(() -> indexer.setIndexerSpeed(-1)),
            Commands.runOnce(() -> launcher.setSpeedLauncher(launcherSpeed)),
            Commands.waitSeconds(seconds),
            Commands.runOnce(() -> launcher.setSpeedLauncher(0)),
            Commands.runOnce(() -> indexer.setIndexerSpeed(0))
        );
    }

    
    private Command ShootFuelKeepingOn(double seconds, double launcherSpeed) {
        /*if (!launcher.isLauncherActive()) {
            return Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherMain(0.7)),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> indexer.setIndexerSpeed(1)),
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(0.7)),
                Commands.waitSeconds(seconds),
                Commands.runOnce(() -> launcher.setSpeedLauncher(0)),
                Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            );
        }*/

        return Commands.sequence(
            //Commands.runOnce(() -> indexer.setIndexerSpeed(-1)),
            Commands.runOnce(() -> launcher.setSpeedLauncher(launcherSpeed)),
            Commands.waitSeconds(seconds)
        );
    }

    public void updateSmartDashboard() {
        // Boolean values for all subsystems
        SmartDashboard.putBoolean("Shooter Active", launcher.isLauncherActive());
        SmartDashboard.putBoolean("Shooter Feeder Active", launcher.isLauncherFeederActive());
        SmartDashboard.putBoolean("Indexer Active", indexer.isIndexerActive());
        SmartDashboard.putBoolean("Intake Active", intake.isIntakeActive());

        // Match time and round time
        double matchTimeLeft =  DriverStation.getMatchTime() > 0 ? 
            DriverStation.getMatchTime() :  160 - timer.get();

        if (matchTimeLeft < 0)
            matchTimeLeft = 0;

        double roundTimeLeft = getRoundTimeLeft(matchTimeLeft);

        SmartDashboard.putNumber("Match Time", matchTimeLeft);
        SmartDashboard.putNumber("Round Time", roundTimeLeft);

        // Auton Options Selection
        m_chooser.setDefaultOption("MiddleAutoR", "MiddleAutoR");
        m_chooser.addOption("RightAuto", "RightAuto");
        m_chooser.addOption("LeftAuto", "LeftAuto");
        m_chooser.addOption("MiddleAutoR", "MiddleAutoR");
        m_chooser.addOption("MiddleAutoL", "MiddleAutoL");
        m_chooser.addOption("ExampleAuto", "ExampleAuto");

        SmartDashboard.putData("Auto Selector", m_chooser);

        // Color Chooser
        m_chooserColor.setDefaultOption("Red", "Red");
        m_chooserColor.addOption("Red", "Red");
        m_chooserColor.addOption("Blue", "Blue");

        SmartDashboard.putData("Color Selector", m_chooserColor);

        if (m_chooserColor.getSelected().equals("Red"))
        {
            hubPosition = redHubPosition;
        } else
        {
            hubPosition = blueHubPosition;
        }

        // Other Features
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Max Speed", MaxSpeed);
        SmartDashboard.putNumber("Launcher Speed", LauncherSpeed);
        field.setRobotPose(drivetrain.getState().Pose);
    }

    private double getRoundTimeLeft(double matchTimeLeft) {
        double nextThreshold = 0;
        for (double t: roundEndTimes) {
            if (matchTimeLeft >= t) {
                nextThreshold = t;
                break;
            }
        }

        return matchTimeLeft - nextThreshold;
    }

    private void cycleSwerveSpeed() {
        MaxSpeed += 0.5;

        if (MaxSpeed > 4.5)
            MaxSpeed = 2.5;
    }

    private void cycleLauncherSpeed() {
        LauncherSpeed += 0.2;

        if (LauncherSpeed > 1)
            LauncherSpeed = 0.6;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        // Movement
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


        // Intake Pivot Down
        /*joystick.x().whileTrue(
            Commands.run(() -> intake.setIntakeAngleSpeed(1), intake)
        );*/

        // Intake Pivot Up
        joystick.leftBumper().whileTrue(
            Commands.run(() -> intake.setIntakeAngleSpeed(-1), intake)
        );

        // Brake
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

        // cycle launcher speed 
        joystick.a().onTrue(
            Commands.sequence(
                Commands.runOnce(() -> cycleLauncherSpeed(), launcher)
            )
        );

        // Intake Spin (toggle)
        joystick.leftTrigger().toggleOnTrue(
            new StartEndCommand(
                () -> {
                    intake.setIntakeSpeed(1);
                    indexer.setIndexerSpeed(-1);
                },
                () -> { 
                    intake.setIntakeSpeed(0);
                    indexer.setIndexerSpeed(0);
                }
            )
        );

        //joystick.leftTrigger().whileFalse(
        //    Commands.run(() -> intake.setIntakeSpeed(0))
        //);

        // Shoot (hold)
        
        // Launcher Main
        joystick.rightTrigger().whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherMain(LauncherSpeed)),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(LauncherSpeed)),
                Commands.runOnce(() -> indexer.setIndexerSpeed(-1))
            )
        );
        
        joystick.rightTrigger().onFalse(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncher(0)),
                Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            )
        );

        // Launcher Feeder
        /*joystick.rightBumper().whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(LauncherSpeed)),
                Commands.waitSeconds(2),
                Commands.runOnce(() -> indexer.setIndexerSpeed(1))
            )
        );

        joystick.rightBumper().onFalse(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(0)),
                Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            )
        ); */


        /* joystick.leftTrigger().onFalse(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(0))
                //Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            )
        ); */

        /* 
        joystick.rightTrigger().and(joystick.rightBumper()).whileTrue(
            Commands.run(() ->  launcher.setSpeedLauncher(
                launcher.launchVelocityToMotorSpeed(launcher.calculateLaunchVelocity(drivetrain, hubPosition))),
                launcher)
        );
        */

        
        joystick.y().onTrue(
            Commands.runOnce(() -> cycleSwerveSpeed())
        );

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

        // Reset the field-centric heading on right bumper press.
        //joystick.rightBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        return new PathPlannerAuto(m_chooser.getSelected());
        
        /*String selectedAuto = "Blue Right";
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
        } */
    } 
    
}
