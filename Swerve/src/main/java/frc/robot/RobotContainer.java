// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.cameraserver.CameraServer;
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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

import frc.robot.Constants;


public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(Constants.Drive.MAX_ANGULAR_RATE_RPS).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private double LauncherSpeed = Constants.Launcher.DEFAULT_SPEED;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Drive.DEADBAND_PERCENT).withRotationalDeadband(MaxAngularRate * Constants.Drive.DEADBAND_PERCENT) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors

            //Fix PID for drivetrain on right bumper press
    private final SwerveRequest.FieldCentricFacingAngle autoAimDrive = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * Constants.Drive.DEADBAND_PERCENT).withRotationalDeadband(MaxAngularRate * Constants.Drive.DEADBAND_PERCENT) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity).withHeadingPID(3, 0.0, 0.1);
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
    private int[] roundEndTimes = Constants.Match.ROUND_END_TIMES;
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

    private final NetworkTableEntry photonEntry;
    public Boolean photonActive = true;

    //private final NetworkTableEntry launcherSpeedEnty;
    public RobotContainer() {
        InitializeSmartDashboard();
        configureBindings();
        setupPathplannerCommands();

        timer.reset();
        timer.start();

        photonEntry = SmartDashboard.getEntry("Photon Working");
        photonEntry.setDefaultBoolean(true);

        //launcherSpeedEnty = SmartDashboard.getEntry("Launcher Speed");
        //launcherSpeedEnty.setDefaultValue(0.6);
    }

    private void setupPathplannerCommands() {
        NamedCommands.registerCommand("ShootFuel", ShootFuel(5, 0.68));
        NamedCommands.registerCommand("ShootFuelFar", ShootFuel(5, 0.8));
        NamedCommands.registerCommand("ShootFuelLong", ShootFuel(8, 0.87));
        NamedCommands.registerCommand("ShootFuelKeepingOn", ShootFuelKeepingOn(3, launcher.distToPower(drivetrain.getHubDist(hubPosition))));
        NamedCommands.registerCommand("TurnOnIntake",  Commands.sequence(                  
            Commands.runOnce(() -> intake.setIntakeSpeed(Constants.Intake.INTAKE_SPEED))
            //Commands.runOnce(() -> indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED))
        ));

        NamedCommands.registerCommand("TurnOffIntake",  Commands.sequence(                  
            Commands.runOnce(() -> intake.setIntakeSpeed(0)),
            Commands.runOnce(() -> indexer.setIndexerSpeed(0))
        ));

        new EventTrigger("DropIntake").onTrue(Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeAngleSpeed(Constants.Intake.INTAKE_ANGLE_SPEED), intake).withTimeout(1)
        ));
        
        new EventTrigger("TurnOnLauncherShooter").onTrue(Commands.sequence(
            Commands.runOnce(() -> 
            {
                launcher.setSpeedLauncherMain(0.7);
                //indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED);
            })
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
        return Commands.sequence(
            Commands.runOnce(() -> indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED)),
            Commands.runOnce(() -> launcher.setSpeedLauncherMain(launcherSpeed)),
            Commands.waitUntil(() -> launcher.isAtTargetSpeed(launcherSpeed)).withTimeout(Constants.Launcher.SPINUP_TIMEOUT_SECONDS),
            Commands.runOnce(() -> launcher.setSpeedLauncherFeed(launcherSpeed)),
            Commands.waitSeconds(seconds),
            Commands.runOnce(() -> launcher.setSpeedLauncher(0)),
            Commands.runOnce(() -> indexer.setIndexerSpeed(0))
        );
    }
    
    private Command ShootFuelKeepingOn(double seconds, double launcherSpeed) {
        return Commands.sequence(
            Commands.runOnce(() -> indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED)),
            Commands.runOnce(() -> launcher.setSpeedLauncherMain(launcherSpeed)),
            Commands.waitUntil(() -> launcher.isAtTargetSpeed(launcherSpeed)).withTimeout(Constants.Launcher.SPINUP_TIMEOUT_SECONDS),
            Commands.runOnce(() -> launcher.setSpeedLauncherFeed(launcherSpeed)),
            Commands.waitSeconds(seconds)
        );
    }

    private void CycleLauncherSpeeds() {
        LauncherSpeed += 0.1;

        if (LauncherSpeed > 1)
            LauncherSpeed = 0.6;
    }

    private void InitializeSmartDashboard() {
        SmartDashboard.putData("Field", field);

        // Auton Options Selection
        m_chooser.setDefaultOption("MiddleAutoR", "MiddleAutoR");
        m_chooser.addOption("RightAuto", "RightAuto");
        m_chooser.addOption("LeftAuto", "LeftAuto");
        m_chooser.addOption("MiddleAutoL", "MiddleAutoL");
        m_chooser.addOption("ExampleAuto", "ExampleAuto");

        SmartDashboard.putData("Auto Selector", m_chooser);

        // Color Chooser
        m_chooserColor.setDefaultOption("Red", "Red");
        m_chooserColor.addOption("Blue", "Blue");

        SmartDashboard.putData("Color Selector", m_chooserColor);
    }

    public void updateSmartDashboard() {
        photonActive = photonEntry.getBoolean(true);

        //if (photonActive)
            //LauncherSpeed = launcherSpeedEnty.getDouble(0.6);
        //else
        SmartDashboard.putNumber("Launcher Speed", LauncherSpeed);


        // Boolean values for all subsystems
        SmartDashboard.putBoolean("Shooter Active", launcher.isLauncherActive());
        SmartDashboard.putBoolean("Shooter Feeder Active", launcher.isLauncherFeederActive());
        SmartDashboard.putBoolean("Indexer Active", indexer.isIndexerActive());
        SmartDashboard.putBoolean("Intake Active", intake.isIntakeActive());

        // Match time and round time
        double matchTimeLeft =  DriverStation.getMatchTime() > 0 ? 
            DriverStation.getMatchTime() :  Constants.Match.MATCH_DURATION_SECONDS - timer.get();

        if (matchTimeLeft < 0)
            matchTimeLeft = 0;

        double roundTimeLeft = getRoundTimeLeft(matchTimeLeft);

        SmartDashboard.putNumber("Match Time", matchTimeLeft);
        SmartDashboard.putNumber("Round Time", roundTimeLeft);

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

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        /*

        Master Controls:
         * a -> NULL
         * b -> Brake
         * x -> NULL
         * y -> NULL
         * left bumper -> 
         * left trigger -> Intake spin and PID
         * left stick -> Intake up?
         * right bumper -> Lock onto target
         * right trigger -> Launcher based on dist and indexer towards launcher
         * right stick -> Intake down
         
         */

        // MOVEMENT
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


        if (!Constants.Testing.TESTING)
        {
            /*

            Controls:
             * a -> NULL
             * b -> Brake
             * x -> NULL
             * y -> cycle Swerve speeds?
             * left bumper -> Bring intake down
             * right bumper -> NULL
             * left trigger -> Runs intake at set speed and runs indexers and locks(PID)
             * right trigger -> Runs launcher at set speed and runs indexers

             */
            // Intake Pivot Down
            intakePivotDown();
            // Brake
            brake();
            // Intake Spin (toggle)
            runIntakeAndIndexer();
            //AutoAim
            autoAimRobot();
            resetFieldPos();
            // Shoot (hold)    
            runShooterAndIndexer();
            // Indexer unjam
            unjamIndexer();
            cycleLauncherSpeed();
            trashCompacting();

            // Run SysId routines when holding back/start and X/Y.
            // Note that each routine should be run exactly once in a single log.
            joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
            joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
            joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
            joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

            drivetrain.registerTelemetry(logger::telemeterize);
        }
        else if (Constants.Testing.TESTING)
        {
            /*
            Controls:
             * a -> NULL
             * b -> Brake
             * x(hold) -> Keep intake steady at 5 degs?
             * y(hold) -> Run indexer towards intake to unjam balls
             * left bumper(hold) -> Bring intake down
             * right bumper(hold) -> Lock onto target
             * left trigger(toggle) -> Start/stop intake and clamp at 0 degs
             * right trigger -> Runs launcher based on dist from hub and locks wheels and sets indexers
             */

            autoAimRobot();
            runShooterAndIndexer();
            intakePivotDown();
            intakePivotUp();
            runIntakeAndIndexer();
            unjamIndexer();
            trashCompacting();
            //#region commented out functions
            /*
            joystick.rightBumper().whileTrue(new RunCommand(() -> {
                drivetrain.calculateAutoAimRot(hubPosition, 0.7);
                Translation2d robotToTarget = hubPosition.minus(getState().Pose.getTranslation());
                double dist = drivetrain.getHubDist();

                // Unit vectors
                Translation2d radialUnit = robotToTarget.div(dist);                          // toward hub
                Translation2d tangentialUnit = new Translation2d(-radialUnit.getY(), radialUnit.getX()); // 90° CCW

                // Joystick inputs
                double radialInput =  joystick.getLeftY(); // push up = move toward hub
                double thetaInput  = -joystick.getLeftX(); // left/right = orbit

                // Combine into field-relative velocity
                Translation2d velocity = radialUnit.times(-radialInput * MaxSpeed)
                                    .plus(tangentialUnit.times(thetaInput * MaxSpeed));

                drivetrain.setControl(
                    autoAimDrive
                        .withVelocityX(velocity.getX())
                        .withVelocityY(velocity.getY())
                        .withTargetDirection(drivetrain.getTargetAngle())
                );
            }, drivetrain));
             */

            //Run Launcher
            // joystick.rightTrigger().whileTrue(
            //     Commands.sequence(
            //         Commands.runOnce(() -> drivetrain.applyRequest(() -> brake)),
            //         Commands.runOnce(() -> 
            //         {
            //             if (drivetrain.getPose().getX() > 4.61 && drivetrain.getPose().getX() < 11.9)
            //             {
            //                 LauncherSpeed = launcher.distToPower(drivetrain.getHubDist(hubPosition) + 1);
            //                 Robot.logMsg("Ferryyy");
            //             }
            //             else
            //             {
            //                 LauncherSpeed = launcher.distToPower(drivetrain.getHubDist(hubPosition));
            //                 Robot.logMsg("KOBEEE");
            //             }
            //         }, launcher),
                
            //         Commands.runOnce(() -> launcher.setSpeedLauncherMain(LauncherSpeed)),
            //         Commands.waitUntil(() -> launcher.isAtTargetSpeed(LauncherSpeed)).withTimeout(Constants.Launcher.SPINUP_TIMEOUT_SECONDS),
            //         Commands.runOnce(() -> launcher.setSpeedLauncherFeed(LauncherSpeed)),
            //         Commands.runOnce(() -> indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED))
            //     )
            // ).onFalse(
            //     Commands.sequence(
            //         Commands.runOnce(() -> launcher.setSpeedLauncher(0)),
            //         Commands.runOnce(() -> indexer.setIndexerSpeed(0)),
            //         Commands.runOnce(() -> Robot.logMsg(" I am scared....Stopping Launcher"))
            //     )
            // );

            // Intake Pivot Down
            // joystick.leftBumper().whileTrue(Commands.run(() -> 
            // {
            //     intake.setIntakeAngleSpeed(0.1);
            //     Robot.logMsg("Intake Angle: " + intake.getIntakeAngle());
            // }, intake))
            // .onFalse(Commands.runOnce(() -> intake.setIntakeAngleSpeed(0)));

            // Pivot up

            //TRASH COMPACTING TEST HEAVILY BEFORE IMPLEMENTING
            /*
            joystick.rightBumper().whileTrue(
                drivetrain.applyRequest(() -> {
                    double vl = launcher.calculateLaunchVelocity(drivetrain, hubPosition);

                    return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(drivetrain.calculateAutoAimRot(hubPosition, vl));
                })
            );
    
            joystick.rightTrigger().and(joystick.rightBumper()).whileTrue(
                Commands.run(() -> launcher.setSpeedLauncher(
                    launcher.launchVelocityToMotorSpeed(launcher.calculateLaunchVelocity(drivetrain, hubPosition))),
                    launcher)
            );
            */
            //#endregion
            drivetrain.registerTelemetry(logger::telemeterize);
        }
    }

    private void trashCompacting() {
        joystick.x().onTrue(Commands.sequence(
            Commands.runOnce(() -> intake.setIntakeAngleSpeed(-0.2)),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> intake.setIntakeAngleSpeed(0))
        ));
    }

    private void intakePivotUp() {
        joystick.a().whileTrue(Commands.runOnce(() -> 
        {
            intake.setIntakeAngleSpeed(-0.1);
        }, intake))
        .onFalse(Commands.runOnce(() -> intake.setIntakeAngleSpeed(0)));
    }

    private void cycleLauncherSpeed() {
        joystick.a().whileTrue(Commands.runOnce(() -> CycleLauncherSpeeds(), launcher));
    }

    private void unjamIndexer() {
        joystick.y().whileTrue(Commands.runOnce(() -> indexer.setIndexerSpeed(-Constants.Indexer.INDEXER_SPEED)))
                .onFalse(Commands.runOnce(() -> indexer.setIndexerSpeed(0)));
    }

    private void runShooterAndIndexer() {
        joystick.rightTrigger().whileTrue(
            Commands.sequence(
                //Commands.runOnce(() -> LauncherSpeed = 0.7),
                Commands.runOnce(() -> Robot.logMsg("Working?")),
                Commands.runOnce(() -> drivetrain.applyRequest(() -> brake)),
                Commands.runOnce(() -> launcher.setSpeedLauncherMain(LauncherSpeed)),
                Commands.waitUntil(() -> launcher.isAtTargetSpeed(LauncherSpeed)).withTimeout(Constants.Launcher.SPINUP_TIMEOUT_SECONDS),
                Commands.runOnce(() -> launcher.setSpeedLauncherFeed(LauncherSpeed)),
                Commands.runOnce(() -> indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED))
                
                // else 
                // {
                //     Commands.sequence(
                //         Commands.runOnce(() -> drivetrain.applyRequest(() -> brake)),
                //         Commands.runOnce(() -> launcher.setSpeedLauncherMain(LauncherSpeed)),
                //         Commands.waitUntil(() -> launcher.isAtTargetSpeed(LauncherSpeed)).withTimeout(Constants.Launcher.SPINUP_TIMEOUT_SECONDS),
                //         Commands.runOnce(() -> launcher.setSpeedLauncherFeed(LauncherSpeed)),
                //         Commands.runOnce(() -> indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED))
                //     );
                // }
            )).onFalse(
            Commands.sequence(
                Commands.runOnce(() -> launcher.setSpeedLauncher(0)),
                Commands.runOnce(() -> indexer.setIndexerSpeed(0))
            )
        );
    }

    private void resetFieldPos() {
        joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    private void autoAimRobot() {
        joystick.rightBumper().whileTrue(new RunCommand(() -> {
            drivetrain.calculateAutoAimRot(hubPosition, 0.7); // updates targetAngle
            drivetrain.setControl(
                autoAimDrive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                            .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                            .withTargetDirection(drivetrain.getTargetAngle())
                
            );
        }, drivetrain));
    }

    private void runIntakeAndIndexer() {
        joystick.leftTrigger().onTrue(
            Commands.runOnce(() -> {
                //intake.setIntakeAngle(0);
                if (intake.isIntakeActive()) {
                    intake.setIntakeSpeed(0);
                    intake.unlockAngle();
                    indexer.setIndexerSpeed(0);
                } else {
                    intake.setIntakeSpeed(Constants.Intake.INTAKE_SPEED);
                    intake.lockAngle(intake.getIntakeAngle());
                    indexer.setIndexerSpeed(Constants.Indexer.INDEXER_SPEED);
                    //intake.lockAngle(0);
                }
            })
        );
    }

    private void brake() {
        joystick.b().whileTrue(drivetrain.applyRequest(() -> brake));
    }

    private void intakePivotDown() {
        joystick.leftBumper().whileTrue(
            Commands.runOnce(() -> intake.setIntakeAngleSpeed(Constants.Intake.INTAKE_ANGLE_SPEED), intake)
        ).onFalse(Commands.runOnce(() -> intake.setIntakeAngleSpeed(0)));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        return new PathPlannerAuto(m_chooser.getSelected());
    } 
}
