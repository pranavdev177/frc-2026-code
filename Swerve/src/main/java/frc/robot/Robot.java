// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private PhotonCamera rightCamera = new PhotonCamera("rightCamera");
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                                    .loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToRightCam = new Transform3d(
        new Translation3d(0.324769, 0.335885, 0.182352), new Rotation3d(0, 0.821464, -0.819854)
    );
    private PhotonPoseEstimator rightPhotonPoseEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToRightCam);
    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_robotContainer.drivetrain.seedFieldCentric();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();

        var rResults = rightCamera.getAllUnreadResults();
        for (var result : rResults) {
            if ((result != null) && (result.hasTargets())) {
                var rightVisionEst = rightPhotonPoseEstimator.estimateCoprocMultiTagPose(result);
                if (rightVisionEst.isEmpty()) {
                    rightVisionEst = rightPhotonPoseEstimator.estimateLowestAmbiguityPose(result);
                }
                if (rightVisionEst.isPresent()) {
                    var est = rightVisionEst.get();
                    m_robotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds
                    );
                }
            }
        }

        System.out.println(m_robotContainer.drivetrain.getState().Pose.getTranslation().getX());

        CommandScheduler.getInstance().run(); 
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_robotContainer.drivetrain.seedFieldCentric();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        m_robotContainer.drivetrain.seedFieldCentric();
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
