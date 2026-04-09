// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private PhotonCamera cam = new PhotonCamera("Arducam");
    //private PhotonCamera rightCamera = new PhotonCamera("rightCamera");
    //private PhotonCamera leftCamera = new PhotonCamera("leftCamera");
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                                    .loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCam = new Transform3d(
        //new Translation3d(13.75, 1.0625, 2.25),
        new Translation3d(0.349, 0.026, 0.057),
        new Rotation3d(0,  0.821464, 0)
    );

    private PhotonPoseEstimator camPhotonPoseEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        m_robotContainer.drivetrain.seedFieldCentric();
        pdh.setSwitchableChannel(true);

        PortForwarder.add(1182, "10.111.74.11", 1182);
    }

    public static void logMsg(String msg) {
        if (Constants.Testing.ENABLE_LOGS) {
            System.out.println(msg);
        }
    }
   @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        m_robotContainer.updateSmartDashboard();

        var results = cam.getAllUnreadResults();
        SmartDashboard.putBoolean("Camera Connected", !results.isEmpty());

        if (!cam.isConnected()) {
            //logMsg("WARNING, CAMERA IS NOT CONNECTED!!!!!");
        }

        for (var result : results)
        {


            if ((result != null) && (result.hasTargets()))
            {
                var visionEst = camPhotonPoseEstimator.estimateCoprocMultiTagPose(result);
                //logMsg("Vision Estimated Pose - " + visionEst.isEmpty());
                if (visionEst.isEmpty())
                {
                    visionEst = camPhotonPoseEstimator.estimateLowestAmbiguityPose(result);
                }
                //logMsg("Vision Estimated Is Present - " + visionEst.isPresent());
                if (visionEst.isPresent())
                {
                    var est = visionEst.get();
                    if (result.getBestTarget().getPoseAmbiguity() > 0.2) {
                        logMsg("Camera is blind");
                        return; // two possible poses too similar, can't trust it
                    }

                    //logMsg("Estimate present " + est);
                    //logMsg("Yawww " + result.getBestTarget().getYaw());
                    m_robotContainer.drivetrain.addVisionMeasurement(
                        est.estimatedPose.toPose2d(),
                        est.timestampSeconds
                    );
                }
            }
        }
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
        Rotation2d startingAngle = DriverStation.getAlliance()
            .filter(a -> a == DriverStation.Alliance.Red)
            .map(a -> Rotation2d.kZero)
            .orElse(Rotation2d.k180deg);

        m_robotContainer.drivetrain.seedFieldCentric(startingAngle);

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

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        Rotation2d startingAngle = DriverStation.getAlliance()
            .filter(a -> a == DriverStation.Alliance.Red)
            .map(a -> Rotation2d.kZero)
            .orElse(Rotation2d.k180deg);

        m_robotContainer.drivetrain.seedFieldCentric(startingAngle);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        /*m_robotContainer.PathOnTheFly().schedule();*/
    }

    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
