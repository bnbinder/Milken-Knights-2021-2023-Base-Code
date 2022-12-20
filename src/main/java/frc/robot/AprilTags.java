// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MKAPRIL;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

/** Add your docs here. */
public class AprilTags {

    private PIDController turnPID;
    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);

    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(3);

    // Change this to match the name of your camera
    PhotonCamera camera;

    PhotonPipelineResult result;

    private AprilTags()
    {
        PortForwarder.add(5800, "photonvision.local", 5800);
        camera = new PhotonCamera("CharminBearsLoveToShit");
        turnPID = new PIDController(MKAPRIL.kP, MKAPRIL.kI, MKAPRIL.kD);
        camera.setPipelineIndex(0);
            // Set driver mode to off.
        camera.setDriverMode(false);
    }

    public static AprilTags getInstance()
    {
        return InstanceHolder.mInstance;
    }

    public double getRCWApril()
    {
        if (result.hasTargets()) {
            // First calculate range
            return turnPID.calculate(result.getBestTarget().getYaw(), 0); 
        }
        else
        {
            return 0.69;
        }
    }

    public void updateApril()
    {
        result = camera.getLatestResult();
    }

    public double getRange()
    {
        if (result.hasTargets()) {
        return
                    PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT_METERS,
                            TARGET_HEIGHT_METERS,
                            CAMERA_PITCH_RADIANS,
                            Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        else 
        {
            return 0.69;
        }
    }

    public void aprilSmartDashboard()
    {
        SmartDashboard.putNumber("apirl", getRCWApril());
        SmartDashboard.putNumber("aprilRQANge", getRange());
        SmartDashboard.putBoolean("DOYOUFUCKIGSEEEE", result.hasTargets());
    }

    private static class InstanceHolder
    {
        private static final AprilTags mInstance = new AprilTags();
    } 
}
