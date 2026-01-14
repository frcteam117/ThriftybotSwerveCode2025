// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// error:
/*Could not find **any** PhotonVision coprocessors on NetworkTables. Double check that PhotonVision is running, and that your camera is connected!
Error at org.photonvision.PhotonCamera.verifyVersion(PhotonCamera.java:490): Could not find **any** PhotonVision coprocessors on NetworkTables. Double check that PhotonVision is running, and that your camera is connected!
PhotonVision coprocessor at path /photonvision/PC_Camera has not reported a message interface UUID - is your coprocessor's camera started? */
package frc.robot; // TEST WHEN U GET HOME BESTIEEEEEEEEEEEEEEEEEEEEEEEEEE

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.SwerveConstants;
import frc.robot.util.PathUtil;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;
//import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;   
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonUtils;

public class Robot extends TimedRobot {
  // private final XboxController m_controller = new XboxController(0);
  private final PS5Controller m_controller = new PS5Controller(0);
  AHRS gyro = new AHRS(NavXComType.kUSB1);
    
  private final Drivetrain m_swerve = new Drivetrain(() -> Rotation2d.fromDegrees(gyro.getYaw()), new Pose2d());  // private final SimDrivetrain m_simSwerve = new SimDrivetrain(new Pose2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9);
    // 
  PhotonCamera camera0; // needs callibrated
  PhotonCamera camera2;
  Timer timer;
  //Timer timer = new Timer();
  public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static List<Pose3d> AprilTagPoses = Arrays.asList();

  //
  List<Integer> aprilTagIDs = Arrays.asList(1, 2, 3); // do we need this?????? maybe get rid of it <---------------------
  public static int curAprilTagID = 0;

  double targetYaw = 0;
  double targetRange; // from photonvision docs
  double kPVision_Turn;
  
  Pose2d curPose;
  double curX;
  double curY;
  //Rotation2d curRot;

  double pathTimerStop = 0.0;

  int curPathStep = 1;

  public static boolean pathRunning = false;

  int totalPathSteps = 0;

  Command curPathCommand;

  
  public Robot () {

    SmartDashboard.putBoolean("running Path1Command",true);
    timer = new Timer();
    timer.reset();
    timer.stop();
    kPVision_Turn = -.03;
    targetYaw = (0.0);
    camera0 = new PhotonCamera("PC_Camera0");
    camera2 = new PhotonCamera("PC_Camera2");
    Rotation2d originRot = new Rotation2d(0);
    Pose2d origin = new Pose2d(0,0,originRot);
    m_swerve.resetOdometry(origin);
    //
    for (int i = 0; i < 33; i++) { // 33 because 32 tags, index 0 will return a safe Null
        Pose3d tagPose = kTagLayout.getTagPose(i).orElse(new Pose3d()); 
        AprilTagPoses.add(tagPose);
    }
    SmartDashboard.putNumber("AprilTag field pose - X",AprilTagPoses.get(1).getX());

  }
  @Override
  public void robotPeriodic() {
      // This runs in all robot modes (disabled, auto, teleop, test)
      m_swerve.periodic();

  } 

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    curPose = m_swerve.getPose();
    curX = curPose.getX();
    curY = curPose.getY();
    //curRot = curPose.getRotation();

    if (m_controller.getSquareButtonPressed()) {
        m_swerve.resetFieldRelativeDirection();
    }

    if (m_controller.getCrossButton()) {
        m_swerve.setX();
    } else {
    driveWithJoystick(true);
    }
  }


  private void resetPathVars() {
    curPathStep = 1;
    totalPathSteps = 0;
    pathTimerStop = 0;
    //curPathCommand = PathCommands.BlankCommand();
    timer.stop();
    timer.reset();
    pathRunning = false;
  }

  private void setSwerve(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    double a =
        m_xspeedLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.03))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC
            * 0.4;
    double b =
        m_yspeedLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.03))
            * SwerveConstants.TOP_SPEED_METERS_PER_SEC
            * 0.4;
    double c =
        m_rotLimiter.calculate(MathUtil.applyDeadband(rot, 0.04))
            * 1.4;
    m_swerve.drive(a, b, c, fieldRelative, getPeriod());
  }

  private void driveWithJoystick(boolean fieldRelative) {
        //setSwerve(0,0,0, fieldRelative);
        boolean targetVisible = false;
        // Read in relevant data from the Camera
        var results = Arrays.asList(camera0.getAllUnreadResults(),camera2.getAllUnreadResults());

        for (int i = 0; i < results.size(); i++) { // looping through results of each camera, with this system camera2 has priority, see if you need to coordinate
            // - it so all cameras combine results or if this system works - THIS IS THE PROBLEM THIS NEVER RETURNS TARGET AND VISIBLE <---------
            if (!results.get(i).isEmpty()) {// Camera processed a new frame since last
                // Get the last one in the list.
                var result = results.get(i).get(results.get(i).size() - 1);
            // SmartDashboard.putNumber("Target tag ID", (result.getTargets().get(result.getTargets().size)-1));
                SmartDashboard.putBoolean("result.hasTargets()", result.hasTargets());
                if (result.hasTargets()) {
                    // At least one AprilTag was seen by the camera - should be getting thru to here on/off but still yes
                    for (var target : result.getTargets()) {
                        if (aprilTagIDs.contains(target.getFiducialId())) { 
                            // found one of the tags in aprilTagIDs
                            curAprilTagID = target.getFiducialId();
                            targetYaw = target.getYaw();
                            targetVisible = true;
                            SmartDashboard.putNumber("Target tag ID", curAprilTagID);
                            SmartDashboard.putNumber("tag vis on camera #",i);
                            System.out.println(target.getYaw());
                            targetRange =
                                        PhotonUtils.calculateDistanceToTargetMeters( // THESE NEED TO BE TUNED???
                                                0.5   , // Measured with a tape measure, or in CAD.
                                                1.435, // From 2024 game manual for ID 22, CHANGE IF U WANT TS TO WORK
                                                Units.degreesToRadians(-30.0), // Measured with a protractor, or in CAD.
                                                Units.degreesToRadians(target.getPitch()));
                        }
                    }
                }
            }
            else {
                //curAprilTagID = 0;
                SmartDashboard.putNumber("Target tag ID", 0);
                SmartDashboard.putNumber("tag vis on camera #",-1);
            }
        }


        if (m_controller.getTriangleButton()) {
            SmartDashboard.putBoolean("triangle down", true);
            SmartDashboard.putNumber("check #",0);
        }
        else {
            SmartDashboard.putBoolean("triangle down", false);
        }
        SmartDashboard.putBoolean("target visible", targetVisible);
        if (!targetVisible) {
            curAprilTagID = 0;
        }

        // Auto-align when requested
        if (m_controller.getTriangleButton()) {
            SmartDashboard.putNumber("check #",1);
            fieldRelative = true;
            
            if (targetRange > 2 && targetVisible) { // reset the camera photonvision values so the targetrange stuff can be accurate?
                SmartDashboard.putNumber("check #",2);
                SmartDashboard.putBoolean("aligning to tag",true);
                double xSpeed =
                    -m_xspeedLimiter.calculate(MathUtil.applyDeadband(targetRange * 0.5, 0.03)) // CONFIGURE STUFF SO U CAN TEST IF TS WORKS W/ SWERVE!!!!!
                    * SwerveConstants.TOP_SPEED_METERS_PER_SEC
                    * 0.4;
                double ySpeed =
                    -m_yspeedLimiter.calculate(MathUtil.applyDeadband(targetYaw * kPVision_Turn, 0.03))
                    * SwerveConstants.TOP_SPEED_METERS_PER_SEC
                    * 0.4;
                    //SmartDashboard.putBoolean("setSwerve",true);
                    setSwerve(xSpeed, ySpeed, 0, fieldRelative); // should rot be rot not 0 here?
            }
            else { // if not aligning to target
                //pathTimerStop = PathUtil.getPathFromTagID(curAprilTagID, m_swerve, fieldRelative, getPeriod()).pathTimerStops().get(curPathStep-1);
                SmartDashboard.putNumber("check #",3);
                if (targetVisible) {
                    SmartDashboard.putNumber("check #",4);
                    if (!pathRunning) { // start path
                        SmartDashboard.putNumber("check #",5);
                        curPathStep = 1;
                        PathUtil.getPathFromTagID(curAprilTagID, m_swerve, fieldRelative, getPeriod(), this, targetYaw); // is targetYaw right here?
                        //Command a = () -> curPathCommand.schedule();
                        //curPathCommand = {() -> PathUtil.getPathFromTagID(curAprilTagID, m_swerve, fieldRelative, getPeriod(), this)};

                    }
                }
            }
        }
        else {
            fieldRelative = true;
            targetYaw = 0;
            setSwerve(-m_controller.getLeftY(), -m_controller.getLeftX(), -m_controller.getRightX(), fieldRelative);
        }
                    
  }
  private void manualControl() {
   //m_swerve.manualDrive(m_controller.getLeftY(), m_controller.getRightX());
  }
}
/*
if (targetVisible) {

}





*/
