// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// error:
/*Could not find **any** PhotonVision coprocessors on NetworkTables. Double check that PhotonVision is running, and that your camera is connected!
Error at org.photonvision.PhotonCamera.verifyVersion(PhotonCamera.java:490): Could not find **any** PhotonVision coprocessors on NetworkTables. Double check that PhotonVision is running, and that your camera is connected!
PhotonVision coprocessor at path /photonvision/PC_Camera has not reported a message interface UUID - is your coprocessor's camera started? */
package frc.robot;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;   
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

  PhotonCamera camera;
  Timer timer;
  //Timer timer = new Timer();

  List<Integer> aprilTagIDs = Arrays.asList(6, 7);
  int curAprilTagID;

  double targetYaw;
  double targetRange; // from photonvision docs
  double kPVision_Turn;
  double pathTimerStop;

  public Robot () {
    Timer timer = new Timer();
    kPVision_Turn = -.03;
    targetYaw = (0.0);
    camera = new PhotonCamera("PC_Camera");
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
    if (m_controller.getSquareButtonPressed()) {
        m_swerve.resetFieldRelativeDirection();
    }

    if (m_controller.getCrossButton()) {
        m_swerve.setX();
    } else {
    driveWithJoystick(true);
    }
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
    boolean targetVisible = false;
    // Read in relevant data from the Camera
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (aprilTagIDs.contains(target.getFiducialId())) { 
                    // found one of the tags in aprilTagIDs
                    curAprilTagID = target.getFiducialId();
                    targetYaw = target.getYaw();
                    targetVisible = true;
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



    // Auto-align when requested
    if (m_controller.getSquareButton() && targetVisible) {
        // Driver wants auto-alignment to tag 7
        // And, tag 7 is in sight, so we can turn toward it.
        // Override the driver's turn command with an automatic one that turns toward the tag.
        //rotation = pid.calculate(targetYaw, 0);
        SmartDashboard.putBoolean("", true);
        fieldRelative = false;

        if (targetRange > 2) {
            double xSpeed =
                -m_xspeedLimiter.calculate(MathUtil.applyDeadband(targetRange * 0.5, 0.03)) // CONFIGURE STUFF SO U CAN TEST IF TS WORKS W/ SWERVE!!!!!
                * SwerveConstants.TOP_SPEED_METERS_PER_SEC
                * 0.4;
            double ySpeed =
                -m_yspeedLimiter.calculate(MathUtil.applyDeadband(targetYaw * kPVision_Turn, 0.03))
                * SwerveConstants.TOP_SPEED_METERS_PER_SEC
                * 0.4;
                setSwerve(xSpeed, ySpeed, 0, fieldRelative); // should rot be rot not 0 here?
        }
        else {
            List<Double> values = PathUtil.getValuesFromTagID(curAprilTagID);
            pathTimerStop = values.get(3);
            if (!timer.hasElapsed(0.01)) { // can 0.01 be 0? idk. who knows
                timer.start(); // Start the timer when autonomous begins
            }
            if (!timer.hasElapsed(pathTimerStop)) {
                setSwerve(values.get(0), values.get(1), values.get(2), fieldRelative);
            }
            else if (timer.hasElapsed(0.01)) {
                pathTimerStop = 0.0;
                timer.stop(); 
            }
            //setSwerve(values.get(0), values.get(1), values.get(2), fieldRelative);
        }
    }
    else {
        //setSwerve(-m_controller.getLeftY(), -m_controller.getLeftX(), -m_controller.getRightX(), fieldRelative);
    }
  }
 
  private void manualControl() {
   //m_swerve.manualDrive(m_controller.getLeftY(), m_controller.getRightX());
  }
}
