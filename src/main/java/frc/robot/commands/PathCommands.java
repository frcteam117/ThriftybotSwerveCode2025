// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
//
import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.generated.SwerveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//
import edu.wpi.first.wpilibj.Timer;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.time.Period;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PathCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  //
  public static boolean condition = false; // this whole system is dogshit but this is prolly the worst part. its funny tho
  public static double limiter = 0.1;
  //- u need another condition for each in the sequence, ig you could make it a list and change the # in each sequence part? sure idk gn
  public static boolean end = false;
  Timer timer;
    //Drivetrain m_swerve; // does this just work????????
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9);
    //
    //
    static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    //static List<Pose3d> AprilTagPoses = Robot.AprilTagPoses;
    //
    //--------------------------------------------
    private static boolean CloseEnough(Pose2d curPose, Pose2d targetPose) { // gotta be a better way 2 do this but again idfk
        double difX = Math.abs(targetPose.getX())-Math.abs(curPose.getX()); 
        double difY = Math.abs(targetPose.getY())-Math.abs(curPose.getY()); 
        if ((Math.abs(difX)+Math.abs(difY))/2 <= 0.1) {
            return true;
        }
        else { return false;}
    }

    private static List<Double> CalcSwerveValues(Pose2d curPose, Pose2d targetPose) {
        double difX = Math.abs(targetPose.getX())-Math.abs(curPose.getX()); 
        double difY = Math.abs(targetPose.getY())-Math.abs(curPose.getY()); 
        double difRot = targetPose.getRotation().getDegrees()-curPose.getRotation().getDegrees();
        //
        //
        double xSpeed = 0;
        double ySpeed = 0;
        double rot = 0; // add this later idk man
        //Rotation2d difRot = Pose2d.getRotation();
        xSpeed = difX * limiter;
        ySpeed = difY * limiter;
        rot = difRot;
        // is this math right?????
        List<Double> values = Arrays.asList(xSpeed,ySpeed,rot);
        return values;

    }

    public static void BlankCommand() {

    }
    public static void StopSwerve(Drivetrain drivetrain, Boolean fieldRelative, Double m_period) {
        //Drivetrain m_swerve,
                drivetrain.drive(0.0, 0.0, 0.0, fieldRelative, m_period); // add way to stop the robot?????

    }

    // path commands vvv
    // IDK IF I HAVE TO ADD .relativeTo TO THE END OF ALL THE POSE OR NOT??????????????????
    public static void Path1Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot) {
        //Drivetrain m_swerve,
        //Drivetrain m_swerve,
        //SmartDashboard.putNumber("AprilTag field pose - X",AprilTagPoses.get(1).getX());
        /* */
        SmartDashboard.putBoolean("running Path1Command",true);
        condition = false;
        end = false;
                SmartDashboard.putBoolean("condition1",true);
                condition = false;
                Pose2d targetPose = new Pose2d(-0.5, 0.5, Rotation2d.fromDegrees(0));
                List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);

                if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                    drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                }
                else {
                    condition = true;
                }
                if (condition) {
                    SmartDashboard.putBoolean("condition2",true);
                    condition = false;
                    targetPose = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0));
                    values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                    if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                        drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                    }
                    else {
                        end = true;

                    }
                }
                if (end) {
                    robot.pathRunning = false; // does tthis actually change the variable in Robot.java???? idk man
                    condition = false;
                    end = false;
                }

    }

    public static void Path2Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot) {
        //Drivetrain m_swerve,
        //Drivetrain m_swerve,
        /* */
        SmartDashboard.putBoolean("running Path1Command",true);
        condition = false;
        end = false;
                SmartDashboard.putBoolean("condition1",true);
                condition = false;
                Pose2d targetPose = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(0));
                List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);

                if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                    drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                }
                else {
                    condition = true;
                }
                if (condition) {
                    SmartDashboard.putBoolean("condition2",true);
                    condition = false;
                    targetPose = new Pose2d(-0.5, -0.5, Rotation2d.fromDegrees(0));
                    values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                    if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                        drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                    }
                    else {
                        end = true;

                    }
                }
                if (end) {
                    robot.pathRunning = false; // does tthis actually change the variable in Robot.java???? idk man
                    robot.curAprilTagID = 0;
                    condition = false;
                    end = false;
                }
            }
    
    public static void AutoPrototype(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot, Double targetYaw) {
        //Drivetrain m_swerve,
        //Drivetrain m_swerve,
        /* */
        //Pose2d targetPose = new Pose2d(-0.5, 0.5, Rotation2d.fromDegrees(0));
        int targetTagID = 2;
        SmartDashboard.putBoolean("running AutoPrototype",true);
        condition = false;
        end = false;
                SmartDashboard.putBoolean("condition1",true);
                condition = false;

                Pose2d targetPose = new Pose2d(
                    Robot.AprilTagPoses.get(targetTagID).getX(), // go to a tag
                    Robot.AprilTagPoses.get(targetTagID).getY(),
                    Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
                );
                
                List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);

                if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                    drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                }
                else {
                    condition = false;
                    end = true;
                }
                if (end) {
                    robot.pathRunning = false; // does tthis actually change the variable in Robot.java???? idk man
                    robot.curAprilTagID = 0;
                    condition = false;
                    end = false;
                }

            }
    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    }