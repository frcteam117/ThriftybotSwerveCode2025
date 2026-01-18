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
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
  public static double limiter = 0.2;
  public static double speedCap = 0.5;
  private static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9); // import these from robot for better continuity?
  Timer timer;
    //Drivetrain m_swerve; // does this just work????????
    //
    //
    static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private static void setSwerve(Drivetrain drivetrain, double m_period, double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
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
        drivetrain.drive(a, b, c, fieldRelative, m_period);
    }

    private static boolean CloseEnough(Pose2d curPose, Pose2d targetPose) { // gotta be a better way 2 do this but again idfk
        double difX = Math.abs(targetPose.getX())-Math.abs(curPose.getX()); 
        double difY = Math.abs(targetPose.getY())-Math.abs(curPose.getY()); 
        if ((Math.abs(difX)+Math.abs(difY))/2 <= 0.2) {
            double difRot = Math.abs(
                targetPose.getRotation().getDegrees())
                - Math.abs(curPose.getRotation().getDegrees()); 
            if (difRot <= 5) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
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
        if (xSpeed > speedCap) {
            xSpeed = speedCap;
        }
        if (ySpeed > speedCap) {
            ySpeed = speedCap;
        }
        // is this math right?????
        List<Double> values = Arrays.asList(xSpeed,ySpeed,rot);
        return values;

    }

    public static Command BlankCommand() {
        return Commands.runOnce( () -> {});
    }
    public static Command StopSwerve(Drivetrain drivetrain, Boolean fieldRelative, Double m_period) {
        //Drivetrain m_swerve,
        return Commands.runOnce( () -> {
                drivetrain.drive(0.0, 0.0, 0.0, fieldRelative, m_period); // add way to stop the robot?????
        });

    }
//===========================================================================================================
    // IDK IF I HAVE TO ADD .relativeTo TO THE END OF ALL THE POSE OR NOT??????????????????

    public static Command Path1Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period) {
        Pose2d targetPose = new Pose2d(-0.1, 0.1, Rotation2d.fromDegrees(0));
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                    setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPose))
        );
    }

    public static Command Path2Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period) {
        Pose2d targetPose = new Pose2d(0.1, -0.1, Rotation2d.fromDegrees(0));
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                    setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPose))
        );
    }
    
    public static Command DriveToCenterFromOrigin(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw) {
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
            Robot.AprilTagPoses.get(12).getX(), // go to a tag
            Robot.AprilTagPoses.get(12).getY(),
            Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
        ),
        new Pose2d(6.5, 0.6, Rotation2d.fromDegrees(0)),
        new Pose2d(8.3, 4, Rotation2d.fromDegrees(0))
        );        
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0));
                    setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
            //
            Commands.run(() -> {
                List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(1));
                setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(1))),

            Commands.run(() -> {
                List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(2));
                setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(2)))
        );
    }

    public static Command AutoPrototype2(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw) {
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
            Robot.AprilTagPoses.get(3).getX(), // go to a tag
            Robot.AprilTagPoses.get(3).getY(),
            Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
        ));        
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0));
                    setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
            //
            Commands.run(() -> {
                // run intake for 5 seconds
            }).withTimeout(5)
        );
    }
    // differentiate autos into their own folder at some point!
    public static Command ShootThenClimbAuto(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw) {
        List<Pose2d> targetPoses = Arrays.asList(
            new Pose2d(1.678, 3.75, Rotation2d.fromDegrees(0))
        );
        return Commands.sequence(
            Commands.run(() -> {
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPoses.get(0));
                    setSwerve(drivetrain, m_period, values.get(0), values.get(1), values.get(2),fieldRelative);
            }).until(() -> CloseEnough(drivetrain.getPose(),targetPoses.get(0))),
            //
            Commands.runOnce(() -> {
                // run climbLevel1
            })
        );
    }
}