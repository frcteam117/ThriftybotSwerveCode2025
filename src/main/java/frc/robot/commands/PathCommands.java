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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private static boolean CloseEnough(Pose2d curPose, Pose2d targetPose) { // gotta be a better way 2 do this but again idfk
        double difX = Math.abs(curPose.getX()-targetPose.getX());
        double difY = Math.abs(curPose.getY()-targetPose.getY());
        if ((difX+difY)/2 <= 0.1) {
            return true;
        }
        else { return false;}
    }

    private static List<Double> CalcSwerveValues(Pose2d curPose, Pose2d targetPose) {
        double difX = Math.abs(curPose.getX()-targetPose.getX());
        double difY = Math.abs(curPose.getY()-targetPose.getY());
        //
        if (difX > 1) {difX = 1;};
        if (difY > 1) {difY = 1;};
        //
        double xSpeed = 0;
        double ySpeed = 0;
        double rot = 0; // add this later idk man
        //Rotation2d difRot = Pose2d.getRotation();
        if (!(difX == 0) && !(difY == 0)) {
             xSpeed = (difX / difY) * limiter;
             ySpeed = (difY / difX) * limiter;
        }
        else {
            if ((difX == 0) && (difY == 0))  {
             xSpeed = 0 * limiter;
                 ySpeed = 0 * limiter;
            }
            if (difX == 0) {
                 xSpeed = 1 * limiter;
                 ySpeed = 0 * limiter;
            };
            if (difY == 0) { 
                 xSpeed = 0 * limiter;
                 ySpeed = 1 * limiter;
            };
        }
        // is this math right?????
        List<Double> values = Arrays.asList(xSpeed,ySpeed,rot);
        return values;

    }


    private PathCommands() {} // ADD A TIME FUNCTION???????

    public static Command BlankCommand() {
        return Commands.sequence(
            Commands.runOnce (
                () -> {}
            )
        );
    }
    public static Command StopSwerve(Drivetrain drivetrain, Boolean fieldRelative, Double m_period) {
        //Drivetrain m_swerve,

        return Commands.sequence(
            Commands.runOnce (
                () -> {
                    drivetrain.drive(0.0, 0.0, 0.0, fieldRelative, m_period); // add way to stop the robot?????
                }
            )
        );

    }

    // path commands vvv

    public static Command Path1Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot) {
        //Drivetrain m_swerve,
        //Drivetrain m_swerve,
        condition = false;
        end = false;
        return Commands.sequence(
            Commands.run(
                () -> {
                    condition = false;
                    Pose2d targetPose = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0));
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);

                    if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                        drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                    }
                    else {
                        Path1Command(drivetrain, fieldRelative, m_period, robot).cancel(); // does this actually stop the command???? IDK
                        condition = true;
                    }
                }
            ),
            Commands.run(
                () -> {
                    if (condition) {
                        condition = false;
                        Pose2d targetPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0));
                        List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                        if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                            drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                        }
                        else {
                            Path1Command(drivetrain, fieldRelative, m_period, robot).cancel(); // does this actually stop the command???? IDK
                            end = true;

                        }
                    }
                }
            ),
            Commands.run(
                () -> {
                    if (end) {
                        robot.pathRunning = false; // does tthis actually change the variable in Robot.java???? idk man
                        condition = false;
                        end = false;
                    }
                }
            )
        );

    }

    public static Command Path2Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot) {
        //Drivetrain m_swerve,
        condition = false;
        end = false;
        return Commands.sequence(
            Commands.run(
                () -> {
                    condition = false;
                    Pose2d targetPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0));
                    List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                    if (!CloseEnough(drivetrain.getPose(), targetPose)) {

                        drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                    }
                    else {
                        Path1Command(drivetrain, fieldRelative, m_period, robot).cancel(); // does this actually stop the command???? IDK
                        condition = true;
                    }
                }
            ),
            Commands.run(
                () -> {
                    if (condition) {
                        condition = false;
                        Pose2d targetPose = new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0));
                        List<Double> values = CalcSwerveValues(drivetrain.getPose(), targetPose);
                        if (!CloseEnough(drivetrain.getPose(), targetPose)) {
                            drivetrain.drive(values.get(0), values.get(1), values.get(2), fieldRelative, m_period); 
                        }
                        else {
                            Path1Command(drivetrain, fieldRelative, m_period, robot).cancel(); // does this actually stop the command???? IDK
                            end = true;

                        }
                    }
                }
            ),
            Commands.run(
                () -> {
                    if (end) {
                        robot.pathRunning = false; // does tthis actually change the variable in Robot.java???? idk man
                        condition = false;
                        end = false;
                    }
                }
            )
        );

    }
    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    }