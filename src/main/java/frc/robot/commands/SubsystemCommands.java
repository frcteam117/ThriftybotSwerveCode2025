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
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SubsystemCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  private static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9); // import these from robot for better continuity?
    //===================================
 
    public static Command BlankCommand() {
        return Commands.runOnce( () -> {});
    }
    public static Command StopSwerve(Drivetrain drivetrain, Boolean fieldRelative, Double m_period) {
        //Drivetrain m_swerve,
        return Commands.runOnce( () -> {
                drivetrain.drive(0.0, 0.0, 0.0, fieldRelative, m_period); // add way to stop the robot?????
        });

    }
    //===================================
    // non-drivetrain subsystem commands:
    public static Command ExpandHopper(Integer stepInSequence) {
        return Commands.runOnce( () -> {});
    }
    public static Command RetractHopper(Integer stepInSequence) {
        return Commands.runOnce( () -> {});
    }
    public static Command SetShooterHoodAngle(Integer stepInSequence) {
        return Commands.runOnce( () -> {});
    }
    public static Command DeployIntake(Integer stepInSequence) {
        return Commands.runOnce( () -> {});
    }   
    public static Command RetractIntake(Integer stepInSequence) {// should this be UndeployIntake instead?
        return Commands.runOnce( () -> {});
    }
    public static Command IntakeFuel(Integer stepInSequence) { // 
        return Commands.runOnce( () -> {});
    }
    public static Command StopIntake(Integer stepInSequence) { // 
        return Commands.runOnce( () -> {});
    }
    public static Command RunLeftShooter(Integer stepInSequence) { // adjust this?
        return Commands.runOnce( () -> {});
    }
    public static Command RunRightShooter(Integer stepInSequence) { // adjust this?,
        return Commands.runOnce( () -> {});
    }
    public static Command StopLeftShooter(Integer stepInSequence) { // adjust this?
        return Commands.runOnce( () -> {});
    }
    public static Command StopRightShooter(Integer stepInSequence) { // adjust this?, 
        return Commands.runOnce( () -> {});
    }
    public static Command TowerAlign(Integer stepInSequence, String position) { // position will be like front left/center/right or side or back yknow
        return Commands.runOnce( () -> {});
    }
    public static Command ExtendClimber(Integer stepInSequence) { // dunno about this one
        return Commands.runOnce( () -> {});
    }
    public static Command RetractClimber(Integer stepInSequence) { // dunno about this one
        return Commands.runOnce( () -> {});
    }
    public static Command ClimbLevel1(Integer stepInSequence) {
        return Commands.runOnce( () -> {});
    }
    //public static Command ClimbLevel3() {
}