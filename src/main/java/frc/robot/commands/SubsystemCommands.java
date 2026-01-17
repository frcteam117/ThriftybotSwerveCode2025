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
  //
  //public static boolean condition = false; // this whole system is dogshit but this is prolly the worst part. its funny tho
  public static double limiter = 0.2;
  public static double speedCap = 0.5;
  //- u need another condition for each in the sequence, ig you could make it a list and change the # in each sequence part? sure idk gn
  public static boolean end = false;
  private static final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1);
  private static final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(9); // import these from robot for better continuity?
  Timer timer;
    //Drivetrain m_swerve; // does this just work????????
    //
    //
    static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static int curMovementStep = 1;
    public static int curSequenceStep = 1;
    public static int totalSequenceSteps = 0; // prolly a better way to do this using another overarchig
    // - sequenceSteps method? but idk i'll wait till i have more motivation
    //static List<Pose3d> AprilTagPoses = Robot.AprilTagPoses;
    //
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
    public static Command IntakeFuel(Integer stepInSequence) {
        return Commands.runOnce( () -> {});
    }
    public static Command RunLeftShooter(Integer stepInSequence) { // adjust this?
        return Commands.runOnce( () -> {});
    }
    public static Command RunRightShooter(Integer stepInSequence) { // adjust this?
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

    //}

    // path commands vvv
    // IDK IF I HAVE TO ADD .relativeTo TO THE END OF ALL THE POSE OR NOT??????????????????
    /* 
    public static Command Path1Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot) {
        SmartDashboard.putBoolean("running Path1Command",true);
        totalSequenceSteps = 1;
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(-0.1, 0.1, Rotation2d.fromDegrees(0)),
        new Pose2d(0.1, -0.1, Rotation2d.fromDegrees(0)));
        pathSteps(drivetrain, fieldRelative, m_period, robot,
        2, 1, targetPoses); //

    }

    public static Command Path2Command(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot) {
        totalSequenceSteps = 1;
        SmartDashboard.putBoolean("running Path2Command",true);
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(0.1, -0.1, Rotation2d.fromDegrees(0)),
        new Pose2d(-0.1, 0.1, Rotation2d.fromDegrees(0)));
        pathSteps(drivetrain, fieldRelative, m_period, robot,
        2, 1,targetPoses); //
    }
    
    public static Command DriveToCenterFromOrigin(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, 
    Robot robot, Double targetYaw) {
        //if (alliance.isPresent()) { // maybe put this back? maybe just hope & pray
            //if (alliance.get() == Alliance.Red) // add alliance specific stuff l8r idgaf rn
            totalSequenceSteps = 1;
                    int targetTagID = 12;
                    SmartDashboard.putBoolean("running AutoPrototype",true);
                    List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
                        Robot.AprilTagPoses.get(targetTagID).getX(), // go to a tag
                        Robot.AprilTagPoses.get(targetTagID).getY(),
                        Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
                    ),
                    new Pose2d(6.5, 0.6, Rotation2d.fromDegrees(0)),
                    new Pose2d(8.3, 4, Rotation2d.fromDegrees(0))
                    );
        //    }
    }
    public static Command AutoPrototype2(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot, Double targetYaw) {
        int targetTagID = 3;
        totalSequenceSteps = 2;
        SmartDashboard.putBoolean("running AutoPrototype",true);
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
            Robot.AprilTagPoses.get(targetTagID).getX(), // go to a tag
            Robot.AprilTagPoses.get(targetTagID).getY(),
            Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
        )
        );
        pathSteps(drivetrain, fieldRelative, m_period, robot, 1, 1,targetPoses); //
        if (CloseEnough(drivetrain.getPose(),
        new Pose2d(8.3, 4, Rotation2d.fromDegrees(0)) 
        ) && !robot.pathRunning) { // if at target pose and no path is running
            // run intake methods here or whatever
        }
    } // the autos should probably be in their own folder but idgaf rn ^^ vvvvvvv
    // add a way to autoset the robots position to be wherever it is t the start of the game,
    // - or have it detect it based on what tags it can see???? <------------- do THISSSSSSSSSSSSSSSSSS
    public static Command ShootThenClimbAuto(Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot, Double targetYaw) {
        int targetTagID = 3;
        totalSequenceSteps = 2;
        SmartDashboard.putBoolean("running AutoPrototype",true);
        List<Pose2d> targetPoses = Arrays.asList(new Pose2d(
            Robot.AprilTagPoses.get(targetTagID).getX(), // go to a tag
            Robot.AprilTagPoses.get(targetTagID).getY(),
            Rotation2d.fromDegrees(targetYaw) //does this need to be the difference of smth? idk
        )
        );
        pathSteps(drivetrain, fieldRelative, m_period, robot, 1, 1, targetPoses); //
        if (CloseEnough(drivetrain.getPose(),
        new Pose2d(8.3, 4, Rotation2d.fromDegrees(0)) 
        ) && !robot.pathRunning) { // if at target pose and no path is running
            // run intake methods here or whatever
        }
    }
        */


}