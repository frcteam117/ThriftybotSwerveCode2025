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

package frc.robot.util;

//import static frc.robot.subsystems.drive.DriveConstants.maxSpeedMetersPerSec;
//import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.*;
import frc.robot.Robot;
import frc.robot.Drivetrain;

public class PathUtil {
  //public record Data(Command Commands,List<Double> targetPose) {};
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private PathUtil() {} // ADD WAY TO HAVE MULTIPLE PATHS FOR ONE APRILTAG???? MORE ARRAYS IDK
  
  public static void getPathFromTagID(int aprilTagID, Drivetrain drivetrain, Boolean fieldRelative, Double m_period, Robot robot, double targetYaw) {
    // commands for each tag: // use fieldRelative to determine if it should be fieldRelative or just offSet!!!!!!!!!
    if (aprilTagID == 0) {
      PathCommands.BlankCommand();
      //List<Double> targetPose = ;
      //Data data = new Data(Commands, targetPose);
    }
    else if (aprilTagID == 1) {
      PathCommands.Path1Command(drivetrain, fieldRelative, m_period, robot);
      //List<Double> targetPose = 1.0);
      //Data data = new Data(Commands, targetPose);
    }
    else if (aprilTagID == 2) {
      PathCommands.Path2Command(drivetrain, fieldRelative, m_period, robot);
      //List<Double> targetPose = 1.0,1.0);
      //Data data = new Data(Commands, targetPose);
    }
    else if (aprilTagID == 3) {
      PathCommands.AutoPrototype(drivetrain, fieldRelative, m_period, robot, targetYaw);
      //List<Double> targetPose = 1.0,1.0);
      //Data data = new Data(Commands, targetPose);
    }
    else {
      PathCommands.BlankCommand();
      //List<Double> targetPose = 0.0);
      //Data data = new Data(Commands, targetPose);
    }
    // other tags vvv
  }
}