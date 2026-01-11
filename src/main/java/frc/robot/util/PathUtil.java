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

//import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class PathUtil {
  /*
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  */
  private PathUtil() {} // ADD WAY TO HAVE MULTIPLE PATHS FOR ONE APRILTAG???? MORE ARRAYS IDK
  
  public static List<List<Double>> getValuesFromTagID(int aprilTagID) {
    // commands for each tag:
    if (aprilTagID == 0) {
      List<List<Double>> values = Arrays.asList(Arrays.asList(0.0,0.0,0.0,0.0)); // 2 movements
      return values;
    }
    //1
    if (aprilTagID == 1) {
      List<List<Double>> values = Arrays.asList(Arrays.asList(0.0,0.05,0.05,2.0),Arrays.asList(0.05,0.0,0.05,2.0)); // 2 movements
      return values;
    }
    //2
    if (aprilTagID == 2) {
      List<List<Double>> values = Arrays.asList(Arrays.asList(0.05,0.0,0.05,2.0),Arrays.asList(0.0,0.05,0.05,2.0),Arrays.asList(0.05,0.05,0.0,2.0));
      // - 3 movements ^^^
      //List<List<Double>> thing = Arrays.asList(Arrays.asList(0.05,0.0,0.05,4.0));
      return values;
    }
    else {
      List<List<Double>> values = Arrays.asList(Arrays.asList(0.0,0.0,0.0,0.0)); // one list but 0 movements
      return values;
    }
    // other tags vvv
  }
}