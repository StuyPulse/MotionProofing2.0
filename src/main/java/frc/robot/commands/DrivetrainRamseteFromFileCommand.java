/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainRamseteFromFileCommand extends DrivetrainRamseteCommand {

  public DrivetrainRamseteFromFileCommand(Drivetrain drivetrain, String pathStr) throws IOException {
    super(drivetrain, TrajectoryUtil.fromPathweaverJson(
      Constants.DEPLOY_DIR.resolve(pathStr)
    )); 
  }

}
