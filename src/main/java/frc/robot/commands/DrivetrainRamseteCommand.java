/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.Drivetrain.*;
import frc.robot.subsystems.Drivetrain;

public class DrivetrainRamseteCommand extends RamseteCommand {
  /*
   * This command is a wrapper for the RamseteCommand 
   * While the RamseteCommand aims to be as robot agnostic as possible 
   * Many of these parameters are the same for one robot 
   * and can be condensed into one command
   */
  public DrivetrainRamseteCommand(Drivetrain drivetrain, Trajectory trajectory) {
    super(
      trajectory,
      drivetrain::getPose,
      new RamseteController(),  
      new SimpleMotorFeedforward(FF.ks, FF.kv, FF.ka), 
      drivetrain.getKinematics(), 
      drivetrain::getWheelSpeeds, 
      new PIDController(PID.kp, PID.ki, PID.kd),
      new PIDController(PID.kp, PID.ki, PID.kd),
      drivetrain::tankDriveVolts, 
      drivetrain
    ); 
  }

}


