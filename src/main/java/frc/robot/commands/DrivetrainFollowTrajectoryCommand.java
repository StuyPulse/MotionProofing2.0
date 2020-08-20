/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Drivetrain.FF;
import frc.robot.Constants.Drivetrain.PID;
import frc.robot.subsystems.Drivetrain;


public class DrivetrainFollowTrajectoryCommand extends CommandBase {
  /*
   * Stuypulse's implementation of a trajecotry folllower 
   * Intended to be used for debugging 
   */
  private Drivetrain drivetrain; 
  private Trajectory trajectory; 

  private Timer timer;

  private SimpleMotorFeedforward feedforward; 
  private PIDController feedback; 
    
  public DrivetrainFollowTrajectoryCommand(Drivetrain drivetrain, Trajectory trajectory) {
    this.drivetrain = drivetrain; 
    this.trajectory = trajectory;  
    this.feedforward = new SimpleMotorFeedforward(FF.ks, FF.kv, FF.ka); 
    this.feedback = new PIDController(PID.kp, PID.ki, PID.kd);
    timer = new Timer();
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    trajectory = trajectory.relativeTo(drivetrain.getPose()); 

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State currentState = trajectory.sample(timer.get());
    
    // FIXME: figure out how to get vertical velocity (vy)
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      currentState.velocityMetersPerSecond, 
      0,
      currentState.curvatureRadPerMeter
    ); 

    DifferentialDriveWheelSpeeds wheelSpeeds =
      drivetrain.getKinematics().toWheelSpeeds(chassisSpeeds);

    double leftVolts = feedforward.calculate(wheelSpeeds.leftMetersPerSecond); 
    double rightVolts = feedforward.calculate(wheelSpeeds.rightMetersPerSecond); 
  
    double distanceError = currentState.poseMeters.getTranslation().getDistance(
      drivetrain.getPose().getTranslation()
    ); 
    
    leftVolts += feedback.calculate(distanceError);
    rightVolts += feedback.calculate(distanceError);

    drivetrain.tankDriveVolts(leftVolts, rightVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop(); 
    drivetrain.stop(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
