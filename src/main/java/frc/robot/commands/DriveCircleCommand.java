/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import edu.wpi.first.wpilibj.trajectory.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DriveCircleCommand extends SequentialCommandGroup {
  /**
   * Creates a new DriveCircleCommand.
   */
  
  private static final TrajectoryConfig config = new TrajectoryConfig(
    5.0, // Velocity     (m/s)
    8.0  // Acceleration (m/s^2)
  );
  
  private static final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    new Pose2d(10, 5, new Rotation2d(0)),  // Start
    List.of(
      // Interior Points
      new Translation2d(5, 10), 
      new Translation2d(0, 5),
      new Translation2d(5, 0)
    ),
    new Pose2d(10, 5, new Rotation2d(0)),  // End
    config 
  );
  
  public DriveCircleCommand(Drivetrain drivetrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DrivetrainRamseteCommand(drivetrain, trajectory));
  }
}
