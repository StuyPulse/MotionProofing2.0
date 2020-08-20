/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Drivetrain.*;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax leftFrontMotor, rightFrontMotor;
  private CANSparkMax leftBackMotor, rightBackMotor;

  private SpeedControllerGroup leftMotors;
  private SpeedControllerGroup rightMotors;

  private DifferentialDrive motors;

  private CANEncoder leftEncoder, rightEncoder;

  private AHRS navx;

  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;

  public Drivetrain() {

    // Front motors
    leftFrontMotor = new CANSparkMax(LEFT_TOP, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(LEFT_BOTTOM, MotorType.kBrushless);

    // Back motors
    leftBackMotor = new CANSparkMax(RIGHT_TOP, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(RIGHT_BOTTOM, MotorType.kBrushless);

    // Controller groups
    SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
    SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightFrontMotor, rightBackMotor);

    motors = new DifferentialDrive(leftMotors, rightMotors);

    // Encoders
    leftEncoder = leftBackMotor.getEncoder();
    rightEncoder = rightBackMotor.getEncoder();
    resetEncoders();
    setEncoderFactor(ENCODER_FACTOR);

    navx = new AHRS(SPI.Port.kMXP);

    // Kinematics
    kinematics = new DifferentialDriveKinematics(Units.feetToMeters(TRACK_WIDTH));
    // Odometry
    odometry = new DifferentialDriveOdometry(getAngle());
  }

  @Override
  public void periodic() {
    odometry.update(getAngle(), Units.feetToMeters(getLeftDistance()), Units.feetToMeters(getRightDistance()));
  }

  // Return a Rotation2d object from [the initial heading of the robot?]
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(navx.getAngle(), 360) * -1);
  }

  // Set the encoder factors
  private void setEncoderFactor(double factor) {
    leftEncoder.setPositionConversionFactor(factor);
    rightEncoder.setPositionConversionFactor(factor);
  }

  // Reset the encoders
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  // Return left distance
  public double getLeftDistance() {
    return leftEncoder.getPosition();
  }

  // Return right distance
  public double getRightDistance() {
    return rightEncoder.getPosition();
  }

  // Return average distance
  public double getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  // Return the left velocity of the drivetrain
  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }

  // Return the right velocity of the drivetrain
  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }

  // Return the velocity of the drivetrain
  public DifferentialDriveWheelSpeeds getWheelSpeeds() { 
    return new DifferentialDriveWheelSpeeds(
      Units.feetToMeters(getLeftVelocity()), 
      Units.feetToMeters(getRightVelocity())
    );   
  }

  // Reset the Gyro
  public void resetGyro() {
    navx.reset();
  }

  // Return a Pose2d object of the current position
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  // Set the voltage of speedcontrollahs
  public void tankDriveVolts(double lVolts, double rVolts) {
    leftMotors.setVoltage(lVolts);
    rightMotors.setVoltage(-rVolts);
    motors.feed();
  }

  public void stop() {
    tankDriveVolts(0, 0);
  }

  // Arcade drive
  public void arcadeDrive(double speed, double rotation) {
    motors.arcadeDrive(speed, rotation, false);
  }

  // Return drivetrain kinematics
  public DifferentialDriveKinematics getKinematics() {
    return kinematics;   
  }
}























