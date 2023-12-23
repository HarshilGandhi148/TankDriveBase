// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  
  // driveMotorControllers
  public CANSparkMax frontLeftMotor = new CANSparkMax(Constants.CAN_IDs.frontLeftID, MotorType.kBrushless);
  public CANSparkMax frontRightMotor = new CANSparkMax(Constants.CAN_IDs.frontRightID, MotorType.kBrushless);
  public CANSparkMax backLeftMotor = new CANSparkMax(Constants.CAN_IDs.backLeftID, MotorType.kBrushless);
  public CANSparkMax backRightMotor = new CANSparkMax(Constants.CAN_IDs.backRightID, MotorType.kBrushless);

  public AHRS gyro = new AHRS(Port.kMXP);

  // driveEncoders
  public RelativeEncoder frontLeftEncoder;
  public RelativeEncoder frontRightEncoder;
  public RelativeEncoder backLeftEncoder;
  public RelativeEncoder backRightEncoder;
 
  // motor controller groups
  public MotorControllerGroup leftMotorControllerGroup = new MotorControllerGroup(frontLeftMotor, backLeftMotor);
  public MotorControllerGroup righMotorControllerGroup = new MotorControllerGroup(frontRightMotor, backRightMotor);

  DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, righMotorControllerGroup);

  private final DifferentialDriveOdometry odometry;

  
  /** Creates a new Subsystem. */
  public DriveSubsystem() {
    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();

    frontLeftEncoder.setPositionConversionFactor(Constants.RobotConstants.linearConversion);
    frontRightEncoder.setPositionConversionFactor(Constants.RobotConstants.linearConversion);
    backLeftEncoder.setPositionConversionFactor(Constants.RobotConstants.linearConversion);
    backRightEncoder.setPositionConversionFactor(Constants.RobotConstants.linearConversion);

    frontLeftEncoder.setVelocityConversionFactor(Constants.RobotConstants.linearConversion/60);
    frontRightEncoder.setVelocityConversionFactor(Constants.RobotConstants.linearConversion/60);
    backLeftEncoder.setVelocityConversionFactor(Constants.RobotConstants.linearConversion/60);
    backRightEncoder.setVelocityConversionFactor(Constants.RobotConstants.linearConversion/60);

    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);
    
    resetEncoders();
    resetGyro();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
    odometry.resetPosition(gyro.getRotation2d(), 0, 0, new Pose2d());
    }

  public void setCurrentLimits(int currentLimit){
    frontLeftMotor.setSmartCurrentLimit(currentLimit);
    frontRightMotor.setSmartCurrentLimit(currentLimit);
    backLeftMotor.setSmartCurrentLimit(currentLimit);
    backRightMotor.setSmartCurrentLimit(currentLimit);
  }
  
  public void setRampRate(double rate){
    frontLeftMotor.setOpenLoopRampRate(rate);
    frontRightMotor.setOpenLoopRampRate(rate);
    backLeftMotor.setOpenLoopRampRate(rate);
    backRightMotor.setOpenLoopRampRate(rate);
  }

  public void setBrakeEnabled() {
    frontLeftMotor.setIdleMode(IdleMode.kBrake);
    frontRightMotor.setIdleMode(IdleMode.kBrake);
    backLeftMotor.setIdleMode(IdleMode.kBrake);
    backRightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastEnabled() {
    frontLeftMotor.setIdleMode(IdleMode.kCoast);
    frontRightMotor.setIdleMode(IdleMode.kCoast);
    backLeftMotor.setIdleMode(IdleMode.kCoast);
    backRightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void resetEncoders() {
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
  }

  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPosition())/2.0;
  }

  public void resetGyro () {
    gyro.reset();
  }

  public double getGyroYaw () {
    return gyro.getYaw() % 360;
  }

  public double getGyroPitch () {
    return gyro.getPitch() % 360;
  }

  public double getGyroHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public double getGyroTurnRate(){
    return -gyro.getRate();
  }

  public AHRS getGyro() {
    return gyro;
  }

  public void tankDrive(double forward, double rotation){
      differentialDrive.arcadeDrive(forward, rotation);
    }

  public double getRightEncoderPosition()
  {
    return (frontRightEncoder.getPosition() + backRightEncoder.getPosition())/2;
  }

  public double getLeftEncoderPosition()
  {
    return -(frontLeftEncoder.getPosition() + backLeftEncoder.getPosition())/2;
  }

  public double getRightEncoderVelocity()
  {
    return (frontRightEncoder.getVelocity() + backRightEncoder.getVelocity())/2;
  }

  public double getLeftEncoderVelocity()
  {
    return -(frontLeftEncoder.getVelocity() + backLeftEncoder.getVelocity())/2;
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose2d) {
    resetEncoders();
    odometry.resetPosition(gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose2d);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public void tankDriveVots(double left, double right) {
    frontLeftMotor.setVoltage(left);
    backLeftMotor.setVoltage(left);
    frontRightMotor.setVoltage(right);
    backRightMotor.setVoltage(right);
    differentialDrive.feed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
