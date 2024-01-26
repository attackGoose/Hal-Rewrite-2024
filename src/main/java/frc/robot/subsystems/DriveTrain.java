// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Code to allow the robot to move.
 * 
 * <p>Mecanum Drive contains:
 * <p>- 2x CIM Motors on the LEFT side (front and back)
 * <p>- 2x CIM Motors on the RIGHT side (front and back)
 * 
 * @author Ahmed Osman
 */


public class DriveTrain extends SubsystemBase {

  private final Spark motorFL = new Spark(Constants.Ports.FRONTLEFT);
  private final Spark motorFR = new Spark(Constants.Ports.FRONTRIGHT);
  private final Spark motorBL = new Spark(Constants.Ports.BACKLEFT);
  private final Spark motorBR = new Spark(Constants.Ports.BACKRIGHT);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL, motorBL);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(motorFR, motorBR);

  private final DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);
  
  public DriveTrain() {
    drivetrain.setMaxOutput(0.95);
  }

   /**
   * Drive the robot! Uses traditional two Y-axes to move each side.
   * @param leftSpeed - The speed at which the left side motors should be.
   * @param rightSpeed - The speed at which the right side motors should be.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

   /**
   * Drive the robot! Uses more modern X/Y-axes to move and turn seperately.
   * @param xSpeed
   * @param rotation
   */

  public void arcadeDrive(double xSpeed, double rotation) {
    drivetrain.arcadeDrive(xSpeed, rotation);
  }

  /**
   * Drive the robot! Uses fed voltage inputs to move each side.
   * @param leftVoltage - The voltage at which the left side motors should be.
   * @param rightVoltage - The voltage at which the right side motors should be.
   */
  public void voltageDrive(double leftVoltage, double rightVoltage) {
    MathUtil.clamp(leftVoltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    MathUtil.clamp(rightVoltage, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());

    leftMotors.setVoltage(-leftVoltage);
    rightMotors.setVoltage(-rightVoltage);

    drivetrain.feed();
  }

  /**
   * Forces the motors to stop moving, stopping the robot.
   */
  public void driveStop() {
    drivetrain.stopMotor();
  }

  // VOLTAGE FUNCTIONS

  public double getLeftVoltage() {
    return (leftMotors.get() * RobotController.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (rightMotors.get() * RobotController.getBatteryVoltage());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain/Left Motors Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Drivetrain/Right Motors Voltage (V)", getRightVoltage());
    
    SmartDashboard.putNumber("Drivetrain/Left Motors Set Speed [-1,1]:", leftMotors.get());
    SmartDashboard.putNumber("Drivetrain/Right Motors Set Speed [-1,1]", rightMotors.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}