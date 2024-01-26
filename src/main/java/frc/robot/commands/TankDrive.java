// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TankDrive extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveTrain drivetrain;
  private final CommandXboxController driver;

  private double driverLeftY;
  private double driverRightY;
  private double driverLeftX;
  private Boolean arcadeDriveActive;
  private final double driveSpeedCap;

  public TankDrive(DriveTrain drivetrain, CommandXboxController driver, boolean arcadeDriveActive) {
    this.drivetrain = drivetrain;
    this.driver = driver;
    this.arcadeDriveActive = arcadeDriveActive;
    this.driveSpeedCap = Constants.Drivetrain.kDriveSpeedCap;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    driverLeftX = 0;
    driverRightY = 0;
    driverLeftY = 0;
  }

  @Override
  public void execute() {
    driverLeftY = driver.getLeftY();
    driverRightY = driver.getRightY();
    driverLeftX = driver.getLeftX();

    if (arcadeDriveActive) {
      drivetrain.arcadeDrive(driverRightY * driveSpeedCap, driverLeftX * driveSpeedCap);
    } else {
      drivetrain.tankDrive(driverLeftY * driveSpeedCap, driverRightY * driveSpeedCap);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}