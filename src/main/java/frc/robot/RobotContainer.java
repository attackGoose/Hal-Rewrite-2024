// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Ports;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Subsystems
  private final DriveTrain drivetrain = new DriveTrain();

  // Input Devices
  private final CommandXboxController driver = new CommandXboxController(Ports.kDriverController);

  public static SendableChooser<Boolean> controlChooser = new SendableChooser<>();
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();
  private boolean arcadeDriveActive;
  
  public RobotContainer() {
    configureDriverBindings();
    configureDefaultCommands();
    //configureAutonCommands();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureDriverBindings() {
    controlChooser.setDefaultOption("Tank Drive", false);
    controlChooser.addOption("Tank Drive", false);
    //controlChooser.addOption("Arcade Drive", true);

    SmartDashboard.putData("Control Scheme", controlChooser);
    arcadeDriveActive = controlChooser.getSelected();
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new TankDrive(drivetrain, driver, arcadeDriveActive));
  }

  /*private void configureAutonCommands() {
    autonChooser.addOption("Simple Drive Forward", Autos.simpleTimedDrive(drivetrain, Constants.Drivetrain.kTimedDriveTime));
    autonChooser.setDefaultOption("Simple Drive Forward", Autos.simpleTimedDrive(drivetrain, Constants.Drivetrain.kTimedDriveTime));
    
    SmartDashboard.putData("Selected Autonomous", autonChooser);
  }*/

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}