// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
/** An example command that uses an example subsystem. */
public class ShooterwithJoystickCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
 // private final IntakeSubsystem ShooterwithJoystickCommand;
    double BottomMotor = 0;
    double TopMotor = 0;
    private ShooterSubsystem ShooterSubsystem;
    private double TalonFX2;
    private double TalonFX1;
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
 * @return 
   */
  public void ShooterwithJoystickCommand(ShooterSubsystem subsystem) {
    this.ShooterSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ShooterSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Starting Joystick drive command");
    TalonFX1 = 0;
    TalonFX2 = 0;
    
  } 

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (RobotContainer.joy1.getAButton()) {
      BottomMotor = 0.1;
      TopMotor = -0.1;
      //Shooter  Below
      TalonFX1 = 0.7;
      TalonFX2 = -0.7;
    } else {
      BottomMotor = 0;
      TopMotor = 0;
      //Shooter Below
    }
      ShooterSubsystem.ShooterwithJoystickCommand(TalonFX1, TalonFX2);
      
      

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
