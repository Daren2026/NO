// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


public class ShooterSubsystem extends SubsystemBase {

   private TalonFX TalonFX1 = new TalonFX(11);
   private TalonFX TalonFX2 = new TalonFX(43);

  

//CANSparkMax PivotPoint = new CANSparkMax (Constants.DrivetrainConstants.LeftBackCANID, CANSparkLowLevel.MotorType.kBrushless);

// RelativeEncoder leftEncoder = LeftFrontMotor.getEncoder();
// RelativeEncoder rightEncoder = RightFrontMotor.getEncoder();






//DifferentialDrive differentialDrive = new DifferentialDrive(leftMotorControllerGroup, RightMotorControllerGroup);
  public ShooterSubsystem() {
  

 
 


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
  public void IntakewithJoystickCommand(double Speed3, double Speed4) {
    

    TalonFX1.set(Speed3);
    TalonFX2.set(Speed4);
  }

public void ShooterwithJoystickCommand(double bottomMotor, double topMotor) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'ShooterwithJoystickCommand'");
}
  }
 