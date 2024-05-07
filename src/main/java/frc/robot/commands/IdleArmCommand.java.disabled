// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;


public class IdleArmCommand extends Command {
  
  private final ArmSubsystem m_arm;
  private boolean canPIDEnd;
  private boolean canWristReset;

  public IdleArmCommand(ArmSubsystem Marm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = Marm;
    addRequirements(Marm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setArmPIDCommand(ArmConstants.ArmState.IDLE);
    canPIDEnd = true;
    canWristReset = false;
    SmartDashboard.putBoolean("isIdleArmRunning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if(m_arm.getWristEncoder() <= 2.0 && canPIDEnd) {
      m_arm.PIDFallin();
      canPIDEnd = false;
    }

    if (Math.abs(m_arm.getWristVelocity()) <= 0.05 && (canWristReset == false) && (canPIDEnd == false)) {
      canWristReset = true;
    }*/
  }

  // Called once the command ends or is interrupted.
   
  @Override
  public void end(boolean interrupted) {
    /*if(canWristReset == true) {
      m_arm.resetWrist();
    }
    SmartDashboard.putBoolean("isIdleArmRunning", false);*/
  }

}
