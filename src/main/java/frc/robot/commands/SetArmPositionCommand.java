package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class SetArmPositionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  private final double targetAngleDegrees;
  
  public SetArmPositionCommand(ArmSubsystem armSubsystem, double targetAngleDegrees) {
    this.armSubsystem = armSubsystem;
    this.targetAngleDegrees = targetAngleDegrees;
    addRequirements(armSubsystem);
  }
  
  @Override
  public void initialize() {
    armSubsystem.setTargetPosition(targetAngleDegrees);
    System.out.println("Moving arm to " + targetAngleDegrees + " degrees");
  }
  
  @Override
  public void execute() {}
  
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Arm movement interrupted at " + 
                        armSubsystem.getAngleDegrees() + " degrees");
    } else {
      System.out.println("Arm reached target: " + targetAngleDegrees + " degrees");
    }
  }
  
  @Override
  public boolean isFinished() {
    return armSubsystem.atTargetPosition();
  }
}