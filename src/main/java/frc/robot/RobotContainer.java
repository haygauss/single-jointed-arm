package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {
  private final ArmSubsystem armSubsystem = new ArmSubsystem();
  private final CommandXboxController controller = new CommandXboxController(0);
  
  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {
    
    controller.a().onTrue(
        Commands.runOnce(() -> armSubsystem.setTargetPosition(-70), armSubsystem)
    );
    
    
    controller.b().onTrue(
        Commands.runOnce(() -> armSubsystem.setTargetPosition(0), armSubsystem)
    );
    

    controller.x().onTrue(
        Commands.runOnce(() -> armSubsystem.setTargetPosition(50), armSubsystem)
    );
    
    
    controller.y().onTrue(
        Commands.runOnce(() -> armSubsystem.setTargetPosition(100), armSubsystem)
    );
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}