// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Motors;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  private final Motors motors = new Motors();

  public RobotContainer() {

    motors.setDefaultCommand(
      Commands.parallel(
          motors.run_standard_1(HumanControls.leftJoyY),
          motors.run_standard_2(HumanControls.RightJoyY)
      )
  );

    configureBindings();
  }

  private void configureBindings() {
      HumanControls.LT.whileTrue(motors.inverted_1(HumanControls.leftJoyY));
      HumanControls.RT.whileTrue(motors.inverted_2(HumanControls.RightJoyY));
      HumanControls.A.whileTrue(motors.velocity_1());
      HumanControls.B.whileTrue(motors.velocity_2());
      HumanControls.X.whileTrue(motors.reset());
      HumanControls.LB.whileTrue(motors.positionvoltageCommand_1(HumanControls.leftJoyY));
      HumanControls.RB.whileTrue(motors.positionvoltageCommand_2(HumanControls.RightJoyY));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
