package frc.robot;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AUTO.DISTANGLE;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwrvTest extends SequentialCommandGroup {
  /** Creates a new SwrvTest. */
  public SwrvTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      deadline(new Turn(0)).withTimeout(1),
      deadline(new EtherAutoCommand(DISTANGLE.distanceA, DISTANGLE.distance, ((DISTANGLE.angle)),   1, 360)).withTimeout(6));
  }
}
