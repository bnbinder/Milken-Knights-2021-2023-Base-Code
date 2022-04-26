// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Stop;
import frc.robot.Turn;
import frc.robot.Constants.AUTO.DISTANGLE;
import frc.robot.MkSwerveTrain.ETHERAUTO;
import frc.robot.MkSwerveTrain.ETHERRCW;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Swrv extends SequentialCommandGroup {
  /** Creates a new Swrv. */
  public Swrv() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(//deadline(new Turn(((DISTANGLE.angleuno)))).withTimeout(1),
    deadline(new EtherAutoCommand(DISTANGLE.distanceuno, ((DISTANGLE.angleuno)), 0, 0, ETHERAUTO.Curve, ETHERRCW.Forever)).withTimeout(6));
             // deadline(new Stop()));
  }
}
