// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.nio.file.Paths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BouncePath extends SequentialCommandGroup {
  /** Creates a new BouncePath. */
  public BouncePath(Drive drive) {
  
   super(
    new TravelPath(Paths.get("/home/lvuser/deploy/bounceSegmentOne.wpilib.json"), drive, false),
    new TravelPath(Paths.get("/home/lvuser/deploy/bounceSegmentTwo.wpilib.json"), drive, true),
    new TravelPath(Paths.get("/home/lvuser/deploy/bounceSegmentThree.wpilib.json"), drive, false),
    new TravelPath(Paths.get("/home/lvuser/deploy/bounceSegmentFour.wpilib.json"), drive, true)
   );
  }
}
