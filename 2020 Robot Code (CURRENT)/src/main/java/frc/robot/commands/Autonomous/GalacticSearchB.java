// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.nio.file.Paths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FloorIntake;
import frc.robot.commands.FloorIntake.LowerFloorIntake;
import frc.robot.commands.FloorIntake.RaiseFloorIntake;
import frc.robot.commands.FloorIntake.RunFloorIntake;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Sorter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearchB extends SequentialCommandGroup {
  /** Creates a new GalacticSearchB. */
  public GalacticSearchB(Drive drive, Tower tower, Sorter sorter, FloorIntake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LowerFloorIntake(intake),
      new ParallelDeadlineGroup(  
        new TravelPath(Paths.get("/home/lvuser/deploy/galacticSearchBeginningB.wpilib.json"), drive, false),
        new RunFloorIntake(intake)
      )
    );

    if (tower.getBeamBreakThreeState())
    {
    // If this is true we are on the red path
      addCommands(
        new ParallelDeadlineGroup(  
          new TravelPath(Paths.get("/home/lvuser/deploy/galacticSearchBRed.wpilib.json"), drive, false),
          new RunFloorIntake(intake)
        )
      );
    }
    else 
    {
    // If this is running we are on the blue path
      addCommands(
        new ParallelDeadlineGroup(  
          new TravelPath(Paths.get("/home/lvuser/deploy/galacticSearchBBlue.wpilib.json"), drive, false),
          new RunFloorIntake(intake)
        )
      );
    }
    addCommands(new RaiseFloorIntake(intake));
  }
}
