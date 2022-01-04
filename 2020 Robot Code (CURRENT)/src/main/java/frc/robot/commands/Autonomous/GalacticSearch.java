// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.nio.file.Paths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Tower;
import frc.robot.commands.FloorIntake.RunFloorIntake;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearch extends SequentialCommandGroup {
  /** Creates a new GalacticSearch. */
  public GalacticSearch(Drive drive, FloorIntake floor, Tower tower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new TravelPath(Paths.get("/home/lvuser/deploy/bRed.wpilib.json"), drive, false),
        new RunFloorIntake(floor)
      )
    );

    if (tower.getBeamBreakThreeState())
    {
      // If this is true we are on the red B path
      
      addCommands(
        new ParallelDeadlineGroup(
          new TravelPath(Paths.get("/home/lvuser/deploy/bRedFinish.wpilib.json"), drive, false),
          new RunFloorIntake(floor)
        )
      );
    }
    else {
        addCommands(
        new ParallelDeadlineGroup(
          new TravelPath(Paths.get("/home/lvuser/deploy/aRed.wpilib.json"), drive, false),
          new RunFloorIntake(floor)
        )
      );

      if (tower.getBeamBreakThreeState())
      {
        // If this is true we are on the red A path
        
        addCommands(
          new ParallelDeadlineGroup(
            new TravelPath(Paths.get("/home/lvuser/deploy/aRedFinish.wpilib.json"), drive, false),
            new RunFloorIntake(floor)
          )
        );
      }
      else {
        addCommands(
          new ParallelDeadlineGroup(
            new TravelPath(Paths.get("/home/lvuser/deploy/bBlue.wpilib.json"), drive, false),
            new RunFloorIntake(floor)
          )
        );

        if (tower.getBeamBreakThreeState())
        {
          // If this is true we are on the blue B path
          
          addCommands(
            new ParallelDeadlineGroup(
              new TravelPath(Paths.get("/home/lvuser/deploy/bBlueFinish.wpilib.json"), drive, false),
              new RunFloorIntake(floor)
            )
          );
        }
        else {
          addCommands(
            new ParallelDeadlineGroup(
              new TravelPath(Paths.get("/home/lvuser/deploy/aBlue.wpilib.json"), drive, false),
              new RunFloorIntake(floor)
            )
          );

          if (tower.getBeamBreakThreeState())
          {
            // If this is true we are on the blue A path
            
            addCommands(
              new ParallelDeadlineGroup(
                new TravelPath(Paths.get("/home/lvuser/deploy/aBlueFinish.wpilib.json"), drive, false),
                new RunFloorIntake(floor)
              )
            );
          }
        }
      }
    }
  }
}
