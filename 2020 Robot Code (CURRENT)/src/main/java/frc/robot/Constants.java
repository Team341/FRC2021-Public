/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	

  public static class Hanger {
		public static final int WINCH_LEFT_MOTOR_PORT = 0; // Originally 22, switched for intake motor
		public static final int WINCH_RIGHT_MOTOR_PORT = 13;
		
		public static final int ELEVATOR_MOTOR_PORT = 18;

		public static final double ELEVATOR_CONVERSION_FACTOR = 4.0 / 49.0;		// inches/revolution
		public static final double WINCH_CONVERSION_FACTOR = 0.5 / 30; //1.0 / (0.5 / 1.0 * 1.0 / 30.0 * 1.0 / 42.0);			// inches/tick

		public static final double RUN_HANGER_SPEED = 0.1;
		public static final double RUN_WINCH_SPEED = 1.0;
		public static final double UNWINCH_SPEED = -0.5;
		public static final double HANGER_DISTANCE_TOLERANCE_INCHES = 1.0;

		public static final int WINCH_MOTOR_CURRENT_LIMIT = 40;

		public static final int ELEVATOR_MOTOR_CURRENT_LIMIT = 30;

		public static final double ELEVATOR_HEIGHT_THRESHOLD = 1.0;
	}
	
	public static class HangerHeights {
		public static final double LOW_BAR_HEIGHT = 9.294539;
		public static final double MID_BAR_HEIGHT = 11.883125;
		public static final double HIGH_BAR_HEIGHT = 18.098059; 
	}

    public static class Shooter {
      public static final int SHOOTER_MOTOR_PORT_LEFT = 21;//11;
	  public static final int SHOOTER_MOTOR_PORT_RIGHT = 14;
	  public static final int CAN_ENCODER_PORT = 2;
	  public static final int rows = 9;
	  public static final int cols = 2;
	  public static final double RPM_COEFFICIENT = 1.0 / 6.0;
	  public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 60;
	  public static final double DEFAULT_SHOOTER_RPM = 4000.0;
	  public static final double BASE_RPM_SPEED = 3500.0;
	  public static final int RPM_BOOST_CYCLE_COUNT = 13;
	  public static final double RPM_BOOST = 150.0; //rpm
    }    
       
    public static class Solenoids{
      public static final int LEFT_PTO_A = 0;
      public static final int LEFT_PTO_B = 1;
      public static final int CLIMBER_RELEASE = 2;
      public static final int INTAKE = 3;
      public static final int RIGHT_POPPER_A = 4;
      public static final int RIGHT_POPPER_B = 5;
      public static final int STATUS_LIGHT = 7;
    }
  
    public static class Drive{
      public static final int LEFT_ENCODER_A_PORT = 0;
      public static final int LEFT_ENCODER_B_PORT = 1;
      public static final int RIGHT_ENCODER_A_PORT = 2;
      public static final int RIGHT_ENCODER_B_PORT = 3;
      public static final double DISTANCE_PER_ENCODER_PULSE = (double) (6.0 * Math.PI / 2048.0);
	  public static final double DRIVE_MAX_VELOCITY = 12.2;
	  public static final double DRIVE_MAX_ACCELERATION = 8.8;
      public static final double PID_DRIVE_DISTANCE_TOLERANCE = 0.2;

      public static final int LEFT_MOTOR_A_PORT = 23;
      public static final int LEFT_MOTOR_B_PORT = 24;
      public static final int LEFT_MOTOR_C_PORT = 25;
      
      public static final int RIGHT_MOTOR_A_PORT = 10;
      public static final int RIGHT_MOTOR_B_PORT = 11;
      public static final int RIGHT_MOTOR_C_PORT = 12;
      
      public static final double TURN_DAMPENING_FACTOR = 0.75;

      // TODO update these once we have a robot to characterize
      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.596535912);
      public static final double ksVolts = 0.149;
      public static final double kvVoltSecondsPerMeter = 3.22;
      public static final double kaVoltSecondsSquaredPerMeter = 0.168;
	  public static final double kPDriveVel = 6.38; //1.67; //3.34;// 6.38; //2.7; //6.38 3.79;
	  public static final double kMaxSpeedMetersPerSecond = 12.2 * 0.3048;
	  public static final double kMaxAccelerationMetersPerSecond = 8.8 * 0.3048;

      public static final int POPPER_PORT = 0;

      public static final double ANGLE_TOLERANCE = 2.0;

      public static final double DEGREE_P = 0.0475;
      public static final double DEGREE_I = 0.0009;
      public static final double DEGREE_D = 0.004;
	  public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
	  public static final double ALPHA_FILTER = 1.0;     

	  public static final double MAX_TURN_RATE_DEG_PER_S = 100;
      public static final double MAX_TURN_ACCELERATION_DEG_PER_S_SQUARED = 300;

      public static final double TURN_TOLERANCE_DEG = 5;
	  public static final double TURN_RATE_TOLERANCE_DEG_PER_S = 10; // degrees per second
	  
	  public static final double PROFILED_TURN_P = 0.0175;
	  public static final double PROFILED_TURN_I = 0.0;
	  public static final double PROFILED_TURN_D = 0.0009;
	}
	
	public static class FloorIntake {
		public static final int FLOOR_INTAKE_MOTOR_PORT = 16; 
		public static final double RUN_SPEED = 0.5;
		public static final double REVERSE_SPEED = -1.0 * 0.5;
		public static final int SOLENOID_PORT = 1;
		public static final int FLOOR_INTAKE_MOTOR_CURRENT_LIMIT = 30;
		public static final int FLOOR_INTAKE_MOTOR_PORT_TWO = 22;
	}

	public static class LEDs {
		public static final int SHOOTER_LED_PORT = 0;
		public static final int SHOOTER_LED_BUFFER = 16;
		public static final List<Integer> OFF_RGB = List.of(0, 0, 0);
		public static final List<Integer> BLUE_RGB = List.of(0, 0, 255);
		public static final List<Integer> WHITE_RGB = List.of(255, 255, 255);
		public static final List<Integer> RED_RGB = List.of(255, 0, 0);
		public static final List<Integer> GREEN_RGB = List.of(0, 255, 0);
		public static final List<Integer> YELLOW_RGB = List.of(255, 255, 0);
		public static final List<Integer> MAGENTA_RGB = List.of(255, 0, 255);
		public static final List<Integer> CYAN_RGB = List.of(0, 255, 255);

		public static final int DIAGNOSTIC_LED_PORT = 1;
		public static final int DIAGNOSTIC_LED_BUFFER = 9;
	}

	public static class Bindings {
		public static final int XBOX_CONTROLLER_PORT = 0;

		public final static int A_BUTTON = 1, 
								B_BUTTON = 2, 
								X_BUTTON = 3, 
								Y_BUTTON = 4, 
								LEFT_BUMPER = 5,
								RIGHT_BUMPER = 6, 
								BACK_BUTTON = 7, 
								START_BUTTON = 8, 
								LEFT_STICK = 9, 
								RIGHT_STICK = 10;

		public static final int DRIVER_CONTROLLER_PORT = 0;
	}

 
  public static class XboxController {
	public static final int DRIVER_PORT = 0;
	public static final int OPERATOR_PORT = 1;
    public static final double DEAD_BAND = 0.05; //0.2

    public static final int B_BUTTON = 2;
    public static final int LEFT_TRIGGER = 2;
    public static final int RIGHT_TRIGGER = 3;
	public static final double HANGER_DEAD_BAND = 0.1; 
  }

	public static class Tower {
		public static final int TOWER_MOTOR_PORT = 17;
		public static final double RUN_TOWER_MOTOR_SPEED = 1.0;
		public static final double TOWER_LOAD_SPEED = 0.5;

		public static final int BEAM_BREAK_ONE_PORT = 4;
		public static final int BEAM_BREAK_TWO_PORT = 5;
		public static final int BEAM_BREAK_THREE_PORT = 6;
		public static final int BEAM_BREAK_FOUR_PORT = 7;
	}

	public static class Auto {
		public static final double kRamseteB = 2.0;
		public static final double kRamseteZeta = 0.7;
	}
	public static class Sorter {
		public static final int SORTER_MOTOR_PORT = 20;
		public static final double SORTER_FORWARDS_SPEED = 1.0;
		public static final double SORTER_BACKWARDS_SPEED = -1.0;
		public static final int SORTER_CURRENT_LIMIT = 30;
	}
	public static class ControlWheel {
		public static final int CONTROL_WHEEL_MOTOR = 19;
		public static final Color BLUE_TARGET = ColorMatch.makeColor(0.143, 0.427, 0.429);
		public static final Color GREEN_TARGET = ColorMatch.makeColor(0.197, 0.561, 0.240);
		public static final Color RED_TARGET = ColorMatch.makeColor(0.401, 0.232, 0.214);
		public static final Color YELLOW_TARGET = ColorMatch.makeColor(0.361, 0.624, 0.063);
		public static final int COMMAND_PANEL_CURRENT_LIMIT = 30;
		public static final double DEFAULT_CONTROL_WHEEL_SPEED = 0.5;
		public static final int CONTROL_WHEEL_SOLENOID_PORT = 2;
		public static final int LEFT_LIMIT_SWITCH_PORT = 10;
		public static final int RIGH_LIMIT_SWITCH_PORT = 11;
		public static final double CONTROL_WHEEL_CONVERSION_FACTOR = 12.57 / 49.0;
		public static final double ALIGN_TO_CONTROL_WHEEL_SPEED = 0.3;
	}
	
	public static class Vision {
		public static final double VISION_KP = 0.125;
		public static final double VISION_KI = 0.0;
		public static final double VISION_KD = 0.4;
	}
	
  	// System wide timing setting
	public static final int TIMEOUT_MS = 30;
}
