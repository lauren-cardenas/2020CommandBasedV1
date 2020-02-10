/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        
        public static final int kLeftMotor1Port = 0;
        public static final int kLeftMotor2Port = 1;
        public static final int kRightMotor1Port = 2;
        public static final int kRightMotor2Port = 3;

        public static final int[] kLeftEncoderPorts = new int[]{0,1};
        public static final int[] kRightEncoderPorts = new int[]{2,3};
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;

        public static final int kEncoderCPR = 1024;
        public static final double kWheelDiameterInches = 6;
        public static final double kEncoderDistancePerPulse =
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;
    }

    public static final class ArmConstants{
        public static final int kMotorPort = 4;
        public static final int kRollerPort = 5;

        public static final int kDown = 0;
        public static final int kUp = 1;
    }

    public static final class LimeLightConstants{
        public static final double kP = -0.05;
        public static final double kMinCommand = 0.01;
    }

    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double DRIVING_SPEED = 0.85;
        public static final double TURNING_RATE = 0.7;

        public static final double ARM_SPEED = 0.5;
        public static final double ROLLER_SPEED = 0.5;

        public static final double SHOOTER_SPEED = 1;
    }
}
