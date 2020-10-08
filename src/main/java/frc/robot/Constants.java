/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public interface Constants {

    Path DEPLOY_DIR = Filesystem.getDeployDirectory().toPath(); 

    public interface Drivetrain {
        int LEFT_TOP = 7;
        int LEFT_BOTTOM = 6;

        int RIGHT_TOP = 4;
        int RIGHT_BOTTOM = 3;

        int LEFT_ENCODER_A = 0;
        int LEFT_ENCODER_B = 1;
        int RIGHT_ENCODER_A = 2;
        int RIGHT_ENCODER_B = 3;

        double WHEEL_DIAMETER = 0.5; 
        double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; 
        double ENCODER_FACTOR = WHEEL_CIRCUMFERENCE * (1.0 / 16.71);

        // 30 in by 29 in
        double TRACK_WIDTH = 29 / 12;

        public interface FF {
            double ks = 0;
            double kv = 0;
            double ka = 0;
        }

        public interface PID {
            double kp = 0;  
            double ki = 0;
            double kd = 0;
        }
    }
}
