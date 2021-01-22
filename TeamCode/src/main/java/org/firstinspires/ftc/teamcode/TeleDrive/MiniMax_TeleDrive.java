package org.firstinspires.ftc.teamcode.TeleDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
@TeleOp(name = "MiniMax TeleDrive ", group = "A")
@Disabled
public class MiniMax_TeleDrive extends MiniMax_TeleDriveHandler {
    // Below are VARIABLES that must be outside the "while (opModeIsActive())" loop
        double left;
        double right;
        double drive;
        double turn;
        double max;

    public void handleGamepad1(Gamepad gamepad){


        drive = -gamepad.left_stick_y;
        turn  =  gamepad.right_stick_x;

        // Combine drive and turn for blended motion.
        left  = drive + turn;
        right = drive - turn;

        // Normalize the values so neither exceed +/- 1.0
        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        // Output the safe vales to the motor drives.
            /*
                CHAWKS: Put in the power number!
                        Which motors are we missing?
                        What would happen if you apply power to half the wheels?
             */
        leftBack.setPower(left);
        rightBack.setPower(right);


        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.update();


            /*
                CHAWKS: We are the end! What happens now?
            */

    }

}
