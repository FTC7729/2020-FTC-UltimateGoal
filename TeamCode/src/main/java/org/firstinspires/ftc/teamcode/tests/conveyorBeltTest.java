package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "conveyorBeltTest ", group = "B")
//@Disabled
public class conveyorBeltTest extends JerseyGirl_TeleDriveHandler_Test_Conveyor {


    public final double INCREMENT = 0.02;



    public void handleGamepad1(Gamepad gamepad) {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further,

        final boolean isButtonX = gamepad.x;
        final boolean isButtonY = gamepad.y;
        //telemetry.addData("pad1", "left:%.2f, right:%.2f, dir:%s", leftStickY, rightStickY, drivingDirection.name());


        // switch driving directions
        if (isButtonX) {
            double position = conveyorTest.getPosition() + INCREMENT;
            conveyorTest.setPosition(position);
        } else if (isButtonY) {
            double position = conveyorTest.getPosition() - INCREMENT;
            conveyorTest.setPosition(position);
        }

        // if either the DPAD left/right buttons are depressed, then we are strafing and setting the
        // wheel power to move left or right


        // the moment we take our finger off the DPAD, we are using the left and right stick values
        // to determine the power to apply to wheels.


    }
}
