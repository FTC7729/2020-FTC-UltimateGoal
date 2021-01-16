package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "intakeTest_TeleOp ", group = "B")
//@Disabled
public class intaketTest_TeleOp extends JerseyGirl_TeleDriveHandler_Test_Intake {
    public static final double DRIVE_SPEED = 1;
    public static final double COUNTS_PER_MOTOR_REV = 288;    // eg: Rev Core Hex Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    public void handleGamepad1(Gamepad gamepad) {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further,

        final boolean isButtonX = gamepad.x;
        final boolean isButtonY = gamepad.y;
        //telemetry.addData("pad1", "left:%.2f, right:%.2f, dir:%s", leftStickY, rightStickY, drivingDirection.name());


        // switch driving directions
        if (isButtonX) {
         intakeTest.setPower(DRIVE_SPEED);
        } else if (isButtonY) {
            intakeTest.setPower(-DRIVE_SPEED);
        }
        else {
            intakeTest.setPower(0);
        }


        // if either the DPAD left/right buttons are depressed, then we are strafing and setting the
        // wheel power to move left or right


        // the moment we take our finger off the DPAD, we are using the left and right stick values
        // to determine the power to apply to wheels.


    }
}
