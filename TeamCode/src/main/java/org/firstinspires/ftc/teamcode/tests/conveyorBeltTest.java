package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.TeleDrive.JerseyGirl_TeleDriveHandler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name = "conveyorBeltTest ", group = "B")
//@Disabled
public class conveyorBeltTest extends JerseyGirl_TeleDriveHandler {

    double speedScale = 0.25;

    public final int LIFT_MAX_POS = 6618;

    public final int LIFT_MIN_POS = 0;

    public final int LIFT_ON_FOUNDATION = 770;

    public final int CLAW_MAX_POS = 5000;

    public final int CLAW_MIN_POS = 0;

    public final double ROTATION_CONSTANT = 0.4;

    double subtractAngle = 0;

    double prevTime = 0;
    double startTime = System.currentTimeMillis();
    double prevLStick = 0;
    double prevMag1 = 0;
    double prevMag2 = 0;
    double timeSinceLastIncrement = 0;
    final double timeIncrement = 50;
    final double powIncrement = 0.2;
    final double deadZone = 0.05;

    public void handleGamepad1(Gamepad gamepad) {
        // TODO: would be nice to use exponential scaling of the Y value so that as you move stick further,
        float leftStickY = Range.clip(-gamepad.left_stick_y, -1, 1);
        float rightStickY = Range.clip(-gamepad.right_stick_y, -1, 1);
        final boolean isButtonX = gamepad.x;
        final boolean isButtonY = gamepad.y;
        //telemetry.addData("pad1", "left:%.2f, right:%.2f, dir:%s", leftStickY, rightStickY, drivingDirection.name());

        final boolean isDPADLeft = gamepad.dpad_left;
        final boolean isDPADRight = gamepad.dpad_right;
        final boolean isDPADUp = gamepad1.dpad_up;
        final boolean isDPADDown = gamepad1.dpad_down;

        // switch driving directions
        if (isButtonX) {

        } else if (isButtonY) {

        }

        // if either the DPAD left/right buttons are depressed, then we are strafing and setting the
        // wheel power to move left or right


        // the moment we take our finger off the DPAD, we are using the left and right stick values
        // to determine the power to apply to wheels.


    }
}
