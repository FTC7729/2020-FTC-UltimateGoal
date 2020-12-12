package org.firstinspires.ftc.teamcode.TeleDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Jersey TeleDrive ", group = "B")
//@Disabled
public class JerseyGirl_TeleDrive extends JerseyGirl_TeleDriveHandler {

    double speedScale = 0.25;
    public void goForward(double power) {
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    public void goBackward(double power){
        goForward(-power);
    }
    public void strafeLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    public void strafeRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }
    public void handleGamepad1(Gamepad gamepad){
       double rStickX;
       double rStickY;
       double lStickX;
       double targetAngle;
       double mag1;
       double mag2;
       double rotationPower;
       double maxPower;
       double scaleDown;
       boolean dpadLeft;
       boolean dpadRight;
       boolean dpadUp;
       boolean dpadDown;


        rStickX = gamepad.right_stick_x * speedScale;
        rStickY = -gamepad.right_stick_y * speedScale;
        lStickX = gamepad.left_stick_x * speedScale;
        dpadLeft = gamepad.dpad_left;
        dpadRight = gamepad.dpad_right;
        dpadUp = gamepad.dpad_up;
        dpadDown = gamepad.dpad_down;

        targetAngle = (Math.atan2(rStickY, rStickX));
        if(dpadLeft){
            strafeLeft(0.5);
        } else if(dpadRight){
            strafeRight(0.5);
        } else if(dpadUp){
            goForward(0.5);
        } else if(dpadDown) {
            goBackward(0.5);
        }
        rotationPower = -lStickX;
        mag1 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(targetAngle + Math.PI / 4));
        mag2 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(targetAngle - Math.PI / 4));

        maxPower = Math.max(Math.abs(mag1) + Math.abs(rotationPower), Math.abs(mag2) + Math.abs(rotationPower)) + 0.15;
        scaleDown = 1.0;

        if (maxPower > 1)
            scaleDown = 1.0 / maxPower;

        leftFront.setPower((mag2 - rotationPower) * scaleDown);
        leftBack.setPower((mag1 - rotationPower) * scaleDown);
        rightBack.setPower((mag2 + rotationPower) * scaleDown);
        rightFront.setPower((mag1 + rotationPower) * scaleDown);

    }

}
