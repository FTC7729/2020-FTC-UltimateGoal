package org.firstinspires.ftc.teamcode.TeleDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
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

    public void goBackward(double power) {
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
        double rStickX;
        double rStickY;
        double lStickX = 0;
        double targetAngle;
        double mag1;
        double mag2;
        double rotationPower;
        double maxPower;
        double scaleDown;
        double currentAngle;
        double totalAngle = 0;
        Orientation angles;
        boolean lBumper = false;
        boolean rBumper = false;
        boolean dpadLeft;
        boolean dpadRight;
        boolean dpadUp;
        boolean dpadDown;
        boolean aPress;

        int numOfIncrements = 0;
        //we have to increment later, so we save how often we do it now
        while (timeSinceLastIncrement > timeIncrement) {
            timeSinceLastIncrement -= timeIncrement;
            //changes the power of motors
            if (lStickX > prevLStick) {
                prevLStick += powIncrement;
            } else if (lStickX < prevLStick) {
                prevLStick -= powIncrement;
            }

            numOfIncrements++;
        }

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        currentAngle = (double) angles.firstAngle - subtractAngle;
        telemetry.addData("Heading: ", "%.3f", currentAngle);
        rStickX = gamepad.right_stick_x * speedScale;
        rStickY = -gamepad.right_stick_y * speedScale;
        lStickX = gamepad.left_stick_x * speedScale;
        dpadLeft = gamepad.dpad_left;
        dpadRight = gamepad.dpad_right;
        dpadUp = gamepad.dpad_up;
        dpadDown = gamepad.dpad_down;
        aPress = gamepad.a;
       
        if (lStickX > 0) {
            lStickX = Math.pow(lStickX, 1.6);
        }else{
            lStickX = -Math.pow(-lStickX, 1.6);
        }
        targetAngle = (Math.atan2(rStickY,rStickX));
        totalAngle = targetAngle + currentAngle;
        if (lBumper){ //slow drive
            speedScale = 0.25;
        }
        if (rBumper){ //normal drive
            speedScale = 1;
        }
        targetAngle = (Math.atan2(rStickY, rStickX));
        if (dpadLeft) {
            strafeLeft(0.5);
        } else if (dpadRight) {
            strafeRight(0.5);
        } else if (dpadUp) {
            goForward(0.5);
        } else if (dpadDown) {
            goBackward(0.5);
        } else {


            rotationPower = -lStickX * ROTATION_CONSTANT;
            mag1 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(totalAngle + Math.PI / 4));
            mag2 = Math.sqrt(Math.pow(rStickX, 2) + Math.pow(rStickY, 2)) * (Math.sin(totalAngle - Math.PI / 4));

            maxPower = Math.max(Math.abs(mag1) + Math.abs(rotationPower), Math.abs(mag2) + Math.abs(rotationPower));

            //adjusts mag1 and 2 so that it's smooth
            for (int i = 0; i < numOfIncrements; i++) {
                if (mag1 > prevMag1) {
                    prevMag1 += powIncrement;
                } else if (mag1 < prevMag1) {
                    prevMag1 -= powIncrement;
                }

                if (mag2 > prevMag2) {
                    prevMag2 += powIncrement;
                } else if (mag2 < prevMag2) {
                    prevMag2 -= powIncrement;
                }
            }

            if (Math.abs(prevMag1) < deadZone) {
                prevMag1 = 0;
            }
            if (Math.abs(prevMag2) < deadZone) {
                prevMag2 = 0;
            }

            mag1 = prevMag1;
            mag2 = prevMag2;

            telemetry.addData("mag1: ", "%.3f", mag1);
            telemetry.addData("mag2: ", "%.3f", mag2);
            telemetry.addData("rotationPower: ", "%.3f", rotationPower);

            if (maxPower > 1.0)
                scaleDown = 1.0 / maxPower;
            else
                scaleDown = 1.0;


            leftFront.setPower((mag2 - rotationPower) * scaleDown);
            leftBack.setPower((mag1 - rotationPower) * scaleDown);
            rightBack.setPower((mag2 + rotationPower) * scaleDown);
            rightFront.setPower((mag1 + rotationPower) * scaleDown);
        }
    }
}


