
package org.firstinspires.ftc.teamcode.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * CHAWKS:  Naming convention is camel case!
 * <p>
 * front
 * (LF)--------(RF)
 * |    robot   |
 * (LB)--------(RB)
 * back
 * <p>
 * Motor channel:  Left Front (LF) drive motor:        "leftFront"
 * Motor channel:  Right Front (RF) drive motor:        "rightFront"
 * Motor channel:  Left Back (LB) drive motor:        "leftBack"
 * Motor channel:  Right Back (RB) drive motor:        "rightBack"
 */
public abstract class jerseyGirlHardwareMap extends LinearOpMode {
    /* Public OpMode members. */
    // CHAWKS: The Robot Parts need to be established here
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    static final double THRESHOLD = 1.5;
    BNO055IMU imu;
    /////////////////////////////////////

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    /* CHAWKS: Call and declare the robot here */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront1;

    /* Constructor */
    public jerseyGirlHardwareMap() {

    }

    public static final double COUNTS_PER_MOTOR_REV = 288;    // eg: Rev Core Hex Motor Encoder
    public static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double WHEEL_DIAMETER_INCHES = 3.5;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double DRIVE_SPEED = 1;
    public static final double TURN_SPEED = 1;

    // Initialize standard Hardware interfaces
    /*
        CHAWKS: On Driver Station - [INIT] - Button //
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        // Do Not ever remove this.
        hwMap = ahwMap;

        // Define and Initialize Motors
        /*
            CHAWKS: The deviceName should ALWAYS ALWAYS ALWAYS
                    match the part name to avoid confusion
         */
        leftFront = hwMap.get(DcMotor.class, "leftFront");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        leftBack = hwMap.get(DcMotor.class, "leftBack");
        rightBack = hwMap.get(DcMotor.class, "rightBack");
//Threshold for Gyro turning so that we will not continuously attempt to reach an exact value


        // Set Direction/Motion for Motors
        /*
            CHAWKS: Why are we reversing the Right Wheels?
         */
        leftFront.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightBack.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to ZERO! power
        /*
            CHAWKS: Why do we set the power to zero?
         */
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        // Set all motors to run without encoders.
        /*
            CHAWKS: Encoder Exercise!
         */

        //leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //get and initialize IMU
        imu = hwMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    /*
     *  CHAWKS: It's a METHOD!!!
     *
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void gyroTurn(double power, double target) {
        Orientation angles;
        double error;
        double k = 6 / 360.0;
        double kInt = 3 / 3600.0;
        double eInt = 0;
        double prevTime = System.currentTimeMillis();
        double globalAngle = 0;
        double lastAngle = 0;
        double deltaAngle = 0;
        while (opModeIsActive()) {
            double currentTime = System.currentTimeMillis();
            double loopTime = (currentTime - prevTime) / 1000.0; // In seconds
            prevTime = currentTime;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            deltaAngle = angle - lastAngle;

            //adjusts the change in angle (deltaAngle) to be the actual change in angle
            if (deltaAngle < -180) {
                deltaAngle += 360;
            } else if (deltaAngle > 180) {
                deltaAngle -= 360;
            }
            globalAngle += deltaAngle;
            lastAngle = angle;

            error = target - globalAngle;
            eInt += loopTime * error;
            telemetry.addData("Heading", angles.firstAngle + " degrees");
            telemetry.addData("GlobalAngle", globalAngle + " degrees");
            telemetry.addData("Error", error + " degrees");
            telemetry.addData("Loop time: ", loopTime + " ms");
            telemetry.update();
            if (error == 0) {
                stopMotors();
                break;
            }
            turnLeft(k * error + kInt * eInt);
            idle();
        }
    }

    public void goForward(double power, int distance) {
        Orientation angles;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double error;
        double k = 3/360.0;
        int leftFrontTarget = leftFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);

        while (opModeIsActive() &&
                (power > 0 && leftFront.getCurrentPosition() < leftFrontTarget && rightFront.getCurrentPosition() < rightFrontTarget && leftBack.getCurrentPosition() < leftBackTarget && rightBack.getCurrentPosition() < rightBackTarget) ||
                (power < 0 && leftFront.getCurrentPosition() > leftFrontTarget && rightFront.getCurrentPosition() > rightFrontTarget && leftBack.getCurrentPosition() > leftBackTarget && rightBack.getCurrentPosition() > rightBackTarget)
        ) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            telemetry.addData("firstAngle",angles.firstAngle+" degrees");
            telemetry.addData("leftFront ",leftFront.getCurrentPosition());
            telemetry.addData("rightFront ",rightFront.getCurrentPosition());
            telemetry.addData("leftBack ",leftBack.getCurrentPosition());
            telemetry.addData("rightBack ",rightBack.getCurrentPosition());

            telemetry.update();
            leftFront.setPower((power - (error * k)));
            rightFront.setPower((power + (error * k)));
            leftBack.setPower((power - (error * k)));
            rightBack.setPower((power + (error * k)));
        }
        stopMotors();

    }
    
    public void goBackward(double power, int distance) {
        goForward(-power, -distance);
    }

    public void turnLeft(double power) {
        leftFront.setPower(-power);
        rightFront.setPower(power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
    }

    public void turnRight(double power) {
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(power);
        rightBack.setPower(-power);
    }

    public void strafeLeft(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3 / 360.0;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = leftFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                && (leftFront.getCurrentPosition() > leftFrontTarget && rightFront.getCurrentPosition() < rightFrontTarget && leftBack.getCurrentPosition() < leftBackTarget && rightBack.getCurrentPosition() > rightBackTarget)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            leftFront.setPower(-(power + (error * k)));
            rightFront.setPower((power + (error * k)));
            leftBack.setPower((power - (error * k)));
            rightBack.setPower(-(power - (error * k)));
            telemetry.addData("error: ", error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ", leftFront.getCurrentPosition());


            telemetry.update();

        }
        stopMotors();
    }

    public void strafeRight(double power, int distance) {
        Orientation angles;
        double error;
        double k = 3 / 360.0;
        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int leftFrontTarget = leftFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        int rightFrontTarget = rightFront.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int leftBackTarget = leftBack.getCurrentPosition() - (int) (distance * COUNTS_PER_INCH);
        int rightBackTarget = rightBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        leftFront.setTargetPosition(leftFrontTarget);
        rightFront.setTargetPosition(rightFrontTarget);
        leftBack.setTargetPosition(leftBackTarget);
        rightBack.setTargetPosition(rightBackTarget);

        while (opModeIsActive()
                && (leftFront.getCurrentPosition() < leftFrontTarget && rightFront.getCurrentPosition() > rightFrontTarget && leftBack.getCurrentPosition() > leftBackTarget && rightBack.getCurrentPosition() < rightBackTarget)) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //finds the angle given by the imu [-180, 180]
            double angle = angles.firstAngle;
            error = startAngle - angle;
            leftFront.setPower((power - (error * k)));
            rightFront.setPower(-(power - (error * k)));
            leftBack.setPower(-(power + (error * k)));
            rightBack.setPower((power + (error * k)));
            telemetry.addData("error: ", error);
            telemetry.addData("leftfront dest: ", leftFrontTarget);
            telemetry.addData("leftFront pos: ", leftFront.getCurrentPosition());


            telemetry.update();

        }
        stopMotors();
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        /*
            CHAWKS: opModeIsActive is a very awesome & important
                    Autonomous mode is 30 seconds...
         */
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            leftBack.setTargetPosition(newLeftTarget);
            rightBack.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftBack.setPower(Math.abs(speed));
            rightBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftBack.isBusy() && rightBack.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        leftBack.getCurrentPosition(),
                        rightBack.getCurrentPosition());
                telemetry.update();
            }


            // Stop all motion;
            leftBack.setPower(0);
            rightBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void stopMotors() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}

