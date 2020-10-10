package org.firstinspires.ftc.teamcode.TeleDrive; //set your package

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMap.HardwareMap_Example;
import org.firstinspires.ftc.teamcode.TeleDrive.TeleDrive;

@TeleOp(name = "Your TeleOp Name", group = "TeleOpMode")
public class SampleTeleDriveOpMode extends TeleDrive {
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    /* CHAWKS: Call and declare the robot here */
    HardwareMap_Example robot   = new HardwareMap_Example();    // Use the Example hardware map

    @Override
    public void init() {
        super.init();
        //Init code here
                /*
            CHAWKS: Step 0. Initialize OUR ROBOT
        */
        // MUST HAVE THIS LINE BELOW
        robot.init(hardwareMap);
    }

    public void init_loop() {
        super.init_loop();
        //Init Loop code here
    }

    @Override
    public void start() {
        super.start();
        //Start code here
        runtime.reset();
    }

    @Override
    public void loop() {
        super.loop();
        //Loop code here

        // Below are VARIABLES that must be outside the "while (opModeIsActive())" loop
        double left;
        double right;
        double drive;
        double turn;
        double max;
        drive = -gamepad1.left_stick_y;
        turn  =  gamepad1.right_stick_x;

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
        robot.leftBack.setPower(left);
        robot.rightBack.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        //if (gamepad1.right_bumper)
        //clawOffset += CLAW_SPEED;
        //else if (gamepad1.left_bumper)
        //clawOffset -= CLAW_SPEED;

    }

    @Override
    public void stop() {
        super.stop();
        //Stop code here
    }
}