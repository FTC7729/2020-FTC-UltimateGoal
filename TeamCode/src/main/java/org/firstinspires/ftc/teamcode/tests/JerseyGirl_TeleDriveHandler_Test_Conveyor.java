package org.firstinspires.ftc.teamcode.tests; //set your package

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "JerseyGirl_TeleDriveHandler_Test_Conveyor", group = "TeleOpMode")
@Disabled
public class JerseyGirl_TeleDriveHandler_Test_Conveyor extends TeleDrive {


    public Servo conveyorTest  = null;

    // private HardwareMap ahwMap;

    @Override
    public void init() {
        super.init();
        //Init code here
        /* local OpMode members. */
        //HardwareMap hwMap           =  null;


        // Define and Initialize Motors
        /*
            CHAWKS: The deviceName should ALWAYS ALWAYS ALWAYS
                    match the part name to avoid confusion

*/

          conveyorTest = hardwareMap.servo.get("conveyorTest");

        // Set Direction/Motion for Motors
        /*
            CHAWKS: Why are we reversing the Right Wheels?

*/
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
        handleGamepad1(gamepad1);
        handleGamepad2(gamepad2);
    }

    @Override
    public void stop() {
        super.stop();
        //Stop code here
    }

    public void handleGamepad1(Gamepad gamepad) {

    }

    public void handleGamepad2(Gamepad gamepad) {

    }
}