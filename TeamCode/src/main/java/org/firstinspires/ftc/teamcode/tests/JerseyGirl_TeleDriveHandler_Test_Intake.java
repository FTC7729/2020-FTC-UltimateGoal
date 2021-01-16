package org.firstinspires.ftc.teamcode.tests; //set your package

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "JerseyGirl_TeleDriveHandler_Test_Intake", group = "TeleOpMode")
@Disabled
public class JerseyGirl_TeleDriveHandler_Test_Intake extends TeleDrive {


    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public DcMotor intakeTest   = null;

    HardwareMap hwMap = null;

    // private HardwareMap ahwMap;

    @Override
    public void init() {
        super.init();
     

        intakeTest  = hardwareMap.dcMotor.get("intakeTest");
       

     

        intakeTest.setDirection(DcMotor.Direction.REVERSE);
      

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