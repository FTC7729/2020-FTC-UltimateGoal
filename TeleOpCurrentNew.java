package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.Range;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.DigitalChannel;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.util.Range;
        import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name ="TeleOpCurrentNew", group = "TeleOP")
public class TeleOpCurrentNew extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
    DcMotor raiseArm1;
    DcMotor raiseArm2;
    DcMotor extendArm;
    CRServo claw1;
    CRServo claw2;
    CRServo wrist;
    boolean powerControl = false;
    double powerGiven =0;
    boolean clamp = false;
    int powerButton;
    CRServo drag1, drag2;

    double armPowerMultiplier = 0.5;


    public void init() {
        //hardware map is for phone

        //    touchSense = hardwareMap.get(DigitalChannel.class, "sensor_digital");
        frontRight = hardwareMap.dcMotor.get("front right");
        frontLeft = hardwareMap.dcMotor.get("front left");
        backRight = hardwareMap.dcMotor.get("back right");
        backLeft = hardwareMap.dcMotor.get("back left");
     //   raiseArm1 = hardwareMap.dcMotor.get("raise arm 1");
       // raiseArm2 = hardwareMap.dcMotor.get("raise arm 2");
       // extendArm = hardwareMap.dcMotor.get("extend arm");
        // claw1 = hardwareMap.crservo.get("claw 1");
        //claw2 = hardwareMap.crservo.get("claw 2");
       // drag1 = hardwareMap.crservo.get("drag front");
     //   drag2 = hardwareMap.crservo.get("drag back");
     //   wrist = hardwareMap.crservo.get("wrist");
    }

    private void setRaiseArmPower(float armPower, double multiplier){
        raiseArm1.setPower(armPower*multiplier);
        raiseArm2.setPower(armPower*multiplier);
        return;
    }

    public void loop() {
        //              -----STICK VARIABLES-----
        //For driving
        float move = -gamepad1.left_stick_y;
        float crabWalk = gamepad1.left_stick_x;
        float rotation = -gamepad1.right_stick_x;

        //For arm raising
        float rawRaiseValue = -gamepad2.left_stick_y;




        //              -----WHEEL LOGIC-----
        //Wheels: Holonomic drive formula uses values of gamestick position to move
        double fLeftPower = Range.clip(move + rotation + crabWalk, -1.0, 1.0);
        double bLeftPower = Range.clip(move + rotation - crabWalk, -1.0, 1.0);
        double fRightPower = Range.clip(move - rotation - crabWalk, -1.0, 1.0);
        double bRightPower = Range.clip(move - rotation + crabWalk, -1.0, 1.0);
        //Assignment of motor power in relation to wheels
        frontLeft.setPower(fLeftPower/powerButton);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft.setPower(bLeftPower/powerButton);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setPower(fRightPower/powerButton);

        backRight.setPower(bRightPower/powerButton);

        //raiseArm1.setDirection(DcMotorSimple.Direction.FORWARD);
        //raiseArm2.setDirection(DcMotorSimple.Direction.REVERSE);



        //          -----GAME PAD 1-----

        //              ###SPEED BOOST###
        if(gamepad1.right_trigger>0.1){
            powerButton=1;
        }else{
            powerButton =2;
        }

        //              ###DRAG SERVOS###
      /* if(gamepad1.a){
            drag1.setPower(.5);
            drag2.setPower(-.5);
        } */
       /* else if(gamepad1.b){
            drag1.setPower(-.5);
            drag2.setPower(.5);
        } */
       /* else{
            drag1.setPower(0);
            drag2.setPower(0);
        } */

        /* if(gamepad1.x){
            drag2.setPower(.5);
        }
        else if(gamepad1.y){
            drag2.setPower(-.5);
        }
        else{
            drag2.setPower(0);
        }*/




        //          -----GAME PAD 2-----

        //              ###CLAMPS###
       /* if (gamepad2.x){
            clamp = true;
        } */
      /*  if (gamepad2.y){
            claw1.setPower(1);
            claw2.setPower(-1);
            clamp = false;
        } */
       /* else if (!clamp){
            claw1.setPower(0);
            claw2.setPower(0);
        } */
       /* if (clamp){
            claw1.setPower(-1);
            claw2.setPower(1);
        } */



        //claw1: 1=open, 0=closed
        //claw2: 0=open, 1=closed

        //open
     /*   if (gamepad2.y){
            claw1.setPosition(0.6);
            claw2.setPosition(0.4);
        }
        //close
       else if (gamepad2.x){
            claw1.setPosition(0.4);
            claw2.setPosition(0.6);
        } */



        //              ###ARM EXTENSION###

      //  extendArm.setPower(-gamepad2.right_stick_y);

        //              ###WRIST###

     /*   if (gamepad2.right_bumper){
            wrist.setPower(0.5);
        } */
     /*   else if (gamepad2.left_bumper){
            wrist.setPower(-0.5);
        } */
      /*  else {
            wrist.setPower(0);
        } */


        //              ###ARM RAISING###

        // Fast raise arm mode
       /* if (gamepad2.right_trigger>0){
            //If the driver is trying to move the arm up:
            if (rawRaiseValue > 0) {
                setRaiseArmPower(rawRaiseValue, 0.6);
            }
            //If the driver is trying to move the arm down:
            else if (rawRaiseValue < 0) {
                setRaiseArmPower(0.1f, 0.35);
            }
            //If the driver is not moving the arm
            else {
                setRaiseArmPower(0.23f, 1);
            }
        }*/
        // Slow raise arm mode
       /* else {
            //If the driver is trying to move the arm up:
            if (rawRaiseValue > 0) {
                setRaiseArmPower(rawRaiseValue, 0.35);
            }
            //If the driver is trying to move the arm down:
            else if (rawRaiseValue < 0) {
                setRaiseArmPower(0f, 1);
            }
            //If the driver is not moving the arm
            else {
                setRaiseArmPower(0.23f, 1);
            }
        }*/
        /*
        // Fast raise arm mode
        if (gamepad2.right_trigger>0){
            //If the driver is trying to move the arm up:
            if (rawRaiseValue > 0) {
                setRaiseArmPower(rawRaiseValue, 0.6);
            }
            //If the driver is trying to move the arm down:
            else if (rawRaiseValue < 0) {
                setRaiseArmPower(0.1f, 0.35);
            }
            //If the driver is not moving the arm
            /*else {
                setRaiseArmPower(0.23f, 1);
            }
        }
        // Slow raise arm mode
     /*   else {
            //If the driver is trying to move the arm up:
            if (rawRaiseValue > 0) {
                setRaiseArmPower(rawRaiseValue, 0.35);
            }
            //If the driver is trying to move the arm down:
            else if (rawRaiseValue < 0) {
                setRaiseArmPower(0f, 1);
            }
            //If the driver is not moving the arm
           /* else {
                setRaiseArmPower(0.23f, 1);
            }
        }*/




    }




}
