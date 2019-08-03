package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="FGC_Jamaica_Code", group="Jamaica-Fgc")
public class Jamaica_FGC_2018 extends LinearOpMode {

    // Declare Motors and sensors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftdrive2 = null;
    private DcMotor rightdrive2 = null;
    private DigitalChannel limit = null;
    private DigitalChannel digitalTouch;
    private DistanceSensor distance=null;
    private DistanceSensor cdistance=null;
    private ColorSensor colour = null;
    private DcMotor lift_motor = null;
    private DcMotor lift_motor2 = null;

    private DcMotor wheels = null;
    private DcMotor wheels2 = null;
    private CRServo s2 = null;
    private CRServo windmill=null;
    private CRServo s1 = null;
    private Servo solar_lift = null;


    //Declare global variables
    private int mid, lift_motor_pos,speedmax;
    private boolean fly, fall,turn_off;
    private boolean rightin, leftin, reverse, grab_cube;
    private boolean intake_valid,out,intake_reset, or,or2;
    private  boolean run_power,run_power2,run_power3,run_power4;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

//       Initialize the hardware variables. and link it the code to the sensors and motors
        leftDrive  = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftdrive2  = hardwareMap.get(DcMotor.class, "leftdrive2");
        rightdrive2 = hardwareMap.get(DcMotor.class, "rightdrive2");
        limit = hardwareMap.get(DigitalChannel.class, "limit");
        lift_motor= hardwareMap.get(DcMotor.class,"L1");
        lift_motor2= hardwareMap.get(DcMotor.class,"L2");
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        solar_lift = hardwareMap.get(Servo.class, "solar_lift");
////        digitalTouch = hardwareMap.get(DigitalChannel.class, "skin");
        distance= hardwareMap.get(DistanceSensor.class,"sensor range");
//        colour = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        // get a reference to the distance sensor that shares the same name.
//        cdistance= hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        wheels=hardwareMap.get(DcMotor.class, "wheels");
        wheels2=hardwareMap.get(DcMotor.class, "wheels2");
        windmill=hardwareMap.get(CRServo.class, "windmill");

//
////        // Motors and sensors prior configurations
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        leftdrive2.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        rightdrive2.setDirection(DcMotor.Direction.REVERSE);
        lift_motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        wheels2.setDirection(DcMotorSimple.Direction.REVERSE);
        limit.setMode(DigitalChannel.Mode.INPUT);
//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        //Variable assigning
        mid = lift_motor.getCurrentPosition() - 300;
        intake_valid = true;
        intake_reset = false;
        speedmax=1;

//        // wait fot the user to press start button then run below
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status ", "Run Time: " + runtime.toString());
            telemetry.update();
            //functions for the robot

            driving(); //driving configuration

            intake(); //intake function

            box_Conveyor_Function(); //function for the box conveyor belt

            lift_Function(); //lift configuration
//
            windmill_Turn(); //windmill configuration
            //distance_sensor();
            solar_panel();//solar panel configurations



        }
    }

    private void driving(){
        //assign the values to the motor power variables
        double leftPower = 0;
        double rightPower = 0;
        run_power=gamepad1.dpad_down;
        run_power2=gamepad1.dpad_up;
        run_power3=gamepad1.dpad_left;
        run_power4=gamepad1.dpad_right;

        if(run_power2){
            speedmax=3;
        }
        if(run_power3){
            speedmax = 1;
        }

        if(run_power){
            speedmax=2;
        }
        if(run_power4){
            speedmax=4;
        }
        switch (speedmax){
            case 1:
                leftPower  = gamepad1.left_stick_y *0.5;
                rightPower = gamepad1.right_stick_y * 0.5;
                telemetry.addData("drive speed : ", "50%");
                break;
            case 2:
                leftPower  = gamepad1.left_stick_y *0.25;
                rightPower = gamepad1.right_stick_y * 0.25;
                telemetry.addData("drive speed : ", "25%");
                break;
            case 3:
                leftPower  = gamepad1.left_stick_y ;
                rightPower = gamepad1.right_stick_y ;
                telemetry.addData("drive speed : ", "billz");
                break;
            case 4:
                leftPower  = gamepad1.left_stick_y *0.75;
                rightPower = gamepad1.right_stick_y * 0.75;
                telemetry.addData("drive speed : ", "75%");
                break;


        }
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
        leftdrive2.setPower(leftPower);
        rightdrive2.setPower(rightPower);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

    }

    private void windmill_Turn() {
        //set gamepad values to variables
        boolean millpress = gamepad1.left_bumper;
        boolean spin = gamepad1.right_bumper;
        boolean obst_detect;
        int prevspd = 0;
        //if the robot is close to the windmill or an obstacle slow it down
        if(distance.getDistance(DistanceUnit.CM) <= 3){
            boolean obst_detect = true;
            int prevspd = speedmax;
            speedmax = 2;
        }
        if(obst_detect){
            speedmax = prevspd;
        }

        //If the windmill button is pressed spin for three rotations
        if (millpress) {
            windmill.setPower(1);
            telemetry.addData("Windmill: ", "ON");
            sleep(4500);
            windmill.setPower(0);
            telemetry.addData("Windmill: ", "OFF");
        }
        //if spin button is pressed, the windmill spin until stopped
        if(spin){
            windmill.setPower(1);
            telemetry.addData("Windmill: ", "ON");
        }else{
            windmill.setPower(0);
            telemetry.addData("Windmill: ", "OFF");
        }
    }

    private void lift_Function() {
        //Variables and receiving button calls
        fly = gamepad2.dpad_up;
        fall = gamepad2.dpad_down;
        lift_motor_pos = lift_motor.getCurrentPosition();


        if (fall) {
            if (limit.getState() || lift_motor_pos <= mid) {
                // switch not reach
                lift_motor.setPower(1);
                lift_motor2.setPower(1);
                intake_valid = false;

            }else if (!limit.getState() && lift_motor_pos > mid){
                //switch reached
                lift_halt();
                intake_valid = true;
            }
        }else if(!fall){
            lift_halt();
        }
        //If dpad up is pressed then allow the lift to rise until the switch is reached
        if (fly) {
            if (limit.getState() || lift_motor_pos >= mid) {
                // switch not reach
                lift_motor.setPower(-1);
                lift_motor2.setPower(-1);
                intake_valid = false;

            }else if (!limit.getState() && lift_motor_pos < mid) {
                //switch reached
                lift_halt();
            }
        }else if(!fly){
            lift_halt();
        }


        //Show the position of the
        telemetry.addData("Lift Position: ", "%d |Midpoint: %d , %b", lift_motor_pos, mid,limit.getState());
    }

    private void box_Conveyor_Function() {
        //assigning gamepad values to variables
        rightin = gamepad2.x;
        leftin = gamepad2.y;
        reverse = gamepad2.left_stick_button;

        //Box conveyor turned on
        if (rightin) {
            s1.setPower(-1);
            s2.setPower(-1);
            telemetry.addData("Box Conveyor: ", "ON");
        }
        //Stop the box conveyor
        if (leftin) {
            s1.setPower(0);
            s2.setPower(0);
            telemetry.addData("Box Conveyor: ", "OFF");
        }
        //reverse the box conveyor
        if (reverse) {
            s1.setPower(1);
            s2.setPower(1);
            telemetry.addData("Box Conveyor: ", "REVERSE");
        }
    }

    private void intake(){
        // assigning gamepad controls to variable and variable values
        grab_cube = gamepad2.right_stick_button;
        turn_off= gamepad2.dpad_right;
        boolean out= gamepad2.dpad_left;
        or=gamepad2.left_bumper;
        or2 = gamepad2.right_bumper;

        if(or){
            intake_reset = true;
            telemetry.addData("Box intake: ", "OVERRIDE ON");
        }
        if(or2){
            intake_reset = false;
            telemetry.addData("Box intake: ", "OVERRIDE OFF");
        }

        if(intake_valid||intake_reset){
            // the lift is in position
            telemetry.addData("Box intake: ", "ENABLED");

            if(grab_cube){ // turn on the intake
                wheels.setPower(0.8);
                wheels2.setPower(0.8);
                telemetry.addData("Box Intake: ", "ON");
            }else if(turn_off){ // turn off the intake
                wheels.setPower(0);
                wheels2.setPower(0);
                telemetry.addData("Box Intake: ", "OFF");
            }
            if(out){
                wheels.setPower(-0.8);
                wheels2.setPower(-0.8);
            }

        }else if(!intake_valid){
            wheels.setPower(0);
            wheels2.setPower(0);
            telemetry.addData("Box intake: ", "DISABLED");
        }
    }

    private void solar_panel(){
        if(gamepad1.x){
            solar_lift.setPosition(0);
        }
        if(gamepad1.y){
            solar_lift.setPosition(1);
        }
    }

    private void lift_halt(){
        lift_motor.setPower(0);
        lift_motor2.setPower(0);
//        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift_motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

//    private void distance_sensor(){
//
//        telemetry.addData("range2", String.format("%.01f cm", cdistance.getDistance(DistanceUnit.CM)));
//        telemetry.addData("range", String.format("%.01f cm", distance.getDistance(DistanceUnit.CM)));
//        telemetry.addData("Alpha", colour.alpha());
//        telemetry.addData("Red  ", colour.red());
//        telemetry.addData("Green", sensorColor.green());
//        telemetry.addData("Blue ", sensorColor.blue());
//            if (distance.getDistance(DistanceUnit.CM)<= 10 ){
//
//                telemetry.addData("range test: "," You have reached!");
//
//
//            }
//        if (cdistance.getDistance(DistanceUnit.CM)<= 10 ){
//
//
//            telemetry.addData("range test2: "," I am better :P!");
//
//
//        }
//        }

}