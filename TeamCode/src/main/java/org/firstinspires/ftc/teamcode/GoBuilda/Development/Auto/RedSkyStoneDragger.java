package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
<<<<<<< Updated upstream
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
=======
import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;
import org.firstinspires.ftc.teamcode.Utils.GyroCode;
>>>>>>> Stashed changes

@Autonomous(name="Red SkyStoneDragger", group = "Red")
public class RedSkyStoneDragger extends LinearOpMode
{

<<<<<<< Updated upstream
    //initalizing sensors
    private ColorSensor leftColor, rightColor;
    private DistanceSensor leftDistance, rightDistance;
    //gyro init
    BNO055IMU imu;
    Orientation angles;
    char blockPos = ' ';

    //initializing motors
    private DcMotor MotorFrontY, MotorFrontX, MotorBackX, MotorBackY, motorRotate, motorExtend;
    Servo grasp1, grasp2, angle1, angle2, rightCollection, leftCollection, foundation;
=======
    char blockPos = ' ';

    GoBuildaUtil robot = new GoBuildaUtil();
    GyroCode gyro = new GyroCode();
>>>>>>> Stashed changes


    @Override
    public void runOpMode() throws InterruptedException
    {
<<<<<<< Updated upstream
        initializeMotors();
        initSensors();
=======
        robot.initializeAuto(hardwareMap);
        robot.block_drag_grasp.setPosition(0);
        robot.initializeSensors(hardwareMap);
        gyro.initGyro(hardwareMap);
>>>>>>> Stashed changes

        waitForStart();

        //First Block
        moveSetDistance();
        detectBlock();

<<<<<<< Updated upstream
        moveY(1,0.2);

        collectBlock("right","down");
        Thread.sleep(1500);

        moveY(-17, 0.2);
        moveX(-66, 0.2);

       collectBlock("right","up");
       moveX(69, 0.2);

        moveSetDistance();

        collectBlock("left","down");
        Thread.sleep(2000);

        moveY(-20, 0.2);
        Thread.sleep(500);
        moveX(-75, 0.5);

        collectBlock("left", "up");
        Thread.sleep(500);

        moveX(15, 0.2);
    }


    private void initializeMotors()
    {
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorFrontY = hardwareMap.dcMotor.get("fy");
        MotorBackY = hardwareMap.dcMotor.get("by");
        motorExtend = hardwareMap.dcMotor.get("extend");
        motorRotate = hardwareMap.dcMotor.get("rotate");

        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBackY.setDirection(DcMotorSimple.Direction.REVERSE);
        motorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRotate.setDirection(DcMotorSimple.Direction.REVERSE);


        MotorFrontX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        MotorFrontX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        grasp1 = hardwareMap.servo.get("grasp1");
        grasp2 = hardwareMap.servo.get("grasp2");
        rightCollection = hardwareMap.servo.get("rightCollection");
        leftCollection = hardwareMap.servo.get("leftCollection");
        foundation = hardwareMap.servo.get("foundation");
        angle1 = hardwareMap.servo.get("angle1");
        angle2 = hardwareMap.servo.get("angle2");

//        grasp1.setPosition(1);
//        grasp2.setPosition(0);

    }

    private void initSensors()
    {

        leftColor = hardwareMap.get(ColorSensor.class, "left");
        rightColor = hardwareMap.get(ColorSensor.class, "right");

        leftColor.enableLed(true);
        rightColor.enableLed(true);

        leftDistance = hardwareMap.get(DistanceSensor.class, "left");
        rightDistance = hardwareMap.get(DistanceSensor.class, "right");
    }
=======
        //Grab block
        dragBlock("rot","down");
        Thread.sleep(700);
        dragBlock("grasp", "chomp");
        Thread.sleep(700);
        dragBlock("rot", "up");

        robot.moveY(-8, 0.2,2);

        //moving towards the quarry
        if(blockPos == 'l') { moveX(-79, 0.5); }

        if(blockPos == 'c') { moveX(-70, 0.5); }

        if(blockPos == 'r') { moveX(-62, 0.5); }

        //moving towards the foundation
        moveY(17, 0.75);
        dragBlock("grasp", "unchomp");
        Thread.sleep(500);

        moveY(-13, 0.5);
        dragBlock("rot", "travel");

        //2nd block
        if(blockPos == 'l') { moveX(105, 0.4); }

        if(blockPos == 'c') { moveX(95, 0.4); }

        if(blockPos == 'r') { moveX(90, 0.4); }

        Thread.sleep(250);

        telemetry.addData("Entering Gyro", null);
        telemetry.update();

        robot.NOENC();
        gyro.rotationCorrection(0, 0.3, 1);
        robot.initializeAuto(hardwareMap);
        dragBlock("rot", "up");

        telemetry.addData("EXItinG Gyro", null);
        telemetry.update();

        moveSetDistance();
//        moveY(-1.5, 1);

        //grabbing 2nd block
        dragBlock("rot","down");
        Thread.sleep(400);
        dragBlock("grasp", "chomp");
        Thread.sleep(700);
        dragBlock("rot", "up");

        //moving back and towards the foundation side
        robot.moveY(-8, 0.2,2);
        Thread.sleep(150);

        //Motor gets weird power statem
        robot.NOENC();
        gyro.rotationCorrection(0, 0.3, 1);
        robot.initializeAuto(hardwareMap);

        moveX(-105, 0.5);

        robot.NOENC();
        gyro.rotationCorrection(0, 0.3, 1);
        robot.initializeAuto(hardwareMap);

        //releasing 2nd block
        moveY(19, 0.5);
        dragBlock("grasp", "unchomp");
        moveY(-10, 0.5);

        robot.NOENC();
        gyro.rotationCorrection(0, 0.3, 1);
        robot.initializeAuto(hardwareMap);

        // FoundRot();

        dragBlock("rot", "travel");
        moveX(35.0, 1.0);
    }

>>>>>>> Stashed changes

    private int inchesToCounts(double inches) {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }

    private void moveSetDistance()
    {
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

<<<<<<< Updated upstream
        double current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 50;
=======
        double current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 56;
>>>>>>> Stashed changes

        while (current >= DESIRED_D)
        {
            current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            MotorFrontY.setPower(0.3);
            MotorBackY.setPower(0.3);
            MotorFrontX.setPower(0);
            MotorBackX.setPower(0);
        }

<<<<<<< Updated upstream
        MotorFrontY.setPower(-0.1);
        MotorBackY.setPower(-0.1);

        telemetry.addData("MotorFrontX power", MotorFrontX.getPower());
        telemetry.addData("MotorFrontY power", MotorFrontY.getPower());
        telemetry.addData("MotorBackX power", MotorBackX.getPower());
        telemetry.addData("MotorackY power", MotorBackY.getPower());
=======
        telemetry.addData("robot.MotorFrontX power", robot.MotorFrontX.getPower());
        telemetry.addData("robot.MotorFrontY power", robot.MotorFrontY.getPower());
        telemetry.addData("robot.MotorBackX power", robot.MotorBackX.getPower());
        telemetry.addData("MotorackY power", robot.MotorBackY.getPower());
>>>>>>> Stashed changes
        telemetry.update();

        MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorFrontY.setPower(0);
        MotorBackY.setPower(0);
    }

    private void detectBlock() throws InterruptedException
    {
<<<<<<< Updated upstream
        MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 55;

        while (current >= DESIRED_D)
        {
            current = (leftDistance.getDistance(DistanceUnit.MM) + rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            MotorFrontY.setPower(0.3);
            MotorBackY.setPower(0.3);
        }

        MotorFrontY.setPower(-0.1);
        MotorBackY.setPower(-0.1);

        Thread.sleep(2000);

        double leftNormalizedColors = (leftColor.green() + leftColor.red() + leftColor.blue()) / Math.pow(leftDistance.getDistance(DistanceUnit.MM), 2);
        double rightNormalizedColors = (rightColor.green() + rightColor.red() + rightColor.blue()) / Math.pow(rightDistance.getDistance(DistanceUnit.MM), 2);

        Log.i("LeftNormalizationValue", ""+leftNormalizedColors);
        Log.i("RightNormalizationValue", ""+rightNormalizedColors);
=======
        double leftNormalizedColors = (robot.leftColor.red()*robot.leftColor.green())/Math.pow(robot.leftColor.blue(),2);
        double rightNormalizedColors = (robot.rightColor.red()*robot.rightColor.green()/ Math.pow(robot.rightColor.blue(),2));
>>>>>>> Stashed changes

        // Yellow is greater than 0.5 and black is less than 0.5
        //Y[YB]
        if (leftNormalizedColors > 0.5 && rightNormalizedColors < 0.5)
        {
<<<<<<< Updated upstream
            blockPos = 'r';
            moveX(2, 0.2);
=======
            moveX(-9, 0.25);
>>>>>>> Stashed changes
            telemetry.addData("right", null);
            telemetry.update();
        }

        //Y[BY]
        if (leftNormalizedColors < 0.5 && rightNormalizedColors > 0.5)
        {
            blockPos = 'c';
            moveX(10, 0.2);
            telemetry.addData("center", null);
            telemetry.update();
        }

        //B[YY]
        if (leftNormalizedColors > 0.5 && rightNormalizedColors > 0.5)
        {
<<<<<<< Updated upstream
            blockPos = 'l';
            moveX(20, 0.2);
=======
            moveX(9, 0.25);
>>>>>>> Stashed changes
            telemetry.addData("left", null);
            telemetry.update();
        }
<<<<<<< Updated upstream

        Thread.sleep(500);
        telemetry.update();
        Thread.sleep(500);
=======
>>>>>>> Stashed changes
    }

    void agaga(String position) {
        if (position.equals("release"))
        {
            angle1.setPosition(1);
            angle2.setPosition(0);

            grasp1.setPosition(1);
            grasp2.setPosition(0);

            angle1.setPosition(0);
            angle1.setPosition(1);
        }

        if (position.equals("grasp"))
        {
            grasp1.setPosition(0);
            grasp2.setPosition(1);
        }
    }

    void moveY(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (counts)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (counts)));

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy()) {
            telemetry.addData("Move Y method error = ", MotorFrontY.getTargetPosition() - (MotorFrontY.getCurrentPosition() + counts));
        }
    }

    private void moveX(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        MotorFrontX.setPower(power);
        MotorBackX.setPower(power);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (counts)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (counts)));

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Move X method error = ", MotorFrontX.getTargetPosition() - (MotorFrontX.getCurrentPosition() + counts));
            telemetry.update();
        }
    }

    void botRotate(int distance, double power)
    {
        MotorBackY.setPower(power);
        MotorBackX.setPower(power);
        MotorFrontX.setPower(power);
        MotorFrontY.setPower(power);

        int COUNTS = inchesToCounts(distance);

        MotorFrontX.setTargetPosition(MotorFrontX.getCurrentPosition() + (COUNTS));
        MotorBackX.setTargetPosition(MotorBackX.getCurrentPosition() - (COUNTS));
        MotorFrontY.setTargetPosition(MotorFrontY.getCurrentPosition() + COUNTS);
        MotorBackY.setTargetPosition(MotorBackY.getCurrentPosition() - COUNTS);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    void armExtend(int counts, double power)
    {
        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorExtend.setPower(power);

        motorExtend.setTargetPosition(motorExtend.getCurrentPosition() + counts);

        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

<<<<<<< Updated upstream
        while (opModeIsActive() && motorExtend.isBusy()) {
            telemetry.addData("extension error = ", motorExtend.getTargetPosition() - (motorExtend.getCurrentPosition() + counts));
            telemetry.update();
            if((motorExtend.getCurrentPosition()> motorExtend.getTargetPosition()-20) && (motorExtend.getCurrentPosition()<motorExtend.getTargetPosition()+20))
                break;
        }
=======
        while(opModeIsActive() && robot.MotorBackX.isBusy() && robot.MotorFrontX.isBusy())
        {
            telemetry.addData("frontX", robot.MotorFrontX.isBusy());
            telemetry.addData("backX", robot.MotorBackX.isBusy());
            telemetry.update();
        }

        robot.setZero();
    }

    public void moveY(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        robot.MotorFrontY.setPower(-power);
        robot.MotorBackY.setPower(-power);

        robot.MotorFrontY.setTargetPosition((robot.MotorFrontY.getCurrentPosition() + (counts)));
        robot.MotorBackY.setTargetPosition((robot.MotorBackY.getCurrentPosition() + (counts)));

        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.MotorBackY.isBusy() && robot.MotorFrontY.isBusy())
        {

        }

        robot.setZero();
>>>>>>> Stashed changes
    }

    void armRotate(int counts, double power)
    {

        motorRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorRotate.setPower(power);

        motorRotate.setTargetPosition(motorRotate.getCurrentPosition() + counts);

        motorRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motorRotate.isBusy())
        {
            telemetry.addData(" rotation error = ", motorRotate.getTargetPosition() - (motorRotate.getCurrentPosition() + counts));
            telemetry.update();
            if((motorRotate.getCurrentPosition()> motorRotate.getTargetPosition()-20) && (motorRotate.getCurrentPosition()<motorRotate.getTargetPosition()+20))
                break;
        }
    }

    void collectBlock(String servo, String pos) throws InterruptedException
    {
        if(servo.equalsIgnoreCase("right"))
        {
            if (pos.equalsIgnoreCase("down"))
                rightCollection.setPosition(0.6);

            if (pos.equalsIgnoreCase("up"))
                rightCollection.setPosition(0);
        }

        if(servo.equalsIgnoreCase("left"))
        {
            if (pos.equalsIgnoreCase("down"))
                leftCollection.setPosition(0.35);

            if (pos.equalsIgnoreCase("up"))
<<<<<<< Updated upstream
                leftCollection.setPosition(1);
=======
                robot.block_drag.setPosition(0.65);

            if(pos.equalsIgnoreCase("travel"))
                robot.block_drag.setPosition(0.80);
>>>>>>> Stashed changes
        }
    }

    void FoundRot()
    {

    }
}