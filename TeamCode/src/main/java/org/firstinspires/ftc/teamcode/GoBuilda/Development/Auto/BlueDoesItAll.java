package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;
import org.firstinspires.ftc.teamcode.Utils.GyroCode;
import org.firstinspires.ftc.teamcode.Utils.GyroTest;

@Autonomous(name="Blue Trials", group = "Blue")
public class BlueDoesItAll extends LinearOpMode
{

    char blockPos = ' ';
    GoBuildaUtil robot = new GoBuildaUtil();
    GyroCode gyro = new GyroCode();
    double error = 0;

    ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    GyroTest gt = new GyroTest();

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.initializeBlueAuto(hardwareMap);
        robot.block_drag_grasp.setPosition(0);
        robot.initializeSensors(hardwareMap);
        gyro.initGyro(hardwareMap);

        waitForStart();

        //First Block
        moveY(16, 0.7, 6);
        moveSetDistance(60);
        detectBlock();

        dragBlock("rot","down");
        Thread.sleep(700);
        dragBlock("grasp", "chomp");
        Thread.sleep(700);
        dragBlock("rot", "up");
        robot.moveY(-8, 0.2,2);

        //moving towards the foundation
        if(blockPos == 'l') { moveX(-92, 0.7,6); }

        if(blockPos == 'c') { moveX(-83, 0.7, 7); }

        if(blockPos == 'r') { moveX(-75, 0.7 ,8); }
//
        //putting it on the foundation
        moveSetDistance(55);
        dragBlock("grasp", "unchomp");
        Thread.sleep(500);

        moveY(-10, 0.5, 2);
        dragBlock("rot", "travel");


//      travelling across
        telemetry.addData("Entering Gyro", null);
        telemetry.update();
        robot.initializeTele(hardwareMap);
        gyro.rotationCorrection(0, 0.3, 1);
        robot.initializeBlueAuto(hardwareMap);
        dragBlock("rot", "up");
        telemetry.addData("EXItinG Gyro", null);
        telemetry.update();
//
        //2nd block
        if(blockPos == 'l') { moveX(96, 0.7, 8); }

        if(blockPos == 'c') { moveX(106, 0.7, 8); }

        if(blockPos == 'r') { moveX(96, 0.7, 8); }

        Thread.sleep(150);

        telemetry.addData("Entering Gyro", null);
        telemetry.update();
        robot.initializeTele(hardwareMap);
        gyro.rotationCorrection(0, 0.3, 1);
        robot.initializeBlueAuto(hardwareMap);
        dragBlock("rot", "up");
        telemetry.addData("EXItinG Gyro", null);
        telemetry.update();

        moveSetDistance(60);
//        moveY(-1.5, 1);
//
        //grabbing 2nd block
        dragBlock("rot","down");
        Thread.sleep(400);
        dragBlock("grasp", "chomp");
        Thread.sleep(700);
        dragBlock("rot", "up");

        //moving back and towards the foundation side
        robot.moveY(-8, 0.2, 2);
        Thread.sleep(150);

        robot.initializeTele(hardwareMap);
        gyro.rotationCorrection(0, 0.3, 1);
        robot.initializeBlueAuto(hardwareMap);
//
        moveX(-105, 0.5, 6);

//        robot.NOENC();
//        gyro.rotationCorrection(0, 0.3);
//        robot.initializeAuto(hardwareMap);

        //releasing 2nd block
        moveY(13, 0.5, 4);
        dragBlock("grasp", "unchomp");
//        moveY(-19, 0.5, 4);

//        robot.NOENC();
//        gyro.rotationCorrection(0, 0.3);
//        robot.initializeAuto(hardwareMap);

        FoundRot();

        dragBlock("rot", "travel");
//        moveX(-35.0, 1.0, 4);
    }


    private int inchesToCounts(double inches) {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }

    private void moveSetDistance(int DESIRED_D)
    {
        robot.MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;

        while (current >= DESIRED_D)
        {
            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            robot.MotorFrontY.setPower(0.3);
            robot.MotorBackY.setPower(0.3);
            robot.MotorFrontX.setPower(0);
            robot.MotorBackX.setPower(0);

            current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;
        }

        robot.setZero();

        robot.MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void detectBlock() throws InterruptedException
    {
        double leftNormalizedColors;
        double rightNormalizedColors;

        int leftCt=0;
        int middleCt=0;
        int rightCt=0;

        for (int i=0; i<10; i++)
        {
            leftNormalizedColors = (robot.leftColor.red()*robot.leftColor.green())/Math.pow(robot.leftColor.blue(),2);
            rightNormalizedColors = (robot.rightColor.red()*robot.rightColor.green()/ Math.pow(robot.rightColor.blue(),2));


            if (leftNormalizedColors > 5 && rightNormalizedColors > 5)
            {
                rightCt++;
            }

            //[YB]Y
            if (leftNormalizedColors > 5 && rightNormalizedColors < 5)
            {
                middleCt++;
            }

            //[BY]Y
            if (leftNormalizedColors < 5 && rightNormalizedColors > 5)
            {
                leftCt++;
            }
       }

        if(leftCt > middleCt && leftCt > rightCt)
        {
            telemetry.addData("left", null);
            telemetry.update();
            Thread.sleep(1500);
            moveX(0, 0.25, 1);
            blockPos = 'l';
        }

        if(middleCt > rightCt && middleCt > leftCt)
        {
            telemetry.addData("center", null);
            telemetry.update();
            Thread.sleep(500);
            moveX(9, 0.25, 1);
            blockPos = 'c';
        }

        if(rightCt > leftCt && rightCt > middleCt)
        {
            telemetry.addData("right", null);
            telemetry.update();
            Thread.sleep(500);
            moveX(15, 0.25, 1);
            blockPos = 'r';
        }
    }

    void agaga(String position)
    {
        if (position.equals("release"))
        {
            robot.graspL.setPosition(1);
            robot.graspR.setPosition(0);
        }

        if (position.equals("grasp"))
        {
            robot.graspL.setPosition(0);
            robot.graspR.setPosition(1);
        }
    }

    private void moveX(double inches, double power, double timeLimit)
    {
        elapsedTime.reset();
        elapsedTime.startTime();
        error = 0;

        int counts = inchesToCounts(inches);

        robot.MotorFrontX.setPower(power);
        robot.MotorBackX.setPower(power);
        robot.MotorFrontY.setPower(0);
        robot.MotorBackY.setPower(0);
        robot.MotorFrontX.setTargetPosition((robot.MotorFrontX.getCurrentPosition() + (counts)));
        robot.MotorBackX.setTargetPosition((robot.MotorBackX.getCurrentPosition() + (counts)));

        robot.MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        error = robot.MotorFrontX.getTargetPosition() - robot.MotorFrontX.getCurrentPosition();


        while (opModeIsActive() && robot.MotorBackX.isBusy() && robot.MotorFrontX.isBusy() && elapsedTime.time()<30 && Math.abs(error)>50)
        {

        }
    }

    public void moveY(double inches, double power, double timeLimit)
    {
        elapsedTime.reset();
        elapsedTime.startTime();

        int counts = inchesToCounts(inches);

        robot.MotorFrontY.setPower(-power);
        robot.MotorBackY.setPower(-power);
        robot.MotorFrontX.setPower(0);
        robot.MotorBackX.setPower(0);

        robot.MotorFrontY.setTargetPosition((robot.MotorFrontY.getCurrentPosition() + (counts)));
        robot.MotorBackY.setTargetPosition((robot.MotorBackY.getCurrentPosition() + (counts)));

        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (robot.MotorBackY.isBusy() && robot.MotorFrontY.isBusy()  && elapsedTime.time()<30) { }
    }

    public void foundationServo(String cmd)
    {
        if(cmd.equalsIgnoreCase("engage"))
        {
            robot.foundation2.setPosition(1);
            robot.foundation.setPosition(0);
        }

        if(cmd.equalsIgnoreCase("disengage"))
        {
            robot.foundation2.setPosition(0);
            robot.foundation.setPosition(1);
        }

    }


//    void armExtend(int counts, double power)
//    {
//        motorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        motorExtend.setPower(power);
//
//        motorExtend.setTargetPosition(motorExtend.getCurrentPosition() + counts);
//
//        motorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (opModeIsActive() && motorExtend.isBusy()) {
//            telemetry.addData("extension error = ", motorExtend.getTargetPosition() - (motorExtend.getCurrentPosition() + counts));
//            telemetry.update();
//            if((motorExtend.getCurrentPosition()> motorExtend.getTargetPosition()-20) && (motorExtend.getCurrentPosition()<motorExtend.getTargetPosition()+20))
//                break;
//        }
//    }

    void dragBlock(String servo, String pos) throws InterruptedException
    {
        if(servo.equalsIgnoreCase("grasp"))
        {
            if (pos.equalsIgnoreCase("chomp"))
                robot.block_drag_grasp.setPosition(1);

            if (pos.equalsIgnoreCase("unchomp"))
                robot.block_drag_grasp.setPosition(0);
        }

        if(servo.equalsIgnoreCase("rot"))
        {
            if (pos.equalsIgnoreCase("down"))
                robot.block_drag.setPosition(1);

            if (pos.equalsIgnoreCase("up"))
                robot.block_drag.setPosition(0.65);

            if(pos.equalsIgnoreCase("travel"))
                robot.block_drag.setPosition(0.80);
        }
    }

    void FoundRot() throws InterruptedException
    {
        foundationServo("engage");
        Thread.sleep(1500);
        moveY(-15, 0.9, 4);

        telemetry.addData("Entering Gyro", null);
        telemetry.update();
        robot.initializeTele(hardwareMap);
        gyro.rotationCorrection(90, 1, 15);
        robot.initializeBlueAuto(hardwareMap);
        dragBlock("rot", "up");
        telemetry.addData("EXItinG Gyro", null);
        telemetry.update();

        foundationServo("disengage");



    }
}