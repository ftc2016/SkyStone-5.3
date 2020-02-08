package org.firstinspires.ftc.teamcode.GoBuilda.Development.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;
import org.firstinspires.ftc.teamcode.Utils.GyroCode;

@Autonomous(name="Blue SkyStoneDragger", group = "Blue")
public class BlueSkyStoneDragger extends LinearOpMode
{

    char blockPos = ' ';

    GoBuildaUtil robot = new GoBuildaUtil();
    GyroCode gyro = new GyroCode();


    @Override
    public void runOpMode() throws InterruptedException
    {
        robot.initializeBlueAuto(hardwareMap);
        robot.block_drag_grasp.setPosition(0);
        robot.initializeSensors(hardwareMap);
        gyro.initGyro(hardwareMap);

        waitForStart();

        //First Block
        detectBlock();

        dragBlock("rot","down");
        Thread.sleep(700);
        dragBlock("grasp", "chomp");
        Thread.sleep(700);
        dragBlock("rot", "up");

        robot.moveY(-8, 0.2,2);

        //moving towards the foundation
        if(blockPos == 'l') { moveX(-92, 0.5); }

        if(blockPos == 'c') { moveX(-83, 0.5); }

        if(blockPos == 'r') { moveX(-75, 0.5); }

        //putting it on the foundation
        moveY(14, 0.75);
        dragBlock("grasp", "unchomp");
        Thread.sleep(500);

        moveY(-14, 0.5);
        dragBlock("rot", "travel");

        //2nd block
        if(blockPos == 'l') { moveX(142, 0.4); }

        if(blockPos == 'c') { moveX(132, 0.4); }

        if(blockPos == 'r') { moveX(127, 0.4); }

        Thread.sleep(250);

        telemetry.addData("Entering Gyro", null);
        telemetry.update();

        robot.ENC();
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

//        robot.NOENC();
//        gyro.rotationCorrection(0, 0.3);
//        robot.initializeAuto(hardwareMap);

        moveX(105, 0.5);

//        robot.NOENC();
//        gyro.rotationCorrection(0, 0.3);
//        robot.initializeAuto(hardwareMap);

        //releasing 2nd block
        moveY(13, 0.5);
        dragBlock("grasp", "unchomp");
        moveY(-19, 0.5);

//        robot.NOENC();
//        gyro.rotationCorrection(0, 0.3);
//        robot.initializeAuto(hardwareMap);

        // FoundRot();

        dragBlock("rot", "travel");
        moveX(-35.0, 1.0);
    }


    private int inchesToCounts(double inches) {
        //wheel specification
        final double Servocity_Omnni_Circumference = Math.PI * 4;
        final double GoBuilda_YJ_435_eventsPerRev = 383.6;
        final double COUNTS_PER_REVOLUTION = GoBuilda_YJ_435_eventsPerRev / Servocity_Omnni_Circumference;

        return (int) (COUNTS_PER_REVOLUTION * inches);
    }

    private void moveSetDistance()
    {
        robot.MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 56;

        while (current >= DESIRED_D)
        {
            current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            robot.MotorFrontY.setPower(0.3);
            robot.MotorBackY.setPower(0.3);
            robot.MotorFrontX.setPower(0);
            robot.MotorBackX.setPower(0);
        }

        telemetry.addData("robot.MotorFrontX power", robot.MotorFrontX.getPower());
        telemetry.addData("robot.MotorFrontY power", robot.MotorFrontY.getPower());
        telemetry.addData("robot.MotorBackX power", robot.MotorBackX.getPower());
        telemetry.addData("MotorackY power", robot.MotorBackY.getPower());
        telemetry.update();

        robot.MotorFrontY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.MotorFrontY.setPower(0);
        robot.MotorBackY.setPower(0);
    }

    private void detectBlock() throws InterruptedException
    {
        robot.MotorFrontY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorBackY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.MotorFrontY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.MotorBackY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;
        final int DESIRED_D = 55;

        while (current >= DESIRED_D)
        {
            current = (robot.leftDistance.getDistance(DistanceUnit.MM) + robot.rightDistance.getDistance(DistanceUnit.MM)) / 2;

            telemetry.addData("Moving to set Distance", current);
            telemetry.update();

            robot.MotorFrontY.setPower(0.3);
            robot.MotorBackY.setPower(0.3);
        }

        robot.MotorFrontY.setPower(-0.1);
        robot.MotorBackY.setPower(-0.1);

        Thread.sleep(1000);

        double leftNormalizedColors = (robot.leftColor.red()*robot.leftColor.green())/Math.pow(robot.leftColor.blue(),2);
        double rightNormalizedColors = (robot.rightColor.red()*robot.rightColor.green()/ Math.pow(robot.rightColor.blue(),2));

        // Yellow is greater than 0.5 and black is less than 0.5
        //[YY]B
        if (leftNormalizedColors > 5 && rightNormalizedColors > 5)
        {
            moveX(15, 0.25);
            telemetry.addData("right", null);
            telemetry.update();
            blockPos = 'r';

        }

        //[YB]Y
        if (leftNormalizedColors > 5 && rightNormalizedColors < 5)
        {
            moveX(9, 0.25);
            telemetry.addData("center", null);
            telemetry.update();
            blockPos = 'c';
        }

        //[BY]Y
        if (leftNormalizedColors < 5 && rightNormalizedColors > 5)
        {
            moveX(0, 0.25);
            telemetry.addData("left", null);
            telemetry.update();
            blockPos = 'l';

        }
    }

    void agaga(String position) {
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

    private void moveX(double inches, double power)
    {
        int counts = inchesToCounts(inches);

        robot.MotorFrontX.setPower(power);
        robot.MotorBackX.setPower(power);

        robot.MotorFrontX.setTargetPosition((robot.MotorFrontX.getCurrentPosition() + (counts)));
        robot.MotorBackX.setTargetPosition((robot.MotorBackX.getCurrentPosition() + (counts)));

        robot.MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && robot.MotorBackX.isBusy() && robot.MotorFrontX.isBusy())
        {

        }
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

        while (robot.MotorBackY.isBusy() && (Math.abs(robot.MotorBackY.getCurrentPosition()-robot.MotorBackY.getTargetPosition())>50) && robot.MotorFrontY.isBusy()) { }
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

    void FoundRot()
    {

    }
}