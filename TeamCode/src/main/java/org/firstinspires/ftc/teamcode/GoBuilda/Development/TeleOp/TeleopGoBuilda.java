package org.firstinspires.ftc.teamcode.GoBuilda.Development.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Utils.UtilTest;

@TeleOp(name = "TeleTubbies Test", group = "actual")
public class TeleopGoBuilda extends OpMode
{

    private double liftPosScale = 5.0, liftPowScale = 0.025;
    private double liftPosCurrent, liftPosDes, liftPosError, liftPow;
    private double integrater = 0, intpower = 0.00075;

    private double extendPosScale = 5.0, extendPowScale = 0.025;
    private double extendPosCurrent, extendPosDes, extendPosError, extendPow;


    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0.5, foundPos = 0;

    UtilTest robot = new UtilTest();

    @Override
    public void init()
    {
        robot.initializeTele(hardwareMap);
        robot.initializeSensors(hardwareMap);
    }

    @Override
    public void loop() {
        if (turtle)
            multiplier = 0.45;
        else
            multiplier = 1;

        if (sloth)
            speedK = 0.45;
        else
            speedK = 1;


        // initializing wheel variables
        double powerXWheels = 0;
        double powerYWheels = 0;

        // Handle regular movement
        powerYWheels = gamepad1.left_stick_y;

        // Handle sliding movement
        powerXWheels = gamepad1.right_stick_x;

        robot.MotorBackX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);
        robot.MotorFrontX.setPower((Math.abs(powerXWheels)*powerXWheels)*multiplier);

        robot.MotorBackY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);
        robot.MotorFrontY.setPower((Math.abs(powerYWheels)*powerYWheels)*multiplier);

        //Turning clockwise
        float rotationCW = gamepad1.right_trigger;
        float rotationACW = gamepad1.left_trigger;

        if(gamepad1.b)
            turtle = !turtle;

        if(gamepad2.b)
            sloth = !sloth;

        if (gamepad1.a) {
            multiplier = 1;
        }
        if (gamepad2.a) {
            speedK = 1;
        }

        if(rotationACW !=0)
        {
            robot.MotorFrontX.setPower(-gamepad1.left_trigger);
            robot.MotorFrontY.setPower(gamepad1.left_trigger);
            robot.MotorBackX.setPower(gamepad1.left_trigger);
            robot.MotorBackY.setPower(-gamepad1.left_trigger);
        }

        if (rotationCW !=0)
        {
            robot.MotorFrontX.setPower(gamepad1.right_trigger);
            robot.MotorFrontY.setPower(-gamepad1.right_trigger);
            robot.MotorBackX.setPower(-gamepad1.right_trigger);
            robot.MotorBackY.setPower(gamepad1.right_trigger);
        }

        if (gamepad2.left_bumper)
        {
            robot.graspL.setPosition(1);
            robot.graspR.setPosition(0);
        }
        if (gamepad2.right_bumper) {
        robot.graspL.setPosition(0);
        robot.graspR.setPosition(1);
    }

        if(gamepad1.dpad_up) { foundPos += 0.05; }

        if(gamepad1.dpad_down) {  foundPos -= 0.05; }

        if(gamepad1.dpad_left) { rotPos -= 0.15; }

        if(gamepad1.dpad_left) { rotPos += 0.15; }

        if(gamepad1.right_bumper) { robot.block_drag_grasp.setPosition(1); }

        if(gamepad1.left_bumper) { robot.block_drag_grasp.setPosition(0); }

        robot.block_drag.setPosition(rotPos);


        telemetry.addData("rot Data", rotPos);
        telemetry.addData("robot.block_drag_grasp posn", robot.block_drag_grasp.getPosition());
        telemetry.update();

        liftPosCurrent = robot.motorVertical.getCurrentPosition();

        liftPosDes += liftPosScale * gamepad2.left_stick_y;
        liftPosError = liftPosDes - liftPosCurrent;
        integrater += liftPosError;
        liftPow = Math.min(Math.max(liftPowScale*liftPosError + integrater*intpower, -1.00), 1.00);
        robot.motorVertical.setPower(liftPow);


        extendPosCurrent = robot.motorExtend.getCurrentPosition();

        extendPosDes += extendPosScale * gamepad2.right_stick_y;
        extendPosError = extendPosDes - extendPosCurrent;
        extendPow = Math.min(Math.max(extendPowScale*extendPosError, -1.00), 1.00);
        robot.motorExtend.setPower(extendPow);

    }
}
