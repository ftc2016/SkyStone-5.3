package org.firstinspires.ftc.teamcode.GoBuilda.Development;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Utils.GoBuildaUtil;
import org.firstinspires.ftc.teamcode.Utils.GyroCode;

public class RotationCorrection extends OpMode
{

    GoBuildaUtil robot = new GoBuildaUtil();
    GyroCode gyro = new GyroCode();

    @Override
    public void init()
    {
        robot.initializeAuto(hardwareMap);
        gyro.initGyro(hardwareMap);

    }

    @Override
    public void loop()
    {
        gyro.rotationCorrection(0);
    }
}
