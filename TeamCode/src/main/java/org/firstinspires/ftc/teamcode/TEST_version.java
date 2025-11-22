package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.Math;

import com.qualcomm.hardware.rev.RevIMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="TEST_version")
public class TEST_version extends LinearOpMode {
    @Override
    public void runOpMode() {
        RevIMU imu = hardwareMap.get(RevIMU.class, "imu");
        imu.initialize();

        waitForStart();

        while(opModeIsActive()) {

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double yaw = angles.getYaw(AngleUnit.DEGREES);

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;

            double angle = Math.atan2(y, x);
            double controller_angle = Math.toDegrees(angle);
            double power = Math.hypot(x, y);

            telemetry.addData("IMU",yaw);
            telemetry.addData("Controller角度",controller_angle);
            telemetry.addData("倒し度合い",power);
            telemetry.update();


        }
    }
}
