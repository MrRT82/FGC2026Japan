package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.lang.Math;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

@TeleOp(name="TEST_version")
public class TEST_version extends LinearOpMode {
    @Override
    public void runOpMode() {
        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()) {

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double yaw = angles.firstAngle;

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
