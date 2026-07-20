public class RobotHardware {
    public DcMotor leftMotor;
    public DcMotor rightMotor;
    public DcMotor armMotor;
    public TouchSensor touchSensor;
    public DistanceSensor distanceSensor;

    public RobotHardware(HardwareMap hw){
        DcMotor leftMotor = hw.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hw.get(DcMotor.class, "rightMotor");
        DcMotor armMotor = hw.get(DcMotor.class, "armMotor");
        TouchSensor touchSensor = hw.get(TouchSensor.class, "touchSensor");
        DistanceSensor distanceSensor = hw.get(DistanceSensor.class, "distanceSensor");

        leftMotor.setDirection(DcMotor.Direction.RESERVE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior)(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
