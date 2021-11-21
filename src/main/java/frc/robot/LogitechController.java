package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.GenericHID;

public class LogitechController extends GenericHID {
    public LogitechController(int port) {
        super(port);
        HAL.report(tResourceType.kResourceType_Joystick, port + 1);
    }

    public enum Axis {
        kLeftX(0), kRightX(4), kLeftY(1), kRightY(5), kLeftTrigger(2), kRightTrigger(3);

        @SuppressWarnings({ "MemberName", "PMD.SingularField" })
        public final int value;

        Axis(int value) {
            this.value = value;
        }
    }

    public enum Button {
        kBumperLeft(5),
        kBumperRight(6),
        kStickLeft(9),
        kStickRight(10),
        kA(1),
        kB(2),
        kX(3),
        kY(4),
        kBack(7),
        kStart(8);
    
        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
    
        Button(int value) {
          this.value = value;
        }
      }

    /**
     * Get the X axis value of the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The X axis value of the controller.
     */
    @Override
    public double getX(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawAxis(Axis.kLeftX.value);
        } else {
            return getRawAxis(Axis.kRightX.value);
        }
    }

    /**
     * Get the Y axis value of the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The Y axis value of the controller.
     */
    @Override
    public double getY(Hand hand) {
        if (hand.equals(Hand.kLeft)) {
            return getRawAxis(Axis.kLeftY.value);
        } else {
            return getRawAxis(Axis.kRightY.value);
        }
    }

}