package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.hardware.DukHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Shuttle;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SubmersibleIntake;

import java.util.function.Predicate;

public class InternalTaskInstances {
    public static class SubmersibleIntakeTasks {
        public final Predicate<Double> extendoTask;
        public final Predicate<Double> shuttleTask;
        public final Predicate<Double> liftClearTask;

        private final SubmersibleIntake submersibleIntake;

        public SubmersibleIntakeTasks(SubmersibleIntake _submersibleIntake) {
            submersibleIntake = _submersibleIntake;

            extendoTask = t -> {
                submersibleIntake.extendo.servos.setPower(submersibleIntake.getState().extendoPosition);
                submersibleIntake.carriage.servos.setPower(submersibleIntake.getState().carriagePosition);
                return true;
            };

            shuttleTask = t -> {
                submersibleIntake.shuttle.setState(submersibleIntake.getState().shuttleState);
                if (submersibleIntake.getState().extendoPosition == 0 && submersibleIntake.desiredLiftState != null) {
                    DukHardwareMap.InternalInteractions.setLiftState(submersibleIntake.desiredLiftState);
                    submersibleIntake.desiredLiftState = null;
                }
                return true;
            };

            liftClearTask = t -> {
                if (DukHardwareMap.InternalInteractions.getLiftPosition() > Lift.STATE.EXTENDO_CLEAR.position - DukConstants.AUTOMATED_CONTROLLER_PARAMS.LIFT_ERROR) {
                    submersibleIntake.extendoMovement();
                    return true;
                }
                return false;
            };
        }

        public void cancelAll() {
            TimeManager.cancelTask(liftClearTask);
            TimeManager.cancelTask(extendoTask);
            TimeManager.cancelTask(shuttleTask);
            submersibleIntake.desiredLiftState = null;
        }
    }

    public static class ShuttleTasks {
        public final Predicate<Double> pickupTask;
        public final Predicate<Double> carryTask;

        private final Shuttle shuttle;

        public ShuttleTasks(Shuttle _shuttle) {
            shuttle = _shuttle;

            pickupTask = t -> {
                shuttle.claw.setState(Shuttle.STATE.PICKUP.closed);
                return true;
            };

            carryTask = t -> {
                shuttle.setState(Shuttle.STATE.CARRY);
                return true;
            };
        }

        public void cancelAll() {
            TimeManager.cancelTask(pickupTask);
        }
    }
}
