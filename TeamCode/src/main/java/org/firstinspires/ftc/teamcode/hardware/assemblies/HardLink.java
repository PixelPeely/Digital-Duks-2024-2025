package org.firstinspires.ftc.teamcode.hardware.assemblies;

import org.firstinspires.ftc.teamcode.hardware.CachedPeripheral;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_AAEServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_CRServo;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_DcMotor;
import org.firstinspires.ftc.teamcode.hardware.wrappers.C_Servo;

import java.util.ArrayList;
import java.util.List;

public class HardLink implements CachedPeripheral {
    public static class Link {
        public CachedPeripheral component;
        public double power;
        private double ratio;

        /**
         * @param _component Component to link
         * @param _ratio The number of revolutions this component needs to make per baseline revolution
         */
        public Link(CachedPeripheral _component, double _ratio) {
            component = _component;
            ratio = _ratio;
        }

        public void setPower(double _power) {
            power = _power * ratio;
        }

        public void dispatch() {
            if (component instanceof C_DcMotor) ((C_DcMotor) component).setPower(power);
            else if (component instanceof C_Servo) ((C_Servo) component).setPosition(power);
            else if (component instanceof C_CRServo) ((C_CRServo) component).setPower(power);
            else if (component instanceof C_AAEServo) ((C_AAEServo) component).setTargetPosition(power);
            else if (component instanceof Linkage) ((Linkage) component).setLinearPosition(power);
            component.dispatchCache();
        }
    }

    public List<Link> links = new ArrayList<Link>();
    private double largestRatio;

    public HardLink(Link... _links) {
        for (Link l : _links) addLink(l);
    }

    public void addLink(Link link) {
        links.add(link);
        largestRatio = Math.max(largestRatio, link.ratio);
    }

    public void removeLink(Link link) {
        links.remove(link);

        if (link.ratio == largestRatio) {
            largestRatio = 0;
            for (Link l : links)
                largestRatio = Math.max(l.ratio, largestRatio);
        }
    }

    public void setPower(double power) {
        final double normal = power / largestRatio;
        links.forEach(l -> l.setPower(normal));
    }

    public double getAveragePower() {
        double sum = 0;
        for (Link l : links) {
            double _power = 0;
            if (l.component instanceof C_DcMotor) _power = ((C_DcMotor) l.component).getCurrentPosition();
            else if (l.component instanceof C_Servo) _power = ((C_Servo) l.component).getPosition();
            else if (l.component instanceof C_CRServo) _power = ((C_CRServo) l.component).getPower();
            else if (l.component instanceof C_AAEServo) _power = ((C_AAEServo) l.component).getCurrentPosition();
            else if (l.component instanceof Linkage) _power = ((Linkage) l.component).getLinearPosition();
            sum += _power / l.ratio;
        }
        return sum;
    }

    @Override
    public void dispatchCache() {
        links.forEach(Link::dispatch);
    }

    @Override
    public void refreshCache() {
        links.forEach(l -> l.component.refreshCache());
    }

    @Override
    public void allowDispatch(boolean state) {
        links.forEach(l -> l.component.allowDispatch(state));
    }

    @Override
    public boolean isValid() {
        for (Link l : links) if (!l.component.isValid()) return false;
        return true;
    }
}
