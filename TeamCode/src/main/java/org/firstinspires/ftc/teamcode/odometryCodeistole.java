
public void odometry() {
    oldRightPosition = currentRightPosition;
    oldLeftPosition = currentLeftPosition;
    oldAuxPosition = currentAuxPosition;

    currentRightPosition = -encoderRight.getCurrentPosition();
    currentLeftPosition = encoderLeft.getCurrentPosition();
    currentAuxPosition = encoderAux.getCurrentPosition();

    int dn1 = currentLeftPosition  - oldLeftPosition;
    int dn2 = currentRightPosition - oldRightPosition;
    int dn3 = currentAuxPosition - oldAuxPosition;

    // the robot has moved and turned a tiny bit between two measurements:
    double dtheta = cm_per_tick * ((dn2-dn1) / (LENGTH));
    double dx = cm_per_tick * ((dn1+dn2) / 2.0);
    double dy = cm_per_tick * (dn3 + ((dn2-dn1) / 2.0));

    telemetrydx = dx;
    telemetrydy = dy;
    telemetrydh = dtheta;

    // small movement of the robot gets added to the field coordinate system:
    pos.h += dtheta / 2;
    pos.x += dx * Math.cos(pos.h) - dy * Math.sin(pos.h);
    pos.y += dx * Math.sin(pos.h) + dy * Math.cos(pos.h);
    pos.h += dtheta / 2;
    pos.h = normDiff(pos.h);
}
