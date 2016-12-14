use <car.scad>;
use <jetsontx1.scad>;
use <Zed.scad>;

car();
translate([300, -5, 45]) rotate([0, 0, 90])jetsonTX1();
translate([350, 0, 60]) rotate([90, 0, 90]) zedCam();