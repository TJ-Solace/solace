$fn=100;

module zedCam() {
    color("skyblue")
    linear_extrude(height=32, scale=[1, 28/30], slices=20, twist=0)
        union() {
            square([85.5*2+4-30, 30]);
            translate([0, 15, 0]) circle(d=30);
            translate([85.5*2+4-30, 15, 0]) circle(d=30);
        }
}

zedCam();