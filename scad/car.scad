$fn=100;

module wheel() {
    difference() {
        linear_extrude(67) circle(d=140, true);
        translate([0, 0, 25.25]) linear_extrude(67) circle(d=32, true);
        translate([0, 0, -1]) linear_extrude(26) circle(d=140-27*2);
        }
    translate([-27, -24.75, 0]) square([60, 54.5]);
}

module car() {
    
}

wheel();