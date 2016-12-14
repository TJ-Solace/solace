$fn=100;

module wheel() {
    difference() {
        linear_extrude(67) circle(d=140, true);
        translate([0, 0, 25.25]) linear_extrude(67) circle(d=32, true);
        translate([0, 0, -1]) linear_extrude(26) circle(d=140-27*2);
        }
}

module axle() {
    translate([0, 0, 52]) linear_extrude(52) square([50, 285], true);
    color([0.15, 0.15, 0.15]) translate([0, -285/2, 140/2]) rotate([90, 0, 0]) wheel();
    color([0.15, 0.15, 0.15]) translate([0, 285/2, 140/2]) rotate([-90, 0, 0]) wheel();
}

module car() {
    translate([310/2, 150/2, -59-4]) {
        translate([0, 0, 59]) linear_extrude(4) square([310, 150], true);
        translate([-70/2 - 310/2, 0, 0]) axle();
        translate([70/2 + 310/2, 0, 0]) axle();
    }
    color("red") {
        translate([0, 8, 0]) {
            //linear_extrude(10) square([160, 133]);
            cube([45, 5, 33]);
            translate([115, 0, 0]) cube([45, 5, 33]);
            translate([115, 150-16-5, 0]) cube([45, 5, 33]);
            translate([0, 150-16-5, 0]) cube([45, 5, 33]);
        }
            translate([0, 150/2-25/2, 0]) cube([139.5, 25, 42.5]);
            translate([10-37, 150/2-55.5/2, 0]) cube([37, 55.5, 42.5]);
        
        translate([4, 9, 0]) {
            cube([5, 150-20, 8.35]);
            translate([115+45-4-5-5, 0, 0]) cube([5, 150-20, 8.35]);
        }
    }
}

car();