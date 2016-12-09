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
    translate([0, -285/2, 140/2]) rotate([90, 0, 0]) wheel();
    translate([0, 285/2, 140/2]) rotate([-90, 0, 0]) wheel();
}

module car() {
    translate([310/2, 150/2, -59-4]) {
        translate([0, 0, 59]) linear_extrude(4) square([310, 150], true);
        translate([-70/2 - 310/2, 0, 0]) axle();
        translate([70/2 + 310/2, 0, 0]) axle();
    }
    translate([4, 8, 0]) {
        square([, 133]);
    }
}

car();