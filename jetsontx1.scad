$fn=100;
module jetsonTX1() {
    color("darkgreen") {  // board
        linear_extrude(1.75) {
            difference() {
                translate([-6.35, -5.08,0]) {
                    square([163.83+6.35, 165.1 +5.08], false);
                }
                circle(d=3.95);
                translate([157.48, 0, 0]) {
                    circle(d=3.95);
                }
                translate([157.48, 132.08, 0]) {
                    circle(d=3.95);
                }
                translate([0, 165.1-9.8,0]) {
                    circle(d=3.95);
                }
            }
        }
    }
    translate([0, 0, 1.75]) { // everything is on top of the board
        color("silver") {  // compute module
            linear_extrude(37) {
                translate([29, 47.68, 0]) {
                    translate([-3.75, -3.5, 0]) {
                        square([51, 87], false);
                    }
                }
            }
        }
        
        color("black") {  // power connector
            translate([163.83-3.75-10.25, 165.1-13.25,0]) {
                linear_extrude(10) {
                    square([10.25,13.25], false);
                }
            }
        }
        
        color("red") { // micro usb port
            translate([163.83-3.75-10.25-32.2-8, 165.1-6,0]) {
                linear_extrude(3) {
                    square([8, 6], false);
                }
            }
        }
        
        color("blue") { // usb a port
            translate([163.83-3.75-10.25-32.2-8-6.8-14, 165.1-16.5, 0]) {
                linear_extrude(7) {
                    square([14, 16.5], false);
                }
            }
        }
        
        color("hotpink") { // hdmi port
            translate([163.83-3.75-10.25-32.2-8-6.8-14-7.25-14.5, 165.1-18.5, 0]) {
                linear_extrude(6.1) {
                    square([14.5, 18.5], false);
                }
            }
        }
        
        color("gold") { // sd card slot
            translate([163.83-3.75-10.25-32.2-8-6.8-14-7.25-14.5-28-3, 165.1-35+3, 0]) {
                linear_extrude(2.5) {
                    square([28, 35], false);
                }
            }
        }
    }
}

jetsonTX1();