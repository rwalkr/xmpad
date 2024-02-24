include <BOSL/constants.scad>
use <BOSL/shapes.scad>

$fn = 50;

U = 19;
K = 14;
KH = 6.5;

SB_D = 50;
SB_T = 6;
SB_H = 18;

PW = 4 + 3*U + 2 + 4;
PD = 4 + 2 * U + 1 + 3 + SB_D + 4;
PH = KH + 4;
PT = 2;

module stop_button () {
  translate([0, 0, SB_H])
  rotate([180, 0, 0])
  color("red") {
    difference() {
      union() {
        //cylinder(h = 6, d = SB_D);
        cyl(h = SB_T, d = SB_D, fillet=3, chamfer = 0, center=false);
        translate([0, 0, SB_T])
          cylinder(r = 17, h = SB_H-SB_T);
      }
      
      translate([-9, -9, SB_H-9])
        cube([19, 19, 9.1]);
    }
  }
}


module key_hole () {
  color("green", 0.1) {
  translate([-K/2, -K/2, -0.1 - 8 + PT]) {
    cube([K, K, 8.2]);
    translate([K/2-2, -1, 6])
      cube([4, K+2, 1.1]);
  }
  }
}

module key_cap (n) {
  color("yellow", 0.6)
  //translate([-n * U / 2, -U / 2, 0])
  prismoid(size1=[n * U, U], size2=[n * U - 4, U - 4], h=8);
//  cube([n * U, U, 8]);
}

module plate () {
    difference() {
      color("blue")
      union() {
        cuboid([PW, PD, 2], fillet=4, edges=EDGES_Z_ALL, center=false);
      }
      
      translate([0, 3 + 1 + 0.5 * U, 0]) {
        translate([4 + 0.75 * U, 0, 0])
          key_hole();
        translate([PW - 4 -  0.75 * U, 0, 0])
          key_hole();
      }
      translate([4, 3 + 1 + U + 1 + 0.5 * U, 0]) {
        translate([0.5 * U, 0, 0])
          key_hole();
        translate([U + 1 + 0.5 * U, 0, 0])
          key_hole();
        translate([2 * U + 2 + 0.5 * U, 0, 0])
          key_hole();
      }
      translate([PW/2, PD - 3 - SB_D/2, 0])
        key_hole();
    }
}

module feet () {
  color("yellow") {
    cube([PW, 3, PH]);
    translate([0, PD - 3, 0])
      difference() {
        cube([PW, 3, PH]);
        translate([PW/2 - 4, -0.1, -0.1])
          cube([8, 3.2, PH - KH + 0.1]);
      }
  }
}

module legs () {
  color("yellow") {
    translate([4, 4, 0])
       cylinder(h = PH, r = 4);
    translate([PW-4, 4, 0])
       cylinder(h = PH, r = 4);
    translate([4, PD-4, 0])
       cylinder(h = PH, r = 4);
    translate([PW-4, PD-4, 0])
       cylinder(h = PH, r = 4);
  }
}

module post () {
   translate([0, 0, PH - KH])
     difference() {
       cylinder(h = KH, d = 4);
       translate([0, 0, -0.1])
       cylinder(h = 3.1, d = 2);
     }
}

module posts () {
  translate([PW/2 - 5.7, PD - 3 - 2, 0])
    post();
  translate([PW/2 + 5.7, PD - 3 - 2, 0])
    post();
  translate([PW/2 - 5.7, PD - 3 - 48.6, 0])
    post();
  translate([PW/2 + 5.7, PD - 3 - 48.6, 0])
    post();
}

  
module pad () {
    translate([0, 0, -PT]) {
      plate();
      translate([0, 0, -PH]) {
        legs();
        posts();
      }
    }
}
  
module preview () {
  translate([PW/2, PD - 3 - SB_D/2, 5])
     stop_button();
  pad();
  
  translate([0, 3 + 1 + 0.5 * U, 2]) {
    translate([4 + 0.75 * U, 0, 0])
      key_cap(1.5);
    translate([PW - 4 -  0.75 * U, 0, 0])
      key_cap(1.5);
    }
  translate([4, 3 + 1 + U + 1 + 0.5 * U, 2]) {
    translate([0.5 * U, 0, 0])
      key_cap(1);
    translate([U + 1 + 0.5 * U, 0, 0])
      key_cap(1);
    translate([2 * U + 2 + 0.5 * U, 0, 0])
      key_cap(1);
  }

}


module print_button () {
  rotate([180, 0, 0])
  stop_button();
}

module print_pad() {
  rotate([180, 0, 0])
  pad();
}


print_pad();
//print_button();
//preview();