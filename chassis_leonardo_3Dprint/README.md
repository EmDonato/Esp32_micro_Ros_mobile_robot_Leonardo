# Chassis Components Directory

This directory contains all the 3D model files (`.stl` and `.f3d`) for the chassis components of the mobile robot platform. Use these files to assemble, simulate, or 3D-print the structural parts of the robot.

---

## Folder Structure

* **Screw\&Nut\_1/**

  * Contains M4 screws and nuts (package 1) used for fastening modular plates and support brackets.

* **Screw\&Nut\_2/**

  * Contains an additional set of M4 screws and nuts (package 2) for supplemental assembly needs.

* **basechassisLeonardo/**

  * Modular platform plates. Two of these plates combine to form the complete chassis base. Each plate shares the same hole pattern for M4 fasteners, ensuring interchangeable assembly.

* **reggiL298N/**

  * Support bracket designed to mount the L298N motor driver securely to the chassis.

* **wire\_linear\_bus/**

  * Cable guide structure for routing wiring along the linear bus of the chassis.

* **holder\_DCDC/**

  * Mounting bracket for the DC–DC converter module. Ensures stable positioning and clearance.

* **battery/**

  * Battery compartment housing. Designed to fit the standard Li‑ion battery pack, with snap‑fit features for quick insertion/removal.

* **jointer/**

  * Snap‑fit connector piece that links two modular base plates. Enables quick assembly without additional hardware.

* **jointer\_level/**

  * Strap‑like clamps that secure stacked base modules at different levels, allowing multi‑tiered chassis configurations.

---

## Usage Notes

* All assemblies use M4 hardware; verify screw lengths to avoid interference.
* `.f3d` files are native Fusion 360 designs—open them to inspect sketches, constraints, and assemblies.
* `.stl` exports are ready for slicing and 3D printing; check tolerances if fitting snap‑fits.
* Maintain consistent hole alignment when stacking modules with `jointer_level` clamps.

For questions or contributions, please open an issue in the repository.
