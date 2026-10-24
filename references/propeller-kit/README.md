# Motor and propeller reference STEP files

These three files were copied from the user-provided
`Tailsitter_XING2_1404_Propeller_CAD_Kit.zip` for use by
[`motor_prop_installation.py`](../../motor_prop_installation.py). The source
archive was left unchanged.

- `iFlight_XING2_1404_4600KV_REFERENCE.step`: motor packaging reference.
- `PROP_3x4_2B_CW.step`: clockwise 3x4 propeller packaging reference used by
  the example assembly.
- `PROP_3x4_2B_CCW.step`: counterclockwise counterpart retained for future
  packaging checks.

The propeller geometry is generic clearance geometry. It does not establish
thrust, efficiency, structural strength, or flight suitability. The STEP
motor shaft extension and the 18.5 mm mount-face-to-hub-center placement are
packaging assumptions; verify them against the physical motor and propeller
before manufacturing or flight use.
