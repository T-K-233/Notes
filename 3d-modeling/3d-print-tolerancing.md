# 3D Print Tolerancing and Printer Profile

## Tolerancing

The tolerancing of the part is controlled by the `TOLERANCE_OFFSET` variable in the CAD model.

<table><thead><tr><th>Desired Fit</th><th>Clearance Gap (mm)</th><th width="135">Multiplier</th></tr></thead><tbody><tr><td>Press Fit</td><td>Line to line (0)</td><td>0 x</td></tr><tr><td>Tight Fit</td><td>0.1</td><td>1 x</td></tr><tr><td>Normal Fit</td><td>0.2</td><td>2 x</td></tr><tr><td>Loss Fit</td><td>0.3</td><td>3 x</td></tr></tbody></table>

Tight fit is commonly used in bearing interfaces.

## Printer Profile

Changes from Standard profile:

Wall Line Count: 2 -> 3, increase strength and shape

Outer Wall Wipe Distance: 0.0 -> 0.1 mm, hide z seam

Printing temperature: 200 -> 210 C

Combing mode: All

Support Overhang Angle: 65 deg

Support density: 20% -> 10%

Support X/Y distance: 0.8 -> 1.5 mm, avoid generating support in horizontal M3 holes

Build plate

Skirt line count: 3 -> 2, save filament, faster printing

## Reference

{% embed url="https://forum.freecad.org/viewtopic.php?t=28602" %}

{% embed url="https://all3dp.com/2/3d-printing-tolerances-test-fdm/" %}

{% embed url="https://3dchimera.com/blogs/connecting-the-dots/3d-printing-tolerances-fits" %}
