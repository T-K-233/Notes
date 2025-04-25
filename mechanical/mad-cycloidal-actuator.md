# MAD Cycloidal Actuator

## New Method (Updated 2023-09-30)

```python
# generate_cycloidal_profile.py
import numpy as np
import matplotlib.pyplot as plt
import ezdxf


def generateCycloidalGearProfile(r_housing=36, r_roller=4, N=16, eccentricity=2):
    theta = np.arange(0, 2*np.pi, 0.001)

    psi = - np.arctan(np.sin((1 - N) * theta) / (r_housing / (eccentricity * N) - np.cos((1 - N)*theta)))

    x = r_housing * np.cos(theta) - r_roller * np.cos(theta - psi) - eccentricity * np.cos(N * theta)
    y = -r_housing * np.sin(theta) + r_roller * np.sin(theta - psi) + eccentricity * np.sin(N * theta)

    return np.concatenate(([x], [y]), axis=0)


def writeDXF(filename, fit_points, ref_circle_radius=10):
    doc = ezdxf.new("R2010") # create a new DXF drawing in R2010 fromat 
    msp = doc.modelspace()

    fit_points = fit_points.T.tolist()

    start_point = fit_points[0]

    # close the loop
    fit_points.append(start_point)

    spline = msp.add_spline(fit_points)

    spline.set_closed([(start_point[0], start_point[1], 0)])

    print(spline.dxf.n_fit_points)

    circle = msp.add_circle((0, 0), radius=ref_circle_radius)
    
    doc.saveas(filename)

# 5010

fit_points = generateCycloidalGearProfile(
    r_housing=29.,
    r_roller=2.5,
    N=16.,
    eccentricity=1.25
    )
'''
#fit_points_ = generateCycloidalGearProfile(
#    r_housing=29.,
#    r_roller=3.,
#    N=16.,
#    eccentricity=1.25
#    )

'''
# M6C12
'''
fit_points = generateCycloidalGearProfile(
    r_housing=36,
    r_roller=4,
    N=16,
    eccentricity=2
    )
'''
print(fit_points.shape)

x, y = fit_points
#x_, y_ = fit_points_
plt.plot(x, y)
#plt.plot(x_, y_)
plt.show()

writeDXF("cycloidal_profile.dxf", fit_points, ref_circle_radius=31)

```



### RI-60 Parameters

<table><thead><tr><th width="340.3333333333333"></th><th></th><th></th></tr></thead><tbody><tr><td>Housing Radius</td><td>29</td><td>mm</td></tr><tr><td>Housing Roller Diameter</td><td>5</td><td>mm</td></tr><tr><td>Central Hole Diameter</td><td>18</td><td>mm</td></tr><tr><td>Side Hole Diameter</td><td>13</td><td>mm</td></tr><tr><td>Eccentricity</td><td>1.25</td><td>mm</td></tr><tr><td>Spacing Radius of Side Hole</td><td>17</td><td>mm</td></tr></tbody></table>

### RI-70 Parameters

<table><thead><tr><th width="340.3333333333333"></th><th></th><th></th></tr></thead><tbody><tr><td>Housing Radius</td><td>36</td><td>mm</td></tr><tr><td>Housing Roller Diameter</td><td>8</td><td>mm</td></tr><tr><td>Central Hole Diameter</td><td>18</td><td>mm</td></tr><tr><td>Side Hole Diameter</td><td>13</td><td>mm</td></tr><tr><td>Eccentricity</td><td>2</td><td>mm</td></tr><tr><td>Spacing Radius of Side Hole</td><td>20</td><td>mm</td></tr></tbody></table>





## Old Method

Go in an active sketch plane

![](<../.gitbook/assets/image (8) (1) (1) (1).png>)

Run the following script:

```python

import math

import adsk.core, adsk.fusion, traceback


def generateCycloidalOutline(n_planet_teeth, n_housing_teeth, e, pin_to_center_distance):
    intersectsItself = False
    lastMaxAngle = -math.pi / 2
    
    origin = adsk.core.Point3D.create(0, 0, 0)
    halfAngle = 2 * math.pi / n_planet_teeth / 2
    N = 101

    points = adsk.core.ObjectCollection.create()

    for ip in range(N + 1):
        angle = (2 * math.pi / n_planet_teeth) / N * ip
        x = pin_to_center_distance * math.cos(angle) + e * math.cos(n_housing_teeth * angle)# - e
        y = pin_to_center_distance * math.sin(angle) + e * math.sin(n_housing_teeth * angle)

        p = adsk.core.Point3D.create(x, y, 0)
        points.add(p)

        #currentAngle = Angle.getToPoint(origin, p)
        currentAngle = math.atan2(p.y - origin.y, p.x - origin.x)

        # if not intersectsItself and currentAngle < lastMaxAngle:
        #     intersectsItself = True

        # if intersectsItself and currentAngle <= halfAngle:
        #     break

        lastMaxAngle = currentAngle

    # if intersectsItself:
    #     upperPoints = [adsk.core.Point3D.create(p.x, -p.y, 0) for p in points]
    #     upperPoints = [getRotated(p, Matrix.getForRotation(2 * halfAngle, origin)) for p in upperPoints]
    #     return [points, upperPoints]

    return points


def run(context):
    '''
    ## Etch M6C12 params
    reduction_ratio = 15
    eccentricity = 2
    pin_diameter = 8
    housing_inner_diameter = 72
    '''
    
    ## Etch 5010 params
    reduction_ratio = 15
    eccentricity = 1.5
    pin_diameter = 5
    housing_inner_diameter = 56




    n_planet_teeth = reduction_ratio
    n_housing_teeth = reduction_ratio + 1

    
    eccentricity *= 0.1
    pin_diameter *= 0.1
    housing_inner_diameter *= 0.1

    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('No active Fusion 360 design', 'No Design')
            return
        rootComp = design.rootComponent
        
        sketch = app.activeEditObject
        

        origin_point = adsk.core.Point3D.create(0, 0, 0)

        points = generateCycloidalOutline(
            n_planet_teeth=n_planet_teeth,
            n_housing_teeth=n_housing_teeth,
            e=eccentricity,
            pin_to_center_distance=housing_inner_diameter/2
            )

        cycloidSpline = sketch.sketchCurves.sketchFittedSplines.add(points)
        
        
        cycloidSplinecoll = adsk.core.ObjectCollection.create()
        
        cycloidSplinecoll.add(cycloidSpline)


        offsetSpline = sketch.offset(cycloidSplinecoll, adsk.core.Point3D.create(), pin_diameter / 2).item(0)
        cycloidSpline.deleteMe()
        
        #endSplinePoint = getRotated(offsetSpline.startSketchPoint.geometry, getForRotation(math.tau / n_planet_teeth, origin_point))
        p = offsetSpline.startSketchPoint.geometry.copy()

        mat = adsk.core.Matrix3D.create()

        mat.setToRotation(math.tau / n_planet_teeth, adsk.core.Vector3D.create(0, 0, 0.5), origin_point)

        p.transformBy(mat)

        endSplinePoint = p
        circularFeatures = [offsetSpline]

        # If profile will be probably open, make more accurate segment
        if not endSplinePoint.isEqualToByTolerance(offsetSpline.endSketchPoint.geometry, 1e-8):
            #line = Sketch.Draw.lines(sketch, [endSplinePoint, offsetSpline.endSketchPoint.geometry])[0]
            points = [endSplinePoint, offsetSpline.endSketchPoint.geometry]
            asConstruction = False
            close = False
            lines = []

            for ip in range(1, len(points)):
                line = sketch.sketchCurves.sketchLines.addByTwoPoints(points[ip - 1], points[ip])
                line.isConstruction = asConstruction
                lines.append(line)

            if close:
                line = sketch.sketchCurves.sketchLines.addByTwoPoints(points[len(points) - 1], points[0])
                line.isConstruction = asConstruction
                lines.append(line)

            circularFeatures.append(lines[0])
            

        #Sketch.Pattern.circular(sketch, circularFeatures, n_planet_teeth, origin=origin_point)   
        sketchObjectOrObjects = circularFeatures
        quantity = n_planet_teeth
        angle = 2 * math.pi
        origin = origin_point

        objects = adsk.core.ObjectCollection.create()
        for o in sketchObjectOrObjects:
            objects.add(o)

        #objects = getCollection(sketchObjectOrObjects)

        for step in range(quantity - 1):
            stepAngle = angle / quantity * (step + 1)
            
            transformMatrix = adsk.core.Matrix3D.create()

            transformMatrix.setToRotation(stepAngle, adsk.core.Vector3D.create(0, 0, 0.5), origin)
            sketch.copy(objects, transformMatrix, sketch)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

```

Then we get the cycloidal gear profile

![](<../.gitbook/assets/image (103).png>)

![](<../.gitbook/assets/image (144).png>)

```python
Pins and housing
════════════════
  rr     28.0000 mm
  dr      5.0000 mm
  zr          16

Planets(s)
══════════
  dc      8.0000 mm
  ds     13.0000 mm
   e      1.5000 mm
  df     48.0000 mm
  da     54.0000 mm
  tp      3.5000 mm
   N           2
  ap      0.5000 mm

Input
═════
  dc      8.0000 mm
   e      1.5000 mm
  tp      3.5000 mm
   N           2
  ap      0.5000 mm

Output
══════
  rb     16.0000 mm
  db     10.0000 mm
  zs           6
  to      1.0000 mm
  ao      0.5000 mm


```

To adapt to the motor we are using, we need to edit the model a bit.

{% embed url="https://www.amazon.com/dp/B082W5B5LZ?psc=1&ref=ppx_yo2ov_dt_b_product_details" %}

<div align="center"><img src="../.gitbook/assets/image (25) (1).png" alt=""></div>

![](<../.gitbook/assets/image (100).png>)
