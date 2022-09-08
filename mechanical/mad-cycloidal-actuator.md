# MAD Cycloidal Actuator

Go in an active sketch plane

![](<../.gitbook/assets/image (8).png>)

Run the following script:

```

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

```
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



![](<../.gitbook/assets/image (25).png>)

![](<../.gitbook/assets/image (100).png>)



