# Blender Python Related

## Pip Install Modules

Blender bundled Python is located in this directory: `<blender_installation>\<version>\python\bin\python.exe`

e.g.

```bash
D:\Documents\Blender\3.1\python\bin\python.exe
```

```
D:\Documents\Blender\3.1\python\bin\python.exe -m pip install numpy
```

```bash
D:\Documents\Blender\3.1\python\bin\python.exe -m pip install pyserial
```

## Include external scripts









### Timed Update

```python
import sys
import os
import json
import logging
import threading

import bpy
import numpy as np
import serial

sys.path.append(os.path.join(bpy.path.abspath("//"), "scripts"))





class UARTTable:
    def __init__(self, port, baudrate=115200, logging_level=logging.INFO):
        self.port = port
        self.baudrate = 115200
        self.ser = None
        self.stop_sig = threading.Event()
        self.stop_sig.clear()

        self.data_table = {}

        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging_level)
        
        if not self.logger.handlers:
            ch = logging.StreamHandler()
            ch.setStream(sys.stdout)
            ch.setLevel(logging.DEBUG)
            # ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(name)s::%(threadName)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
            ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(name)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
            self.logger.addHandler(ch)
    
    def connect(self):
        self.logger.info("Connecting to serial port {0}...".format(self.port))
        # close any exising connection
        if self.ser:
            self.ser.close()
        while 1:
            # if main want to exit, we no longer try connect to port
            if self.stop_sig.is_set():
                return
            try:
                self.ser = serial.Serial(self.port, baudrate=self.baudrate)
            except serial.serialutil.SerialException as e:
                self.logger.error(str(e))
                self.logger.debug("Still trying...")
                time.sleep(0.5)
                continue
            
            # we only reach here if serial connection is established
            if self.ser.is_open:
                break

        self.logger.info("Connected.")
    
    def stop(self):
        self.stop_sig.set()
        if self.ser:
            self.ser.close()
        self.logger.info("Stopped.")
    
    def start(self):
        self.logger.debug("Starting...")
        self.recvPacket()
    
    def startThreaded(self):
        self.logger.debug("Starting thread...")
        self.t = threading.Thread(target=self.recvPacket)
        self.t.start()

    def get(self, key):
        return self.data_table.get(key)
    
    def recvPacket(self):
        while 1:
            if self.stop_sig.is_set():
                return
            
            c = b""
            buf = b""
            while c != b"\n":
                if self.stop_sig.is_set():
                    return
                buf += c
                # atomically handle serial object exceptions
                try:
                    c = self.ser.read()
                except (AttributeError, TypeError, serial.serialutil.SerialException) as e:
                    print(e)
                    self.connect()
                    continue

            data = buf.decode()
            # self.logger.debug(data)
            try:
                data = json.loads(data)
            except json.decoder.JSONDecodeError:
                self.logger.warning("packet format error.")
                continue
            key = data.get("key")
            if not key:
                self.logger.warning("packet format error.")
                continue
            
            self.data_table[key] = data.get("value")








MCU_COM_PORT = "COM22"
#FPS = 30        # use 30 if system performance is poor
FPS = 60


# set up logging
logger = logging.getLogger("BPY")
logger.setLevel(logging.DEBUG)

if not logger.handlers:
    ch = logging.StreamHandler()
    ch.setStream(sys.stdout)
    ch.setLevel(logging.DEBUG)
    # ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(name)s::%(threadName)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
    ch.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(name)s]: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
    logger.addHandler(ch)


# get scene armature object
#track_object = bpy.data.objects["teapot"]
track_object = bpy.data.objects["brush"]

# get the bones we need
#bone_root = armature.pose.bones.get("Root")
#bone_upper_arm_R = armature.pose.bones.get("J_Bip_R_UpperArm")
#bone_lower_arm_R = armature.pose.bones.get("J_Bip_R_LowerArm")

uart_table = UARTTable(MCU_COM_PORT, logging_level=logging.DEBUG)

"""
@param bone: the bone object, obtained from armature.pose.bone["<bone_name>"]
@param rotation: rotation in quaternion (w, x, y, z)
"""
def setBoneRotation(bone, rotation):
    w, x, y, z = rotation
    bone.rotation_quaternion[0] = w
    bone.rotation_quaternion[1] = x
    bone.rotation_quaternion[2] = y
    bone.rotation_quaternion[3] = z

"""
a piece of math code copied from StackOverflow
https://stackoverflow.com/questions/39000758/how-to-multiply-two-quaternions-by-python-or-numpy

@param q1: in form (w, x, y, z)
@param q0: @see q1
"""
def multiplyQuaternion(q1, q0):
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
                     
ypr_init = np.zeros((3))
vel_x = 0
vel_y = 0
vel_z = 0


class ModalTimerOperator(bpy.types.Operator):
    # we need these two fields for Blender
    bl_idname = "wm.modal_timer_operator"
    bl_label = "Modal Timer Operator"
    
    _timer = None
    
    def modal(self, context, event):
        global ypr_init
        global vel_x
        global vel_y
        global vel_z
        
        if event.type == "ESC":
            logger.info("BlenderTimer received ESC.")
            return self.cancel(context)
        
        if event.type == "R" and event.shift:
            try:
                ypr = uart_table.get("/teapot")[3:6]
                ypr = np.array(ypr)
                
                ypr = ypr / 180 * np.pi
                ypr_init = ypr
            except Exception as e:
                print(e)
            logger.info("Resetting position:")
            return {"PASS_THROUGH"}
            
        if event.type == "TIMER":
            # this will eval true every Timer delay seconds
            data = uart_table.get("/teapot")
            
            
            if not data:
                logger.warning("Invalid joint data")
                return {"PASS_THROUGH"}
            
            # convert data to numpy arrays
            acc = np.array(data[3:6])
            ypr = np.array(data[3:6])
            
            ypr = ypr / 180 * np.pi
            
            ypr -= ypr_init
            
            dt = 1./FPS
            
#            logger.info(ypr)
            vel_x = acc[0] * dt
            vel_y = acc[1] * dt
            vel_z = acc[2] * dt
            
            track_object.location[0] += vel_x * dt
            track_object.location[1] += vel_y * dt
            track_object.location[2] += vel_z * dt
            
            track_object.rotation_euler[0] = -ypr[2]
            track_object.rotation_euler[1] = -ypr[1]
            track_object.rotation_euler[2] = ypr[0]
            
#            logger.debug(str(q0) + str(q1))
#            
#            # get inverse transformation of q0, the parent bone
#            q0_inv = q0 * np.array([1, -1, -1, -1])
#            
#            # rotate child about parent, to get relative position of the child
#            q1_rel = multiplyQuaternion(q0_inv, q1)
#            
#            # apply transformation
#            setBoneRotation(bone_upper_arm_R, q0)
#            setBoneRotation(bone_lower_arm_R, q1_rel)

            # if refresh rate is too low, uncomment this line to force Blender to render viewport
#            bpy.ops.wm.redraw_timer(type="DRAW_WIN_SWAP", iterations=1)
    
        return {"PASS_THROUGH"}

    def execute(self, context):
        # update rate is 0.01 second
        self._timer = context.window_manager.event_timer_add(1./FPS, window=context.window)
        context.window_manager.modal_handler_add(self)
        return {"RUNNING_MODAL"}
        
    def cancel(self, context):
        uart_table.stop()
        
        # reset joint position
        track_object.location = (0, 0, 0)
        track_object.rotation_euler = (0, 0, 0)
#        setBoneRotation(bone_upper_arm_R, [1, 0, 0, 0])
#        setBoneRotation(bone_lower_arm_R, [1, 0, 0, 0])
        
        context.window_manager.event_timer_remove(self._timer)
        logger.info("BlenderTimer Stopped.")
        return {"CANCELLED"}


if __name__ == "__main__":
    try:
        logger.info("Starting services.")
        bpy.utils.register_class(ModalTimerOperator)
        
        # uart_table.start()
        uart_table.startThreaded()
        
        # start Blender timer
        bpy.ops.wm.modal_timer_operator()
        
        logger.info("All started.")
    except KeyboardInterrupt:
        uart_table.stop()
        logger.info("Received KeyboardInterrupt, stopped.")

```
