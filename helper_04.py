import os
import sys
import numpy as np
import cv2
import dobotapi
import time
import math
# ============================================================
# Hikrobot MVS SDK setup
# ============================================================

MVS_PYTHON_PATH = (
    r"C:\Program Files (x86)\MVS"
    r"\Development\Samples\Python\MvImport"
)

MVS_DLL_PATH = (
    r"C:\Program Files (x86)\Common Files\MVS"
    r"\Runtime\Win64_x64"
)

sys.path.insert(0, MVS_PYTHON_PATH)
os.add_dll_directory(MVS_DLL_PATH)

from MvCameraControl_class import *

def move_and_wait(robot, x, y, z, tolerance=1.0, timeout=20):
    print(f"Moving to ({x}, {y}, {z})", flush=True)

    robot.move_to(x, y, z)

    start = time.perf_counter()

    while True: 
        
       
        # --------------------------------
        # Position
        # --------------------------------
        pose = robot.get_pose()

        current_x = pose.position.x
        current_y = pose.position.y
        current_z = pose.position.z

        distance = math.sqrt(
            (current_x - x) ** 2 +
            (current_y - y) ** 2 +
            (current_z - z) ** 2
        )

        if distance <= tolerance:
            print("TARGET REACHED", flush=True)
            return

        if time.perf_counter() - start > timeout:
            raise TimeoutError(
                f"Robot failed to reach "
                f"({x}, {y}, {z}) within {timeout}s"
            )

        time.sleep(0.05)
        
def read_calib_file(filename = "calibration.txt"):

    
    image_points = []
    world_points = []
    
    with open(filename, "r") as file:
    
        for line in file:
    
            # Ignore empty lines
            if not line.strip():
                continue
    
            values = line.split()
    
            # Expected:
            # image_x image_y world_X world_Y world_Z
            if len(values) < 5:
                continue
    
            image_x = float(values[0])
            image_y = float(values[1])
    
            world_x = float(values[2])
            world_y = float(values[3])
    
            image_points.append([image_x, image_y])
            world_points.append([world_x, world_y, -43])
    
    
    # Convert to NumPy arrays
    image_points = np.array(image_points, dtype=np.float32)
    world_points = np.array(world_points, dtype=np.float32)


    print("Image points:")
    print(image_points)
    
    print("\nWorld points:")
    print(world_points)
    
    return image_points, world_points


class HikrobotCamera:

    def __init__(self, camera_index=0, timeout_ms=5000):

        self.camera_index = camera_index
        self.timeout_ms = timeout_ms
        self.camera = None

        self._open()

    def _check_ret(self, ret, name):

        if ret != 0:
            raise RuntimeError(
                f"{name} failed: 0x{ret:08X}"
            )

    def _open(self):

        # ----------------------------------------------------
        # Enumerate USB cameras
        # ----------------------------------------------------

        device_list = MV_CC_DEVICE_INFO_LIST()

        ret = MvCamera.MV_CC_EnumDevices(
            MV_USB_DEVICE,
            device_list
        )

        self._check_ret(
            ret,
            "MV_CC_EnumDevices"
        )

        if device_list.nDeviceNum == 0:
            raise RuntimeError(
                "No Hikrobot USB camera found."
            )

        if self.camera_index >= device_list.nDeviceNum:
            raise RuntimeError(
                f"Invalid camera index {self.camera_index}. "
                f"Found {device_list.nDeviceNum} camera(s)."
            )

        print(
            f"Found {device_list.nDeviceNum} "
            f"Hikrobot camera(s)."
        )

        # ----------------------------------------------------
        # Select camera
        # ----------------------------------------------------

        device_info = cast(
            device_list.pDeviceInfo[self.camera_index],
            POINTER(MV_CC_DEVICE_INFO)
        ).contents

        # ----------------------------------------------------
        # Create camera handle
        # ----------------------------------------------------

        self.camera = MvCamera()

        ret = self.camera.MV_CC_CreateHandle(
            device_info
        )

        self._check_ret(
            ret,
            "MV_CC_CreateHandle"
        )

        # ----------------------------------------------------
        # Open camera
        # ----------------------------------------------------

        ret = self.camera.MV_CC_OpenDevice(
            MV_ACCESS_Exclusive,
            0
        )

        self._check_ret(
            ret,
            "MV_CC_OpenDevice"
        )

        # ----------------------------------------------------
        # Continuous acquisition
        # ----------------------------------------------------

        ret = self.camera.MV_CC_SetEnumValue(
            "TriggerMode",
            MV_TRIGGER_MODE_OFF
        )

        self._check_ret(
            ret,
            "Set TriggerMode"
        )

        # ----------------------------------------------------
        # Start grabbing
        # ----------------------------------------------------

        ret = self.camera.MV_CC_StartGrabbing()

        self._check_ret(
            ret,
            "MV_CC_StartGrabbing"
        )

        print("Camera ready.")

    def read(self):

        """
        Grab one frame.

        Returns
        -------
        numpy.ndarray

            The image returned by the camera.

            For a Bayer camera this is currently the
            raw Bayer image.
        """

        # ----------------------------------------------------
        # Frame information structure
        # ----------------------------------------------------

        frame_info = MV_FRAME_OUT_INFO_EX()

        # ----------------------------------------------------
        # Allocate buffer
        #
        # MV-CE050-30UC:
        # 2592 x 1944
        #
        # Allocate enough for 3 bytes/pixel.
        # ----------------------------------------------------

        buffer_size = 2592 * 1944 * 3

        image_buffer = (c_ubyte * buffer_size)()

        # ----------------------------------------------------
        # Get frame
        # ----------------------------------------------------

        ret = self.camera.MV_CC_GetOneFrameTimeout(
            image_buffer,
            buffer_size,
            frame_info,
            self.timeout_ms
        )

        self._check_ret(
            ret,
            "MV_CC_GetOneFrameTimeout"
        )

        width = frame_info.nWidth
        height = frame_info.nHeight
        frame_length = frame_info.nFrameLen

        print(
            f"Received frame: "
            f"{width} x {height}, "
            f"{frame_length} bytes"
        )

        # ----------------------------------------------------
        # Copy native buffer into NumPy
        # ----------------------------------------------------

        image = np.frombuffer(
            image_buffer,
            dtype=np.uint8,
            count=frame_length
        ).copy()

        # ----------------------------------------------------
        # Determine image format
        # ----------------------------------------------------

        pixel_type = frame_info.enPixelType

        print(
            f"Pixel type: {pixel_type}"
        )

        # ----------------------------------------------------
        # Mono8
        # ----------------------------------------------------

        if pixel_type == PixelType_Gvsp_Mono8:

            image = image.reshape(
                height,
                width
            )

            return image

        # ----------------------------------------------------
        # Bayer image
        #
        # We will determine the exact Bayer pattern from
        # the pixel type.
        # ----------------------------------------------------

        if pixel_type == PixelType_Gvsp_BayerRG8:

            image = image.reshape(
                height,
                width
            )

            return cv2.cvtColor(
                image,
                cv2.COLOR_BAYER_RG2BGR
            )

        elif pixel_type == PixelType_Gvsp_BayerGB8:

            image = image.reshape(
                height,
                width
            )

            return cv2.cvtColor(
                image,
                cv2.COLOR_BAYER_GB2BGR
            )

        elif pixel_type == PixelType_Gvsp_BayerGR8:

            image = image.reshape(
                height,
                width
            )

            return cv2.cvtColor(
                image,
                cv2.COLOR_BAYER_GR2BGR
            )

        elif pixel_type == PixelType_Gvsp_BayerBG8:

            image = image.reshape(
                height,
                width
            )

            return cv2.cvtColor(
                image,
                cv2.COLOR_BAYER_BG2BGR
            )

        # ----------------------------------------------------
        # Unknown format
        # ----------------------------------------------------

        raise RuntimeError(
            f"Unsupported pixel type: {pixel_type}"
        )

    def close(self):

        if self.camera is None:
            return

        try:

            self.camera.MV_CC_StopGrabbing()

        finally:

            try:

                self.camera.MV_CC_CloseDevice()

            finally:

                self.camera.MV_CC_DestroyHandle()

                self.camera = None

        print("Camera closed.")

    def __enter__(self):

        return self

    def __exit__(
        self,
        exc_type,
        exc_value,
        traceback
    ):

        self.close()
