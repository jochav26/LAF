import amcam
import cv2
import numpy as np
import serial  # For serial communication with Arduino
import time

def runRegistration(currImgArray, prevImgArray):
    if prevImgArray is None:
        return 0, 0
    gray1 = np.float32(cv2.cvtColor(currImgArray, cv2.COLOR_BGR2GRAY))
    gray2 = np.float32(cv2.cvtColor(prevImgArray, cv2.COLOR_BGR2GRAY))
    (dx, dy), _ = cv2.phaseCorrelate(gray1, gray2)
    height, width = currImgArray.shape[:2]
    num_pixels_x = dx * width
    num_pixels_y = dy * height
    print("Number of pixels mapped in X direction:", num_pixels_x)
    print("Number of pixels mapped in Y direction:", num_pixels_y)
    return num_pixels_x, num_pixels_y

def filterBlueAndWhiteColors(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])
    blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 50, 255])
    white_mask = cv2.inRange(hsv, lower_white, upper_white)
    combined_mask = cv2.bitwise_or(blue_mask, white_mask)
    result_img = cv2.bitwise_and(img, img, mask=combined_mask)
    return result_img, blue_mask, white_mask

class App:
    def __init__(self):
        self.hcam = None
        self.buf = None
        self.total = 0                            
        self.prevImg = None
        self.width = None
        self.height = None
        self.arduino = None

    @staticmethod
    def cameraCallback(nEvent, ctx):
        if nEvent == amcam.AMCAM_EVENT_IMAGE:
            ctx.CameraCallback(nEvent)
    def CameraCallback(self, nEvent):
        if nEvent == amcam.AMCAM_EVENT_IMAGE:
            try:
                self.hcam.PullImageV2(self.buf, 24, None)
                self.total += 1
                print('pull image ok, total = {}'.format(self.total))
                img = np.frombuffer(self.buf, dtype=np.uint8).reshape(self.height, self.width, 3)
                filtered_img, blue_mask, white_mask = filterBlueAndWhiteColors(img)
            
                # Convert the filtered image to grayscale for brightness calculation
                gray_filtered_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)

                # Calculate the average brightness of the image
                avg_brightness = np.mean(gray_filtered_img)  # Average brightness value
                print(f"Average Brightness: {avg_brightness:.2f}")


                # Find the location of the brightest point
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray_filtered_img)
                brightest_point = max_loc

                # Draw the green dot at the brightest point
                cv2.circle(filtered_img, brightest_point, 5, (0, 255, 0), -1)  # Green dot
                cv2.putText(filtered_img, "X: {}   Y: {} ".format(brightest_point[0], brightest_point[1]),
                            (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
                
                # Display the brightness value on the image
                cv2.putText(filtered_img, "Brightness: {:.2f}".format(avg_brightness),
                            (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
                
                
                # Control the laser based on the position of the brightest point
                command = "STOP"
                if brightest_point[0] > 582:  # Laser unfocused to the positive direction
                    cv2.putText(filtered_img, "Laser Unfocused (Positive)", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    command = "UP 100 150"
                elif brightest_point[0] < 560:  # Laser unfocused to the negative direction
                    cv2.putText(filtered_img, "Laser Unfocused (Negative)", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
                    command = "DOWN 100 150"
            
                if command in {"DOWN 100 150", "UP 100 150", "STOP"}:
                    self.arduino.write((command + "\r\n").encode())
                    
                # Show processed image
                cv2.imshow('Filtered Image (Blue + White with Centroid)', filtered_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):  # Quit safely
                    cv2.destroyAllWindows()
                    exit()
    
                # Dump frame memory
                self.buf[:] = bytearray(len(self.buf))  # Reset buffer
                del img, filtered_img, blue_mask, white_mask  # Free variables
                cv2.waitKey(1)  # Force OpenCV memory cleanup
                
            
                # Resize and show the image
                max_width = 800
                max_height = 600
                height, width = filtered_img.shape[:2]
                scale = min(max_width / width, max_height / height)
                resized_img = cv2.resize(filtered_img, (int(width * scale), int(height * scale)))
                cv2.imshow('Filtered Image (Blue + White with Centroid)', resized_img)
                cv2.waitKey(1)
               
                
                # # Optionally run registration function
                # px, py = runRegistration(resized_img, self.prevImg)
                # self.prevImg = resized_img

            except amcam.HRESULTException as ex:
                print('pull image failed, hr=0x{:x}'.format(ex.hr))
                
    # def CameraCallback(self, nEvent):
    #     if nEvent == amcam.AMCAM_EVENT_IMAGE:
    #         try:
    #             self.hcam.PullImageV2(self.buf, 24, None)
    #             self.total += 1
    #             print('pull image ok, total = {}'.format(self.total))
    #             img = np.frombuffer(self.buf, dtype=np.uint8).reshape(self.height, self.width, 3)
    #             filtered_img, blue_mask, white_mask = filterBlueAndWhiteColors(img)
    #             moments = cv2.moments(blue_mask)
                
    #             if self.arduino and moments["m00"] != 0:
    #                 cX = int(moments["m10"] / moments["m00"])
    #                 cY = int(moments["m01"] / moments["m00"])
    #                 cv2.circle(filtered_img, (cX, cY), 5, (0, 255, 0), -1)  # Green dot
    #                 cv2.putText(filtered_img, "X: {}   Y: {} ".format(cX, cY), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    #                 command = "stop"
    #                 if cX > 582:
    #                     cv2.putText(filtered_img, "Laser Unfocused (Positive)", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    #                     command = "reverse"
    #                 elif cX < 560:
    #                     cv2.putText(filtered_img, "Laser Unfocused (Negative)", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    #                     command = "forward"
    #                 if command in {"forward", "reverse", "stop"}:
    #                     self.arduino.write((command + "\r\n").encode())

    #             max_width = 800
    #             max_height = 600
    #             height, width = filtered_img.shape[:2]
    #             scale = min(max_width / width, max_height / height)
    #             resized_img = cv2.resize(filtered_img, (int(width * scale), int(height * scale)))
    #             cv2.imshow('Filtered Image (Blue + White with Centroid)', resized_img)
    #             cv2.waitKey(1)
    #             px, py = runRegistration(resized_img, self.prevImg)
    #             self.prevImg = resized_img
    #         except amcam.HRESULTException as ex:
    #             print('pull image failed, hr=0x{:x}'.format(ex.hr))

    def open_arduino(self, port="COM12"):
        try:
            self.arduino = serial.Serial(port, 9600, timeout=1)
            print(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            print(f"Error: Could not open port {port}. {e}")
            return False
        return True

    def close_arduino(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
            print("Arduino connection closed")

    def run(self):
        a = amcam.Amcam.EnumV2()
        if len(a) > 0:
            print(f'{a[0].displayname}: flag = {a[0].model.flag}, preview = {a[0].model.preview}, still = {a[0].model.still}')
            for r in a[0].model.res:
                print(f'\t = [{r.width} x {r.height}]')
            self.hcam = amcam.Amcam.Open(a[0].id)
            if self.hcam:
                try:
                    self.hcam.put_Size(1228, 922)
                    self.hcam.put_ExpoTime(100)
                    framerate = self.hcam.get_FrameRate()
                    timeexprange = self.hcam.get_ExpTimeRange()
                    gainexp = self.hcam.get_ExpoAGain()
                    gainexprange = self.hcam.get_ExpoAGainRange()
                    timeexp = self.hcam.get_ExpoTime()
                    width, height = self.hcam.get_Size()
                    self.width = width
                    self.height = height
                    bufsize = ((width * 24 + 31) // 32 * 4) * height
                    print('Image size: {} x {}, bufsize = {}'.format(width, height, bufsize))
                    print('Framerate: {}'.format(framerate))
                    self.buf = bytes(bufsize)

                    # Open Arduino connection
                    if not self.open_arduino("COM12"):
                        return

                    if self.buf:
                        try:
                            self.hcam.put_ExpoTime(100)
                            self.hcam.StartPullModeWithCallback(self.cameraCallback, self)
                            input('Press ENTER to exit')
                        except amcam.HRESULTException as ex:
                            print('Failed to start camera, hr=0x{:x}'.format(ex.hr))
                finally:
                    self.hcam.Close()
                    self.hcam = None
                    self.buf = None
                    self.close_arduino()
                    cv2.destroyAllWindows()


app = App()
app.run()
