# ====== Sample Code for Smart Design Technology Blog ======

# Intel Realsense D435 cam has RGB camera with 1920×1080 resolution
# Depth camera is 1280x720
# FOV is limited to 69deg x 42deg (H x V) - the RGB camera FOV

# If you run this on a non-Intel CPU, explore other options for rs.align
    # On the NVIDIA Jetson AGX we build the pyrealsense lib with CUDA

import pyrealsense2 as rs
import mediapipe as mp
import cv2
import numpy as np
import datetime as dt
import socket
import sys
from itertools import combinations

font = cv2.FONT_HERSHEY_SIMPLEX
org = (20, 100)
fontScale = .5
color = (0,50,255)
thickness = 1

# ====== Realsense ======
realsense_ctx = rs.context()
connected_devices = [] # List of serial numbers for present cameras
for i in range(len(realsense_ctx.devices)):
    detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    print(f"{detected_camera}")
    connected_devices.append(detected_camera)
device = connected_devices[0] # In this example we are only using one camera
pipeline = rs.pipeline()
config = rs.config()
background_removed_color = 153 # Grey

# ====== Mediapipe ======
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

# ====== Socket ======
bSocket = True

if (len(sys.argv) > 1):
	print(sys.argv)
	if sys.argv[1] == "--no-socket":
		bSocket = False


if bSocket:
	# open socket to omniverse machine
	mysocket = socket.socket()
	#mysocket.connect(('192.168.1.62',12346))
	mysocket.connect(('127.0.0.1',12346))


def close_socket(thissocket):
    try:
        thissocket.shutdown(socket.SHUT_RDWR)
        thissocket.close()
        thissocket = None
    except socket.error as e:
        pass
    print("socket is closed")

def get_pixel_depth(depth_image_flipped, pixel_coodinate, img_w, img_h):
        #calculate distance to end of the nose
    x_pixel = pixel_coodinate[0]
    y_pixel = pixel_coodinate[1]
    if x_pixel >= img_w:
        x_pixel = img_w - 1
    if y_pixel >= img_h:
        y_pixel = img_h - 1
    
    return depth_image_flipped[y_pixel, x_pixel] * depth_scale


def get_camera_to_pixel_vector(pixel_coodinate, inv_cam_matrix, distance_from_camera):
    homogeneous_nose_coordinates = np.array([
        [pixel_coodinate[0]], 
        [pixel_coodinate[1]],
        [1]
    ])
    homogeneous_nose_coordinates = homogeneous_nose_coordinates / np.linalg.norm(homogeneous_nose_coordinates)
    cam_to_pixel = inv_cam_matrix @ homogeneous_nose_coordinates
    
    # Normalize the input vectors
    cam_to_pixel = cam_to_pixel / np.linalg.norm(cam_to_pixel)
    #print(vector_to_nose.flatten())
    return cam_to_pixel * distance_from_camera


def estimate_plane_normal(points):
    planes = list(combinations(points, 3))
    #print(points)
    normals = []
    for points in planes:
        a = points[1] - points[0]
        b = points[2] - points[0]
        #print (a,b)
        normal = np.cross(a, b)

        if normal[2] > 0:
            normal = -1 * normal

        normals.append(normal)

    plane_normal = np.mean(normals, axis=0)
    plane_normal = plane_normal / np.linalg.norm(plane_normal)
    

    return plane_normal

# ====== Enable Streams ======
config.enable_device(device)

# # For worse FPS, but better resolution:
# stream_res_x = 1280
# stream_res_y = 720
# # For better FPS. but worse resolution:
stream_res_x = 640
stream_res_y = 480

stream_fps = 30

config.enable_stream(rs.stream.depth, stream_res_x, stream_res_y, rs.format.z16, stream_fps)
config.enable_stream(rs.stream.color, stream_res_x, stream_res_y, rs.format.bgr8, stream_fps)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# ====== Get depth Scale ======
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"\tDepth Scale for Camera SN {device} is: {depth_scale}")

# ====== Set clipping distance ======
clipping_distance_in_meters = 2
clipping_distance = clipping_distance_in_meters / depth_scale
print(f"\tConfiguration Successful for SN {device}")

# ====== Get and process images ====== 
print(f"Starting to capture images on SN: {device}")

while True:
    start_time = dt.datetime.today().timestamp()

    # Get and align frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not aligned_depth_frame or not color_frame:
        continue

    # Process images
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image_flipped = cv2.flip(depth_image,1)
    color_image = np.asanyarray(color_frame.get_data())

    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #Depth image is 1 channel, while color image is 3
    background_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), background_removed_color, color_image)

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    images = cv2.flip(background_removed,1)
    color_image = cv2.flip(color_image,1)
    color_images_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    
    #added from face-pose client
    img_h, img_w, img_c = color_images_rgb.shape

    # calculate intrinsic values
    focal_length = 1 * img_w
    
    intrinsics = aligned_frames.get_profile().as_video_stream_profile().get_intrinsics()

    cam_matrix = np.array( [
        [intrinsics.fx, 0, img_w/2],
        [0, intrinsics.fy, img_h/2],
        [0, 0, 1]
    ])
    
    inv_cam_matrix = np.linalg.inv(cam_matrix)

    # the distortion paramaters
    dist_matrix = np.zeros((4, 1), dtype=np.float64)

    face_3d = []
    face_2d = []

    # Process face
    results = face_mesh.process(color_images_rgb)
    if results.multi_face_landmarks:
        i=0
        for face_landmarks in results.multi_face_landmarks:
            for idx, lm in enumerate(face_landmarks.landmark):
                if idx in [1, 33, 61, 199, 263, 291]:
                    x, y = int(lm.x * img_w), int(lm.y * img_h)
            
                    cv2.circle(color_images_rgb, (x, y), 10, (0, 0, 255), -1)

                    face_2d.append([x, y])

                    landmark_distance = get_pixel_depth(
                        depth_image_flipped,
                        [x, y],
                        img_w,
                        img_h
                    )

                    cam_to_landmark = get_camera_to_pixel_vector(
                        [x, y], 
                        inv_cam_matrix, 
                        landmark_distance
                    )

                    face_3d.append(cam_to_landmark.flatten())

                    if idx == 1:
                        nose_pixel = (x, y)
                        cam_to_nose = cam_to_landmark
                        nose_distance = landmark_distance

            face_2d = np.array(face_2d, dtype=np.float64)
            face_3d = np.array(face_3d, dtype=np.float64)

            nose_landmark = face_landmarks.landmark[1]

            # solve PnP
            success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

            # get rotatinal matrix
            rmat, jac = cv2.Rodrigues(rot_vec)

            # get angles
            angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

            # get the y rotation degree
            x_degrees = angles[0] * 360
            y_degrees = angles[1] * 360
            z_degrees = angles[2] * 360
            
            # see where the user's head is tilting
            if y < -10:
                text = "Looking Left"
            elif y > 10:
                text = "Looking Right"
            elif x < -10:
                text = "Looking Down"
            elif x > 10:
                text = "Looking Up"
            else:
                text = "Looking Forward"

            # display the nose direction
            p2 = (int(nose_pixel[0] + y_degrees * 10), int(nose_pixel[1] - x_degrees * 10))
            cv2.line(color_images_rgb, (int(nose_pixel[0]), int(nose_pixel[1])), p2, (255, 0, 0), 3)

            face_direction = estimate_plane_normal(face_3d)
            print(nose_distance)
            ##vector of Nose in camera local space
            #print(cam_to_nose.flatten())
            ##Normal of the face in camera local space
            #print(face_direction.flatten())

            # add the text on the image
            cv2.putText(color_images_rgb, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            cv2.putText(color_images_rgb, "x: "+str(np.round(x_degrees, 2)), (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            cv2.putText(color_images_rgb, "y: "+str(np.round(y_degrees, 2)), (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            cv2.putText(color_images_rgb, "z: "+str(np.round(z_degrees, 2)), (500, 150), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            cv2.putText(color_images_rgb, "Distance: "+ str(np.round(nose_distance, 2)), (500, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            xdiff = 0.5-nose_landmark.x
            ydiff = 0.5-nose_landmark.y
            cv2.putText(color_images_rgb, "xdiff: "+str(np.round(xdiff, 2)), (500, 250), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
            images = cv2.putText(color_images_rgb, "ydiff: "+str(np.round(ydiff, 2)), (500, 300), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

        mp_drawing.draw_landmarks(
            image=color_images_rgb,
            landmark_list=face_landmarks,
            connections=mp_face_mesh.FACEMESH_CONTOURS,
            landmark_drawing_spec = drawing_spec,
            connection_drawing_spec=drawing_spec)
        
        if bSocket:
            cam_to_nose = cam_to_nose.flatten()
            try:
                #print("sending:", [cam_to_nose[0], cam_to_nose[1], cam_to_nose[2], face_direction[0], face_direction[1], face_direction[2], xdiff, ydiff])
                sendData = str([
                    cam_to_nose[0], cam_to_nose[1], cam_to_nose[2], 
                    face_direction[0], face_direction[1], face_direction[2], 
                    xdiff, ydiff]
                )
                mysocket.send(sendData.encode())
            except:
                pass

    # Display FPS
    time_diff = dt.datetime.today().timestamp() - start_time
    fps = int(1 / time_diff)
    org3 = (20, org[1] + 60)
    images = cv2.putText(images, f"FPS: {fps}", org3, font, fontScale, color, thickness, cv2.LINE_AA)

    name_of_window = 'SN: ' + str(device)

    # Display images 
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(name_of_window, images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print(f"User pressed break key for SN: {device}")
        break

print(f"Application Closing")
pipeline.stop()
print(f"Application Closed.")
