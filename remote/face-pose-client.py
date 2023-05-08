# from this video: 
# https://www.youtube.com/watch?v=-toNMaS4SeQ

import cv2
import mediapipe as mp
import numpy as np
import time
import socket

# open socket to omniverse machine
mysocket = socket.socket()
mysocket.connect(('192.168.4.209',12346))


def close_socket(thissocket):
    try:
        thissocket.shutdown(socket.SHUT_RDWR)
        thissocket.close()
        thissocket = None
    except socket.error as e:
        pass
    print("socket is closed")


mp_face_mesh = mp.solutions.face_mesh

face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)

mp_drawing = mp.solutions.drawing_utils

drawing_spec = mp_drawing.DrawingSpec(thickness=1, circle_radius=1)

# cap = cv2.VideoCapture(0)
cap = cv2.VideoCapture(1)

try: 
	while cap.isOpened():
		success, image = cap.read()

		start = time.time()

		image = cv2.cvtColor(cv2.flip(image, -1), cv2.COLOR_BGR2RGB)

		# improve performance
		image.flags.writeable = False

		# get the results
		results = face_mesh.process(image)

		# improve performance
		image.flags.writeable = True

		# convert colorspace
		image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)


		img_h, img_w, img_c = image.shape
		face_3d = []
		face_2d = []
		nose_norm = []
		

		if results.multi_face_landmarks:
			for face_landmarks in results.multi_face_landmarks:
				for idx, lm in enumerate(face_landmarks.landmark):
					if idx == 33 or idx == 263 or idx == 1 or idx == 61 or idx == 291 or idx == 199:
						if idx == 1:
							nose_2d = (lm.x * img_w, lm.y * img_h)
							nose_norm = (lm.x, lm.y)
							nose_3d = (lm.x * img_w, lm.y * img_h, lm.z * 3000)

						x, y = int(lm.x * img_w), int(lm.y * img_h)

						face_2d.append([x, y])

						face_3d.append([x, y, lm.z])

				# convert to numpy array
				face_2d = np.array(face_2d, dtype=np.float64)

				# convert to np array
				face_3d = np.array(face_3d, dtype=np.float64)

				# the camera matrix focal length
				focal_length = 1 * img_w

				cam_matrix = np.array( [
					[focal_length, 0, img_h/2],
					[0, focal_length, img_w/2],
					[0, 0, 1]
					])

				# the distortion paramaters
				dist_matrix = np.zeros((4, 1), dtype=np.float64)

				# solve PnP
				success, rot_vec, trans_vec = cv2.solvePnP(face_3d, face_2d, cam_matrix, dist_matrix)

				# get rotatinal matrix
				rmat, jac = cv2.Rodrigues(rot_vec)

				# get angles
				angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)

				# get the y rotation degree
				x = angles[0] * 360
				y = angles[1] * 360
				z = angles[2] * 360

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
				nose_3d_projection, jacobian = cv2.projectPoints(nose_3d, rot_vec, trans_vec, cam_matrix, dist_matrix)

				p1 = (int(nose_2d[0]), int(nose_2d[1]))
				p2 = (int(nose_2d[0] + y * 10), int(nose_2d[1] - x * 10))

				cv2.line(image, p1, p2, (255, 0, 0), 3)
				
				# add the text on the image
				cv2.putText(image, text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
				cv2.putText(image, "x: "+str(np.round(x, 2)), (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
				cv2.putText(image, "y: "+str(np.round(y, 2)), (500, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
				cv2.putText(image, "z: "+str(np.round(z, 2)), (500, 150), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
				xdiff = 0.5-nose_norm[0]
				ydiff = 0.5-nose_norm[1]
				cv2.putText(image, "xdiff: "+str(np.round(xdiff, 2)), (500, 250), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)
				cv2.putText(image, "ydiff: "+str(np.round(ydiff, 2)), (500, 300), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

			end = time.time()
			totalTime = end-start

			fps = 1/totalTime
			# print("FPS: ", fps)

			cv2.putText(image, f'FPS: {int(fps)}', (20, 450), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 2)

			mp_drawing.draw_landmarks(
				image=image,
				landmark_list=face_landmarks,
				connections=mp_face_mesh.FACEMESH_CONTOURS,
				landmark_drawing_spec = drawing_spec,
				connection_drawing_spec=drawing_spec)
			try:
				print("sending:", [x, y, z, xdiff, ydiff])
				sendData = str([x, y, z, xdiff, ydiff])
				mysocket.send(sendData.encode())
			except:
				pass

		cv2.imshow('Head Pose Estimation', image)

		if cv2.waitKey(5) & 0xFF == 27:
			break
except KeyboardInterrupt:
	print("quitting")
close_socket(mysocket)

cap.release()


