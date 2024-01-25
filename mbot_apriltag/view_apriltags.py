import cv2
import time
import numpy as np
import yaml
from dt_apriltags import Detector
from gst_cam import camera
from mbot_lcm_msgs.pose2D_t import pose2D_t
import lcm

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")

#Coordinate system: 
# Camera: Z = away from camera, X = to the right of image of camera, Y = towards the ground of image of camera
# Tag: Z = into tag, X = to the right of tag, Y = towards the ground of tag


SMALL_TAG_IDS = [10, 20, 30, 40]
SMALL_TAG_SIZE_MM = 10.8
LARGE_TAG_IDS = [1, 2, 3, 4]
LARGE_TAG_SIZE_MM = 54

CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# Load camera parameters from yaml
def get_camera_params():
    with open("camera_params.yaml", 'r') as f:
        params = yaml.safe_load(f)

    camera_matrix = np.array(params['camera_matrix'], dtype=np.float32)
    K = camera_matrix
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    camera_params = [fx, fy, cx, cy]
    distortion_coefficients = np.array(params['distortion_coefficients'], dtype=np.float32)
    extrinsic_matrix = np.array(params['extrinsic_matrix'], dtype=np.float32)
    return camera_matrix, camera_params, distortion_coefficients,extrinsic_matrix

#Using pose R and t of detected apriltags, return the angle and distance of the tag from the camera
# Angle = angle between camera's Z axis and tag's Z axis, in degrees
# Distance = XZ plane distance between camera and tag, in meters
def get_angle_and_dist_of_tag(tag_id, pose_R_mat, pose_t_mat):
    

    distance = np.sqrt(float(pose_t_mat[0]**2 + pose_t_mat[2]**2))
    if tag_id in SMALL_TAG_IDS:
        distance *= (SMALL_TAG_SIZE_MM / LARGE_TAG_SIZE_MM) #Detector assumes everything is large tag size. Scale down distance for small tags
    yaw = np.arctan2(pose_R_mat[0][2], pose_R_mat[2][2]) * 180 / np.pi
    return distance, yaw
    
def main():
    camera_matrix, camera_params, distortion_coefficients,extrinsic_matrix = get_camera_params()
    cap = cv2.VideoCapture(camera(0, CAMERA_WIDTH, CAMERA_HEIGHT))
    R = extrinsic_matrix[:, :3]  # First three columns of extrinsic matrix
    t = extrinsic_matrix[:, 3] 

    extrinsic_matrix = np.vstack((extrinsic_matrix,[0,0,0,1]))
    H_inv = np.linalg.inv(extrinsic_matrix)#4x4

    time.sleep(3)

    # Initialize AprilTag detector
    detector = Detector(searchpath=['apriltags3py/apriltags/lib', 'apriltags3py/apriltags/lib'],
                            families='tagCustom48h12',
                            nthreads=4,
                            quad_decimate=2,
                            quad_sigma=0.4,
                            refine_edges=1,
                            decode_sharpening=1,
                            max_hamming=1,
                            debug=0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame = cv2.undistort(frame, camera_matrix, distortion_coefficients)

        # Detect AprilTags
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = detector.detect(gray, True, camera_params, LARGE_TAG_SIZE_MM * (1/1000))
        # print(tags)
        

        for det in tags:
            # Draw bounding box
            for i in range(4):
                start_point = tuple(det.corners[i-1].astype(int))
                end_point = tuple(det.corners[i].astype(int))
                cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

            # Draw tag family and ID on the image
            tag_info = "ID:{}".format(det.tag_id)
            # the below is in camera frame
            dist, angle = get_angle_and_dist_of_tag(det.tag_id, det.pose_R, det.pose_t)

            cv2.putText(frame, tag_info, (int(det.center[0]), int(det.center[1])), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            

            print(f"ID {det.tag_id}: angle {angle}, distance {dist}")

            # print("pose",det.pose_t)
            # print("pose rot",det.pose_R)
            x = det.pose_t[0]
            y = det.pose_t[1]

           
            val = np.sqrt(det.pose_R[0,0]*det.pose_R[0,0]+det.pose_R[1,0]*det.pose_R[1,0])
            singular = val < 1e-6
            # check for singularity:
            if not singular:
                theta = np.arctan2(det.pose_R[1,0],det.pose_R[0,0])
            else:
                theta = np.arctan2(det.pose_R[2,1],det.pose_R[2,2])

            # get the first digit of the tag id
            first_digit = int(str(det.tag_id)[0])

            if (first_digit % 2 !=0):
                pass
                print("not even")
                
            else:
                # in here we need to the logic to go to the next april tag here
                # from the extrinsic matrix separate out your rotation and translation parts
                pose = pose2D_t()
                
                # R = det.pose_R  # 3x3 rotation matrix
                # t = det.pose_t  # 3x1 translation vector

                # # Convert to 4x4 transformation matrix
                # pose_matrix = np.eye(4)  # 4x4 identity matrix
                # pose_matrix[:3, :3] = R  # Set the upper 3x3 part as the rotation matrix
                # pose_matrix[:3, 3] = t.squeeze()
                
                # p = np.array([x,y,theta,1]).reshape((4,1))
                # x = pose_matrix[0, 3]
                # y = pose_matrix[1, 3]
                # theta = np.arctan2(pose_matrix[1, 0], pose_matrix[0, 0])

                # pose.x = x
                # pose.y = y
                # pose.theta = theta

                # # TEST 
                # pose.x = dist*np.cos(np.deg2rad(angle))
                # pose.y = dist*np.sin(np.deg2rad(angle))
                # pose.theta = np.deg2rad(angle)
                pose.x = det.pose_t[2]
                pose.y = det.pose_t[0]
                pose.x *= (SMALL_TAG_SIZE_MM / LARGE_TAG_SIZE_MM)
                pose.y *= (SMALL_TAG_SIZE_MM / LARGE_TAG_SIZE_MM)
                pose.theta = np.deg2rad(angle)
                # print("pose:",pose.x,pose.y,pose.theta)
                print("----pose is: ", pose.x,pose.y,pose.theta)
                lc.publish("PARCEL_POSITION",pose.encode())
                
                

                
            # print("pose",pose)
        lc.handle_timeout(10)

        cv2.imshow("Camera", frame)

        key = cv2.waitKey(10)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.release()

if __name__ == "__main__":
    main()