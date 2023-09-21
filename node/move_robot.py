
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
   
def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/rrbot/camera1/image_raw", Image, callback)

  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
   

if __name__ == '__main__':
  listener()



from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding='passthrough')

ret, frame = cap.read()

width = frame.shape[1]
height = frame.shape[0]


# Initialize counter for going through frames
frame_count = 0

# Loop to read all 885 frames and apply the algorithm
while frame_count < 885:
    ret, frame = cap.read()

    if not ret:
        break

    # Identify the row 2 pixels above the bottom edge
    row_index_2 = frame.shape[0] - 2 - 1

    # Find pixel values for that row
    row_pixels_2 = frame[row_index_2, :, :]

    # Calculate brightness for each pixel in the row
    brightness_values_2 = np.sum(row_pixels_2[:, :3], axis=1)

    # Find the coordinates of the leftmost and rightmost dark spots
    dark_pixel_indices_2 = np.where(brightness_values_2 < 188)[0]
    if len(dark_pixel_indices_2) > 0:
        leftmost_dark_spot = dark_pixel_indices_2[0]
        rightmost_dark_spot = dark_pixel_indices_2[-1]
        middle_spot = (leftmost_dark_spot + rightmost_dark_spot) // 2
        middle_coordinates = (middle_spot, row_index_2)


        # Draw a large red dot at the middle spot using OpenCV's circle function

        cv2.circle(frame, middle_coordinates, 5, [0, 0, 255], -1)  # Red circle with r = 5

    else:
        middle_coordinates = None


    # Convert frame to RGB for displaying
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    out.write(frame_rgb)

    frame_count += 1

cap.release()
out.release()
