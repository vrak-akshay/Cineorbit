
import cv2
import numpy as np

# Initialize ORB
orb = cv2.ORB_create(
    nfeatures=500,
    scaleFactor=1.2,
    nlevels=8
)

# Open laptop webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Camera could not be opened.")
    exit()

while True:

    ret, frame = cap.read()

    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 1. Shi-Tomasi corner detection
    corners = cv2.goodFeaturesToTrack(
        gray,
        maxCorners=300,
        qualityLevel=0.01,
        minDistance=10,
        blockSize=7
    )

    # Draw Shi-Tomasi landmarks
    if corners is not None:
        for corner in corners:
            x, y = corner.ravel()
            cv2.circle(
                frame,
                (int(x), int(y)),
                4,
                (0, 255, 0),
                -1
            )

    # 2. ORB feature detection and description
    keypoints, descriptors = orb.detectAndCompute(
        gray, None
    )

    # Draw ORB keypoints
    orb_frame = cv2.drawKeypoints(
        frame,
        keypoints,
        None,
        color=(0, 255, 255),
        flags=cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS
    )

    # Display information
    cv2.putText(
        orb_frame,
        f"Shi-Tomasi: {0 if corners is None else len(corners)}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2
    )

    cv2.putText(
        orb_frame,
        f"ORB: {len(keypoints)}",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2
    )

    cv2.imshow("RCPLS - Landmark Detection", orb_frame)

    # Press Q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
