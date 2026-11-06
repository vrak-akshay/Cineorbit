
import cv2
import numpy as np

# ==========================================
# RCPLS - MILESTONE 2.5
# Adaptive Landmark Detection
# Shi-Tomasi + Lucas-Kanade + Registry
# ==========================================

# Shi-Tomasi parameters
feature_params = dict(
    maxCorners=200,
    qualityLevel=0.01,
    minDistance=10,
    blockSize=7
)

# Lucas-Kanade parameters
lk_params = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(
        cv2.TERM_CRITERIA_EPS |
        cv2.TERM_CRITERIA_COUNT,
        30,
        0.01
    )
)

# Tracking validation
FB_THRESHOLD = 1.5
PHOTO_THRESHOLD = 20

# Adaptive detection parameters
MAX_ACTIVE = 200
MIN_ACTIVE = 80
EXCLUSION_RADIUS = 12

# ==========================================
# 1. INITIALIZE CAMERA
# ==========================================

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("ERROR: Cannot access webcam.")
    exit()

ret, old_frame = cap.read()

if not ret:
    print("ERROR: Cannot read camera frame.")
    cap.release()
    exit()

old_gray = cv2.cvtColor(
    old_frame,
    cv2.COLOR_BGR2GRAY
)

# ==========================================
# 2. INITIAL LANDMARK DETECTION
# ==========================================

p0 = cv2.goodFeaturesToTrack(
    old_gray,
    mask=None,
    **feature_params
)

if p0 is None:
    print("ERROR: No landmarks detected.")
    cap.release()
    exit()

initial_total = len(p0)

print("Initial landmarks:", initial_total)

# ==========================================
# 3. LANDMARK REGISTRY
# ==========================================

landmark_registry = {}

next_landmark_id = 1

landmark_ids = []

for point in p0:

    landmark_id = f"L{next_landmark_id:03d}"
    next_landmark_id += 1

    x, y = point.ravel()

    landmark_registry[landmark_id] = {
        "position": (float(x), float(y)),
        "status": "TRACKING",
        "history": [(float(x), float(y))]
    }

    landmark_ids.append(landmark_id)

# Original landmark IDs are preserved for survival calculations
original_ids = set(landmark_ids)

# Drawing trails
mask = np.zeros_like(old_frame)

frame_count = 0

# ==========================================
# 4. MAIN TRACKING LOOP
# ==========================================

while True:

    ret, frame = cap.read()

    if not ret:
        print("Camera frame unavailable.")
        break

    frame_gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    # ======================================
    # 5. FORWARD AND BACKWARD TRACKING
    # ======================================

    if len(p0) > 0:

        p1, status_f, error_f = cv2.calcOpticalFlowPyrLK(
            old_gray,
            frame_gray,
            p0,
            None,
            **lk_params
        )

        if p1 is None or status_f is None:
            print("Forward tracking failed.")
            break

        p0_back, status_b, error_b = cv2.calcOpticalFlowPyrLK(
            frame_gray,
            old_gray,
            p1,
            None,
            **lk_params
        )

        if p0_back is None or status_b is None:
            print("Backward tracking failed.")
            break

        # ==================================
        # 6. TRACKING VALIDATION
        # ==================================

        original_points = p0.reshape(-1, 2)
        backward_points = p0_back.reshape(-1, 2)

        fb_error = np.linalg.norm(
            original_points - backward_points,
            axis=1
        )

        photo_error = error_f.ravel()

        new_positions = p1.reshape(-1, 2)

        h, w = frame_gray.shape

        inside = (
            (new_positions[:, 0] >= 0) &
            (new_positions[:, 0] < w) &
            (new_positions[:, 1] >= 0) &
            (new_positions[:, 1] < h)
        )

        valid = (
            (status_f.ravel() == 1) &
            (status_b.ravel() == 1) &
            (fb_error < FB_THRESHOLD) &
            (photo_error < PHOTO_THRESHOLD) &
            inside
        )

        # Accepted landmarks
        good_new = p1[valid]
        good_old = p0[valid]

        # Rejected landmarks
        lost_old = p0[~valid]

        # Preserve corresponding IDs
        good_ids = [
            landmark_ids[i]
            for i in range(len(landmark_ids))
            if valid[i]
        ]

        lost_ids = [
            landmark_ids[i]
            for i in range(len(landmark_ids))
            if not valid[i]
        ]

    else:

        # Allow new detection even if all previous
        # landmarks have been lost.
        good_new = np.empty((0, 1, 2), dtype=np.float32)
        good_old = np.empty((0, 1, 2), dtype=np.float32)

        lost_old = np.empty((0, 1, 2), dtype=np.float32)

        good_ids = []
        lost_ids = []

    # ======================================
    # 7. UPDATE EXISTING LANDMARK REGISTRY
    # ======================================

    for landmark_id, point in zip(good_ids, good_new):

        x, y = point.ravel()

        entry = landmark_registry[landmark_id]

        entry["position"] = (float(x), float(y))
        entry["status"] = "TRACKING"

        entry["history"].append(
            (float(x), float(y))
        )

    for landmark_id in lost_ids:

        landmark_registry[landmark_id]["status"] = "LOST"

    # ======================================
    # 8. ADAPTIVE LANDMARK DETECTION
    # ======================================

    active_count = len(good_new)

    newly_detected = 0
    new_points = np.empty((0, 1, 2), dtype=np.float32)

    if active_count < MIN_ACTIVE:

        required = MAX_ACTIVE - active_count

        if required > 0:

            # Create mask to exclude regions around
            # currently tracked landmarks.
            detection_mask = np.full(
                frame_gray.shape,
                255,
                dtype=np.uint8
            )

            for point in good_new:

                x, y = point.ravel()

                cv2.circle(
                    detection_mask,
                    (int(x), int(y)),
                    EXCLUSION_RADIUS,
                    0,
                    -1
                )

            # Request only the number of points needed.
            adaptive_params = feature_params.copy()
            adaptive_params["maxCorners"] = required

            candidates = cv2.goodFeaturesToTrack(
                frame_gray,
                mask=detection_mask,
                **adaptive_params
            )

            if candidates is not None:

                new_points = candidates

                # Register every new landmark
                for point in new_points:

                    landmark_id = f"L{next_landmark_id:03d}"
                    next_landmark_id += 1

                    x, y = point.ravel()

                    landmark_registry[landmark_id] = {
                        "position": (float(x), float(y)),
                        "status": "NEW",
                        "history": [(float(x), float(y))]
                    }

                    good_ids.append(landmark_id)

                newly_detected = len(new_points)

                print(
                    f"Frame {frame_count}: "
                    f"Detected {newly_detected} new landmarks"
                )

    # ======================================
    # 9. DRAW EXISTING TRACKED LANDMARKS
    # ======================================

    for landmark_id, new, old in zip(
        good_ids[:active_count],
        good_new,
        good_old
    ):

        x_new, y_new = new.ravel()
        x_old, y_old = old.ravel()

        x_new, y_new = int(x_new), int(y_new)
        x_old, y_old = int(x_old), int(y_old)

        cv2.circle(
            frame,
            (x_new, y_new),
            5,
            (0, 255, 0),
            -1
        )

        cv2.putText(
            frame,
            landmark_id,
            (x_new + 7, y_new - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 255, 0),
            1
        )

        cv2.line(
            mask,
            (x_old, y_old),
            (x_new, y_new),
            (255, 0, 0),
            2
        )

        cv2.arrowedLine(
            frame,
            (x_old, y_old),
            (x_new, y_new),
            (255, 0, 0),
            1,
            tipLength=0.3
        )

    # ======================================
    # 10. DRAW NEW LANDMARKS
    # ======================================

    new_ids = good_ids[active_count:]

    for landmark_id, point in zip(new_ids, new_points):

        x, y = point.ravel()

        cv2.circle(
            frame,
            (int(x), int(y)),
            5,
            (255, 0, 0),
            -1
        )

        cv2.putText(
            frame,
            landmark_id,
            (int(x) + 7, int(y) - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 0, 0),
            1
        )

    # ======================================
    # 11. DRAW REJECTED LANDMARKS
    # ======================================

    for landmark_id, point in zip(lost_ids, lost_old):

        x, y = point.ravel()

        cv2.drawMarker(
            frame,
            (int(x), int(y)),
            (0, 0, 255),
            cv2.MARKER_TILTED_CROSS,
            12,
            2
        )

        cv2.putText(
            frame,
            landmark_id,
            (int(x) + 7, int(y) - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (0, 0, 255),
            1
        )

    # Combine movement trails and current frame
    output = cv2.add(frame, mask)

    # ======================================
    # 12. UPDATE TRACKING POINTS
    # ======================================

    # Combine old accepted points and newly detected points.
    if len(new_points) > 0:

        p0 = np.concatenate(
            (good_new, new_points),
            axis=0
        )

    else:

        p0 = good_new.copy()

    # IDs must follow exactly the same point order
    landmark_ids = good_ids.copy()

    # ======================================
    # 13. STATISTICS
    # ======================================

    active_count = len(p0)

    original_still_tracking = sum(
        1 for landmark_id in landmark_ids
        if landmark_id in original_ids
    )

    survival_rate = (
        original_still_tracking / initial_total
    ) * 100

    cumulative_lost = sum(
        1 for data in landmark_registry.values()
        if data["status"] == "LOST"
    )

    frame_count += 1

    stats = [
        f"Frame: {frame_count}",
        f"Active: {active_count}",
        f"New this frame: {newly_detected}",
        f"Rejected this frame: {len(lost_old)}",
        f"Lost cumulative: {cumulative_lost}",
        f"Original survival: {survival_rate:.1f}%"
    ]

    for i, text in enumerate(stats):

        cv2.putText(
            output,
            text,
            (10, 30 + i * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            2
        )

    cv2.imshow(
        "RCPLS - Adaptive Landmark Detection",
        output
    )

    # ======================================
    # 14. PREPARE NEXT FRAME
    # ======================================

    old_gray = frame_gray.copy()

    # ======================================
    # 15. EXIT
    # ======================================

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

# ==========================================
# 16. FINAL LANDMARK REGISTRY
# ==========================================

print("\n========== LANDMARK REGISTRY ==========")

for landmark_id, data in landmark_registry.items():

    print(
        landmark_id,
        "| Status:", data["status"],
        "| Position:", data["position"],
        "| Observations:", len(data["history"])
    )

print("Total registered:", len(landmark_registry))

print("=======================================")

cap.release()
cv2.destroyAllWindows()

print("Milestone 2.5 completed.")
