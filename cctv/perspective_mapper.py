
import cv2
import numpy as np
import requests

# Global variables
map_points = []
stream_points = []
map_img = None
warped_frame = None

def select_points_map(event, x, y, flags, param):
    """Mouse callback function for map window"""
    global map_points, map_img
    if event == cv2.EVENT_LBUTTONDOWN and len(map_points) < 4:
        map_points.append((x, y))
        cv2.circle(map_img, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Map", map_img)

def select_points_stream(event, x, y, flags, param):
    """Mouse callback function for stream window"""
    global stream_points
    if event == cv2.EVENT_LBUTTONDOWN and len(stream_points) < 4:
        # We need to capture the frame at the moment of the click
        frame = param
        stream_points.append((x, y))
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        cv2.imshow("Stream", frame)

def main():
    global map_points, stream_points, map_img, warped_frame

    # Load the map
    map_img_original = cv2.imread("map.pgm")
    if map_img_original is None:
        print("Error: map.pgm not found or could not be read.")
        return
    map_img = map_img_original.copy()

    # Setup windows and mouse callbacks
    cv2.namedWindow("Map")
    cv2.setMouseCallback("Map", select_points_map)
    cv2.namedWindow("Stream")
    
    print("Instructions:")
    print("1. Click on 4 corresponding points on the 'Map' window.")
    print("2. Click on the same 4 corresponding points on the 'Stream' window.")
    print("3. Press 'w' to warp the perspective after selecting points.")
    print("4. Press 'r' to reset the points.")
    print("5. Press 'q' to quit.")

    # MJPEG Stream URL
    MJPEG_URL = "http://10.10.16.78:8080/?action=stream"
    try:
        session = requests.Session()
        stream = session.get(MJPEG_URL, stream=True, timeout=10)
        stream.raise_for_status()
        bytes_buffer = bytes()

        homography_matrix = None

        while True:
            # Display the map image
            cv2.imshow("Map", map_img)

            # Read frame from stream
            chunk = stream.raw.read(4096)
            if not chunk:
                break
            bytes_buffer += chunk
            start = bytes_buffer.find(b'\xff\xd8')
            end = bytes_buffer.find(b'\xff\xd9')

            if start != -1 and end != -1:
                jpg = bytes_buffer[start:end+2]
                bytes_buffer = bytes_buffer[end+2:]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                # Set mouse callback for stream window with current frame
                cv2.setMouseCallback("Stream", select_points_stream, frame)

                # Draw selected points on the stream frame
                for point in stream_points:
                    cv2.circle(frame, point, 5, (0, 255, 0), -1)
                
                cv2.imshow("Stream", frame)

                if warped_frame is not None:
                    cv2.imshow("Warped Stream", warped_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('w') and len(map_points) == 4 and len(stream_points) == 4:
                print("Calculating homography matrix...")
                pts_src = np.array(stream_points, dtype=float)
                pts_dst = np.array(map_points, dtype=float)
                homography_matrix, status = cv2.findHomography(pts_src, pts_dst)
                print("Homography matrix calculated.")
                np.save('homography_matrix.npy', homography_matrix)
                print("Homography matrix saved to homography_matrix.npy")
            
            elif key == ord('r'):
                print("Resetting points.")
                map_points = []
                stream_points = []
                map_img = map_img_original.copy()
                warped_frame = None
                homography_matrix = None


            if homography_matrix is not None:
                # Warp the current frame
                warped_frame = cv2.warpPerspective(frame, homography_matrix, (map_img.shape[1], map_img.shape[0]))


    except requests.exceptions.RequestException as e:
        print(f"Error connecting to the source stream: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
