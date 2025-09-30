import cv2
import requests
import numpy as np

def main():
    """
    Connects to an MJPEG stream and displays the video.
    """
    # TODO: Replace with your MJPEG stream URL
    url = "http://10.10.16.78:8080/?action=stream"

    try:
        print(f"Connecting to stream: {url}")
        stream = requests.get(url, stream=True, timeout=10)
        stream.raise_for_status()
        print("Connection successful.")

        bytes_buffer = bytes()
        for chunk in stream.iter_content(chunk_size=4096):
            bytes_buffer += chunk
            a = bytes_buffer.find(b'\xff\xd8') # JPEG start
            b = bytes_buffer.find(b'\xff\xd9') # JPEG end
            if a != -1 and b != -1:
                jpg = bytes_buffer[a:b+2]
                bytes_buffer = bytes_buffer[b+2:]
                
                if jpg:
                    # Decode the JPEG image
                    frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        # Resize frame to be larger
                        scale_percent = 300 # percent of original size
                        width = int(frame.shape[1] * scale_percent / 100)
                        height = int(frame.shape[0] * scale_percent / 100)
                        dim = (width, height)
                        resized_frame = cv2.resize(frame, dim, interpolation = cv2.INTER_AREA)

                        # Display the resulting frame
                        cv2.imshow('MJPEG Stream', resized_frame)
                    
                    # Press 'q' to exit
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
    
    except requests.exceptions.RequestException as e:
        print(f"Error connecting to the stream: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cv2.destroyAllWindows()
        print("Stream closed.")

if __name__ == '__main__':
    main()
