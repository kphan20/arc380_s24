import cv2

# Replace 'image_path.jpg' with the path to your image file
image_path = 'img.png'

# Read the image using OpenCV
image = cv2.imread(image_path)

# Check if the image was successfully loaded
if image is not None:
    # Display the image
    cv2.imshow('Image', image)
    cv2.waitKey(0)  # Wait for any key to be pressed
    cv2.destroyAllWindows()  # Close all OpenCV windows
else:
    print("Error: Unable to load the image.")