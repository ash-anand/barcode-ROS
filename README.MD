# Barcode Reader

Implements zbar for ROS. Subscribes to '/usb_cam/image_raw' and publishes detected results under '/code'. 

```/code/image``` displays final image after processing

```/code/qr``` contains the qr code data if one is encountered.

```/code/bar``` contains any other barcode data if one in encountered.

Requiremnets :

OpenCV

Zbar

usb_cam
