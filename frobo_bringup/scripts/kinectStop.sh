echo "Turning off Kinect..."
echo '1-1.3' > /sys/bus/usb/drivers/usb/unbind
sleep 2.5
echo '1-1.3' > /sys/bus/usb/drivers/usb/unbind
echo "Done"

