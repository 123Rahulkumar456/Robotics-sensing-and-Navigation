# gnss_driver

Driver structure:

--gnss_driver  #package name
  --analysis   #contain the matlab codes for analysis and lab report
  --data       #contain the bag and csv files
  --launch     #contain the launch file (driver.launch)
  --msg        #contain the message file (gnss_msg.msg)
  --python     #contain the driver code (gnssDriver.py)
  --CMakeLists.txt
  --package.xml

Topic name: /gnss

# use
clone this repository in a separate folder
catkin_make in that folder
source the devel/setup.bash in this folder
roslaunch gnss_driver driver.launch port:="<port name>"

