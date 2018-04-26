# teleop_controller
Using keyboard to control RM AI Robot.

A developed version of controller of RM AI Robot, providing a class Controller to control robot not only by keyboard but also by user defined node. Besides, it has a simulator access, which is underdeveloping

more in example.py
# Install
~~~
cd ~/catkin_ws/
catkin_make
~~~
# Run
~~~
python path to pyfile/teleop_control.py
python path to pyfile/example.py
~~~

# Reading from the keyboard  and Publishing to Twist!
Moving around:  

      u    i    o
      j    k    l
      m    ,    .  
      
# For Holonomic mode (strafing), hold down the shift key:  

      U    I    O
      J    K    L
      M    <    >
      t : up (+z) b : down (-z)  

# For RM Robot:
## gimbal control:+/-steps by 1 degree
        8
    4       6
        2
## shoot control:
    G-> GUN ON,         T/B +/- fric wheel speed by 100
    5->single shoot     space->continue shoot           0->stop shoot   
    
anything else : stop  

q/z : increase/decrease max speeds by 10%  

w/x : increase/decrease only linear speed by 10%  

e/c : increase/decrease only angular speed by 10%  

CTRL-C to quit  

