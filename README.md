# Seekur Jr. URDF/CAD robot model simulation

To satisfy requirements for Gazebo>=1.9 in ROS Groovy:

~~~{.bash}
sudo apt-get install ros-groovy-pcl-conversions
sudo apt-get install ros-groovy-control-msgs
~~~

Every session should be started as:

~~~{.bash}
source ~/rosws/setup.bash
source devel/setup.bash
localmaster

~~~

Simulator start:

~~~{.bash}
roscore

rosrun gazebo_ros gazebo
~~~

Spawn robot model:

~~~{.bash}
rosrun gazebo_ros spawn_model -file seekurjr.urdf -urdf -z 1 -model seekurjr

rosrun gazebo_ros spawn_model -file src/seekurjr_gazebo/urdf/seekurjr.urdf -urdf -z 1 -model seekurjr

roslaunch seekurjr_gazebo display.xml model:=src/seekurjr_gazebo/urdf/seekurjr.urdf
~~~

Execute ROS stereoprocessing core and rqt gui control application:

~~~
ROS_NAMESPACE=mobileranger rosrun stereo_image_proc stereo_image_proc
rosrun image_view stereo_view stereo:=/mobileranger image:=image_rect_color
rosrun rqt_gui rqt_gui 
~~~

Sample session can be seen by following video link below.

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/Nogzb74IlV0/0.jpg)](https://www.youtube.com/watch?v=Nogzb74IlV0)


# Inertia

Table 1. Main SeekurJr mechanical specification

Specification | Value
--- | ---:
DIMENSIONS (LxWxH) | 1050mm x 840mm x 500mm
WEIGHT      | 77kg (1 battery)
GROUND CLEARANCE | 105mm
TIRES | 400mm
WHEELBASE | 425mm


According to 
[Wikipedia](http://en.wikipedia.org/wiki/List_of_moment_of_inertia_tensors),
list of moment of inertia tensors:

One can calculate approximate inertai tensor for SeekurJr in Ocatve:

~~~{.octave}
m=77; h=0.5; w=0.84; d=1.05; 
ixx = 1/12*m*(h^2+d^2)
iyy = 1/12*m*(w^2+d^2) 
izz = 1/12*m*(h^2+d^2) 
~~~

Results are used in `<inertia />` URDF-tag parameters:

~~~
<inertia  ixx="8.6785" ixy="0"  ixz="0"  iyy="11.602"  iyz="0"  izz="8.6785" />
~~~

Another possible way to calculate inertia tensor for complicated objects is to use
dedicated software. For example one can load solid mechanical model in **Meshlab**
and calculate inertia in two steps:

- View > Show Layer Dialog
- Filter > Quality Measure and computations > Compute Geometric measures

~~~
Opened mesh /home/kp/rosws/catkin/src/seekurjr_gazebo/meshes/seekurjr_body_new_low.stl in 15 msec
All files opened in 1902 msec
Mesh Bounding Box Size 0.627341 0.394017 1.197709
Mesh Bounding Box Diag 1.408301 
Mesh Volume is 0.155966
Mesh Surface is 4.018419
Thin shell barycenter 0.008623 0.248038 -0.001686
Center of Mass is 0.006557 0.130922 -0.038411
Inertia Tensor is :
| 0.011083 -0.000132 -0.000303 |
| -0.000132 0.011182 -0.000597 |
| -0.000303 -0.000597 0.005307 |
Principal axes are :
| 0.885792 -0.460959 0.053744 |
| 0.454450 0.885044 0.100860 |
| -0.094058 -0.064917 0.993448 |
axis momenta are :
| 0.011047 0.011294 0.005230 |
Applied filter Compute Geometric Measures in 12 msec
~~~

Resulting `<inertia />` tag wolud be:

~~~
<inertia  ixx="0.011083" ixy="-0.000132"  ixz="-0.000303"  iyy="0.011182"  iyz="-0.000597"  izz="0.005307" />
~~~

Useful links:

~~~
http://answers.ros.org/question/30539/choosing-the-right-coefficients-for-gazebo-simulation/
http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
http://answers.gazebosim.org/question/4372/the-inertia-matrix-explained/
~~~

Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/base.urdf.xml`.

# Wheels and Steering

As for mu, mu2, slip1, slip2 they are explained here, notice that the values are between [0..1]. For more information check out this ODE page and just search for mu, mu2, slip1, slip2 it will be more deeply explained.
http://answers.gazebosim.org/question/1505/how-do-i-set-up-mu-and-slip-for-a-skid-steer-robot/
http://www.ode.org/ode-latest-userguide.html

For skid-steering model simulation `<plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">` is used.

Example for SeekurJr robot can be found in `seekurjr_gazebo/urdf/base.urdf.xml`
and `seekurjr_gazebo/urdf/base.gazebo.xml`.



## Recommended Mesh Resolution

For collision checking using the ROS motion planning packages, as few faces per link as possible are recommended for the collision meshes that you put into the URDF (ideally less than 1000). If possible, approximating the meshes with other primitives is encouraged. 
    
http://wiki.ros.org/urdf/XML/link

# IMU

~~~
<joint name="imu_joint" type="fixed">
    <axis xyz="1 0 0"/> <!-- 0 1 0 -->
    <origin xyz="0 0 0.19"/>
    <parent link="base_link"/>
    <child link="imu_link"/>
  </joint>


<link name="imu_link">
  <inertial>
    <mass value="0.001"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
  </inertial>
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </visual>
  <collision>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <box size=".001 .001 .001"/>
    </geometry>
  </collision>
</link>


<gazebo>
  <controller:gazebo_ros_imu name="imu_controller" plugin="libgazebo_ros_imu.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>50.0</updateRate> 
    <bodyName>imu_link</bodyName>
    <topicName>imu_data</topicName>
    <gaussianNoise>2.89e-08</gaussianNoise>
    <xyzOffsets>0 0 0</xyzOffsets>
    <rpyOffsets>0 0 0</rpyOffsets>
    <interface:position name="imu_position"/>
  </controller:gazebo_ros_imu>
</gazebo>

~~~



~~~
  <gazebo reference="imu_link">
    <!-- this is expected to be reparented to pelvis with appropriate offset
         when imu_link is reduced by fixed joint reduction -->
    <!-- todo: this is working with gazebo 1.4, need to write a unit test -->
<!--     <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>1000.0</update_rate>     
    </sensor>      -->
  </gazebo>

<!-- IMU -->

<link name="imu_link">
  <inertial>
    <mass value="0.001"/>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.0001"/>
  </inertial>
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </visual>
  <collision>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <box size=".001 .001 .001"/>
    </geometry>
  </collision>
</link>

<joint name="imu_joint" type="fixed">
	<axis xyz="1 0 0"/> <!-- 0 1 0 -->
	<origin xyz="0 0 0.19"/>
	<parent link="base_link"/>
	<child link="imu_link"/>
</joint>


<gazebo>
  <controller:gazebo_ros_imu name="imu_controller" plugin="libgazebo_ros_imu.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>50.0</updateRate> 
    <bodyName>imu_link</bodyName>
    <topicName>imu_data</topicName>
    <gaussianNoise>2.89e-08</gaussianNoise>
    <xyzOffsets>0 0 0</xyzOffsets>
    <rpyOffsets>0 0 0</rpyOffsets>
    <interface:position name="imu_position"/>
  </controller:gazebo_ros_imu>
</gazebo>

<!--
  <gazebo reference="imu_link">
    
    
    <sensor name="imu_sensor" type="imu">
      <always_on>1</always_on>
      <update_rate>1000.0</update_rate>
      <imu>
        <noise>
          <type>gaussian</type>
          
          <rate>
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </rate>
          <accel>
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
    </sensor>
  </gazebo>
-->

<!--
http://answers.ros.org/question/12430/modelling-sensorsimu-in-gazebo/
-->
~~~

# Bumper

~~~
<!-- BAMPER -->
<!--
<gazebo>
  <plugin name="${name}_gazebo_ros_bumper_controller" filename="libgazebo_ros_bumper.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>${update_rate}</updateRate>
    <bumperTopicName>${name}_bumper</bumperTopicName>
    <frameName>world</frameName>
  </plugin>
</gazebo>
-->
~~~

http://answers.gazebosim.org/question/3877/contact-sensor-no-data-output/
http://answers.gazebosim.org/question/5355/adding-ros-integrated-contact-sensors/
http://answers.ros.org/question/29158/how-do-i-use-force-sensor-bumper-sensor-in-gazebo/

# Laser

~~~
<!--
    http://gazebosim.org/wiki/Tutorials/1.9/ROS_Motor_and_Sensor_Plugins
-->
<!--
<sensor type="ray" name="head_hokuyo_sensor">
<plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
-->
~~~



# Camera


http://answers.ros.org/question/61712/how-to-use-libgazebo_ros_cameraso-in-gazebo-urdf/

~~~
<!-- STEREOCAM -->
<gazebo reference="wide_stereo_gazebo_l_stereo_camera_frame">
  <sensor:camera name="wide_stereo_gazebo_l_stereo_camera_sensor">
    <imageSize>640 480</imageSize>
    <imageFormat>BAYER_BGGR8</imageFormat>
    <hfov>90</hfov>
    <nearClip>0.1</nearClip>
    <farClip>100</farClip>
    <updateRate>25.0</updateRate>
    <controller:gazebo_ros_camera name="wide_stereo_gazebo_l_stereo_camera_controller" plugin="libgazebo_ros_camera.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>25.0</updateRate>
      <imageTopicName>wide_stereo/left/image_raw</imageTopicName>
      <cameraInfoTopicName>wide_stereo/left/camera_info</cameraInfoTopicName>
      <frameName>wide_stereo_optical_frame</frameName>
      <hackBaseline>0</hackBaseline>
      <CxPrime>320.5</CxPrime>
      <Cx>320.5</Cx>
      <Cy>240.5</Cy>
      <!-- image_width / (2*tan(hfov_radian /2)) -->
      <!-- 320 for wide and 772.55 for narrow stereo camera -->
      <focal_length>320</focal_length>
      <distortion_k1>0.00000001</distortion_k1>
      <distortion_k2>0.00000001</distortion_k2>
      <distortion_k3>0.00000001</distortion_k3>
      <distortion_t1>0.00000001</distortion_t1>
      <distortion_t2>0.00000001</distortion_t2>
      <interface:camera name="wide_stereo_gazebo_l_stereo_camera_iface"/>
    </controller:gazebo_ros_camera>
  </sensor:camera>
  <turnGravityOff>true</turnGravityOff>
  <material>PR2/Blue</material>
</gazebo>


  <joint name="camera_joint" type="fixed">
    <axis xyz="1 0 0"/> <!-- 0 1 0 -->
    <origin xyz="0 0 0.2"/>
    <parent link="base_link"/>
    <child link="camera_link"/>
  </joint>

    <link name="camera_link">
      <inertial>
        <mass value="0.001" />
        <origin xyz="0 0 0" rpy="0 0 0" />
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001" />
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <box size="0.01 0.01 0.01" />
        </geometry>
        <material name="Blue">
          <color rgba="0.0 0.0 0.8 1"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0" />
        <geometry>
          <box size="0.01 0.01 0.01" />
        </geometry>
      </collision>
    </link>

    
    <joint name="camera_optical_joint" type="fixed">
      <origin xyz="0 0 0" rpy="-1.570796 0.0 -1.570796" />
      <parent link="camera_link" />
      <child link="camera_optical_frame"/>
    </joint>
  
    <link name="camera_optical_frame"/>
    
    <gazebo reference="camera_link">
      <sensor type="camera" name="camera_camera_sensor">
        <update_rate>15</update_rate>
        <camera>
          <horizontal_fov>90</horizontal_fov>
          <image>
            <format>BAYER_BGGR8</format>
            <width>320</width>
            <height>240</height>
          </image>
          <clip>
            <near>0.01</near>
            <far>100</far>
          </clip>
        </camera>

        <plugin name="camera_camera_controller" filename="libgazebo_ros_camera.so">
          <cameraName>camera</cameraName>
          <alwaysOn>true</alwaysOn>
          <updateRate>15</updateRate>
          <imageTopicName>left/image_raw</imageTopicName>
          <cameraInfoTopicName>left/camera_info</cameraInfoTopicName>
          <frameName>camera_optical_frame</frameName>
        </plugin>
      </sensor>
    </gazebo>
~~~
 
    
~~~ 
    <gazebo reference="camera_link">
      <sensor type="camera" name="camera_camera_sensor">
        <update_rate>15</update_rate>
        <camera>
          <horizontal_fov angle='1.57079633'/>
          <image>
            <format>R8G8B8</format>
            <width>320</width>
            <height>240</height>
          </image>
          <clip>
            <near>0.01</near>
            <far>100</far>
          </clip>
        </camera>

        <plugin name="camera_camera_controller" filename="libgazebo_ros_camera.so">
          <cameraName>camera</cameraName>
          <alwaysOn>true</alwaysOn>
          <updateRate>15</updateRate>
          <imageTopicName>left/image_raw</imageTopicName>
          <cameraInfoTopicName>left/camera_info</cameraInfoTopicName>
          <frameName>camera_optical_frame</frameName>
        </plugin>
      </sensor>
    </gazebo>
    
~~~
  


