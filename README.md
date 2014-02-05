# seekurjr model sim

~~~{.bash}
sudo apt-get install ros-groovy-pcl-conversions
sudo apt-get install ros-groovy-control-msgs
~~~



~~~{.bash}
source ~/rosws/setup.bash
source devel/setup.bash
localmaster

~~~

~~~{.bash}
roscore

rosrun gazebo_ros gazebo
~~~

~~~{.bash}
rosrun gazebo_ros spawn_model -file seekurjr.urdf -urdf -z 1 -model seekurjr

rosrun gazebo_ros spawn_model -file src/seekurjr_gazebo/urdf/seekurjr.urdf -urdf -z 1 -model seekurjr

roslaunch seekurjr_gazebo display.xml model:=src/seekurjr_gazebo/urdf/seekurjr.urdf
~~~



According to 
[Wikipedia](http://en.wikipedia.org/wiki/List_of_moment_of_inertia_tensors),
list of moment of inertia tensors:

~~~{.octave}
		m=14; h=0.19; w=0.4; d=0.5; 
		ixx = 1/12*m*(h^2+d^2)
		iyy = 1/12*m*(w^2+d^2) 
		izz = 1/12*m*(h^2+d^2) 
~~~

ROBOT

DIMENSIONS: 1050mm x 840mm x 500mm LxWxH
WEIGHT: 77kg (1 battery)
GROUND CLEARANCE: 105mm
TIRES: 400mm
WHEELBASE: 425mm 

~~~{.octave}
m=77; h=0.5; w=0.84; d=1.05; 
ixx = 1/12*m*(h^2+d^2)
iyy = 1/12*m*(w^2+d^2) 
izz = 1/12*m*(h^2+d^2) 
~~~

<!--
    http://gazebosim.org/wiki/Tutorials/1.9/ROS_Motor_and_Sensor_Plugins
-->
<!--
<sensor type="ray" name="head_hokuyo_sensor">
<plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
-->



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



<!--
http://answers.ros.org/question/30539/choosing-the-right-coefficients-for-gazebo-simulation/

http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model


http://answers.gazebosim.org/question/4372/the-inertia-matrix-explained/

Meshlab: seekurjr_body_new_low.stl
Mesh Volume is 155760256.000000
Mesh Surface is 4038439.750000
Thin shell barycenter 8.409724 247.537674 -3.138560
Center of Mass is 6.480556 131.015488 -45.919720
Inertia Tensor is :
| 11264990380032.000000 -130955894784.000000 -315909963776.000000 |
| -130955894784.000000 11355918696448.000000 -769257701376.000000 |
| -315909963776.000000 -769257701376.000000 5312563118080.000000 |
Principal axes are :
| 0.916011 -0.397463 0.054284 |
| 0.388693 0.912860 0.124917 |
| -0.099203 -0.093326 0.990681 |
axis momenta are :
| 11243634032640.000000 11491582410752.000000 5198255751168.000000 |
-->

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

<!-- SKID-STEERING CONTROLLER -->

<!-- 
http://answers.ros.org/question/61712/how-to-use-libgazebo_ros_cameraso-in-gazebo-urdf/
-->

<!--
<gazebo>
  <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">
    <updateRate>100.0</updateRate>
    <robotNamespace>/</robotNamespace>
    <leftFrontJoint>front_left_wheel_joint</leftFrontJoint>
    <rightFrontJoint>front_right_wheel_joint</rightFrontJoint>
    <leftRearJoint>back_left_wheel_joint</leftRearJoint>
    <rightRearJoint>back_right_wheel_joint</rightRearJoint>
    <wheelSeparation>0.4</wheelSeparation>
    <wheelDiameter>0.215</wheelDiameter>
    <robotBaseFrame>base_link</robotBaseFrame>
    <torque>20</torque>
    <topicName>cmd_vel</topicName>
    <broadcastTF>false</broadcastTF>
  </plugin>
</gazebo>
<gazebo>
  <plugin name="SkidSteerDrivePlugin" filename="libSkidSteerDrivePlugin.so">
      <right_front>p3at_front_right_wheel_joint</right_front>
      <right_rear>p3at_back_right_wheel_joint</right_rear>
      <left_front>p3at_front_left_wheel_joint</left_front>
      <left_rear>p3at_back_left_wheel_joint</left_rear>
      <max_force>5.0</max_force>
  </plugin>
</gazebo>
-->



  <inertia  ixx="7.8864" ixy="0"  ixz="0"  iyy="10.5430"  iyz="5.5721"  izz="5.5721" />


<!-- STEREOCAM -->

<!--
http://answers.ros.org/question/61712/how-to-use-libgazebo_ros_cameraso-in-gazebo-urdf/
-->


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

