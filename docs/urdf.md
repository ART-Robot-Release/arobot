# URDF文件简介 #

ROS [URDF Wiki](http://wiki.ros.org/urdf)

ARTrobot使用了简单的urdf做为机器人的描述，下面对这内容进行简单的解释。

位置：src/robot_description/urdf/robot.URDF

## Base link ##
```
  <pose>0 0 0.94 0.6 0.3 0.8</pose>
  <link name="base_link"/>
  <joint name="base_link_to_body" type="fixed">
      <parent link="base_link"/>
      <child link="body_link"/>
  </joint>
```

base_link是gazebo仿真的需求，base_link是一个虚的节点。

## links ##
```
   <link
     name="body_link">
     <inertial>
       <origin
         xyz="-0.0152253576123707 0.00169980852650529 -0.131240947917013"
         rpy="0 0 0" />
       <mass
         value="9.8555949066319" />
       <inertia
         ixx="0.230608536348037"
         ixy="0.000104353859531529"
         ixz="-0.00476072524142725"
         iyy="0.155367759536467"
         iyz="-0.0019556519060449"
         izz="0.131078480811617" />
     </inertial>
     <visual>
       <origin
         xyz="0 0 0"
         rpy="0 0 0" />
       <geometry>
         <mesh
           filename="package://robot_description/meshes/robot/body_link.STL" />
       </geometry>
       <material
         name="">
         <color
           rgba="1 1 1 1" />
       </material>
     </visual>
     <collision>
       <origin
         xyz="0 0 0"
         rpy="0 0 0" />
       <geometry>
         <mesh
           filename="package://robot_description/meshes/robot/body_link.STL" />
       </geometry>
     </collision>
   </link>
```

每一个link代表了机器人上的各个部分的机械结构，采用DH法建立来描述依赖关系。从上文中可见，其中包括机械属性，视觉属性，碰撞属性等。

## joint 关节 ##
```
   <joint
     name="neck_yaw"
     type="revolute">
     <origin
       xyz="-0.00622802770076672 0.000501513148428171 0.0740732491200115"
       rpy="3.05771063523091 -1.5605576802669E-16 1.5707963267949" />
     <parent
       link="body_link" />
     <child
       link="neck_link" />
     <axis
       xyz="0 -0.0837836846148982 0.996483965848097" />
   <limit lower="-3" upper="3" effort="100" velocity="5.0"/>
 	</joint>
```

joint用来描述关节之间的连接关系，详细的描述可参考官方文档。

## Gazebo ##
```
<gazebo>
	....
</gazebo>
```

之间描述的是gazebo([gazebo urdf](http://gazebosim.org/tutorials/?tut=ros_urdf))的信息，其中重要的是关节和传感器。

### gazebo 与 ros-control ###

```
	<transmission name="tran_neck_yaw">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="neck_yaw">
          <hardwareInterface>EffortJointInterface</hardwareInterface>
      </joint>
      <actuator name="act_neck_yaw">
        <hardwareInterface>EffortJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>

```

关节上的驱动接口描述。

### Gazebo 与 传感器插件 ###

六维力传感器描述。

```
    <gazebo>
    	<plugin name="ft_sensor" filename="libgazebo_ros_ft_sensor.so">
			<robotNamespace>/arobot</robotNamespace>
    		<updateRate>10</updateRate>
    		<topicName>/arobot/ft_sensor_left</topicName>
   		 	<jointName>l_ankle_roll</jointName>
    	</plugin>
    </gazebo>
```

这些接口将通过gazebo_ros_pkgs/gazebo_ros_control的插件（注：在ART下代版本中，这个会重构到ARTrobot的系统中）

gazebo_ros_control会通过ros平台的topic或service机制发送和接受信息。（这样做是方便ros各种功能插件）

> 注：ARTrobot的实机会基于控制高实时性的考虑，在控制循环中，读写关节和读写传感器，并存储到数据库中。（可以通过ros在仿真中模拟这套机）在重构的仿真控制中，将会考虑讲二者完全一致化。