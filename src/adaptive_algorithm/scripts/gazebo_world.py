import random
import cv2
import numpy as np

# 横纵坐标的列表
c_value_x_list = []
c_value_y_list = []
model_radius = 0.267
resolution = 0.05
map_shape = 200
cylinder_number = 10
c_value_list = {"c_value_x_list":c_value_x_list,"c_value_y_list":c_value_y_list}
for j in range(200):
    c_value_x_list.clear()
    c_value_y_list.clear()
    for i in range(cylinder_number):
        c_value_x = random.uniform(1, 9)
        c_value_y = random.uniform(1, 9)
        while (c_value_x > 1.5 and c_value_x < 2.5 and c_value_y > 0.5 and c_value_y < 1.5) or \
            (c_value_x > 7.5 and c_value_x < 8.5 and c_value_y > 7.5 and c_value_y < 8.5) :
            c_value_x = random.uniform(1, 9)
            c_value_y = random.uniform(1, 9)
        c_value_x_list.append(c_value_x)
        c_value_y_list.append(c_value_y)

    emptyImage3 = np.zeros((map_shape, map_shape), np.uint8)
    emptyImage3[...] = 255
    cv2.line(img=emptyImage3, pt1=(1, 1), pt2=(1, 199), color=1, thickness=2)
    cv2.line(img=emptyImage3, pt1=(1, 1), pt2=(199, 1), color=1, thickness=2)
    cv2.line(img=emptyImage3, pt1=(199, 199), pt2=(1, 199), color=1, thickness=2)
    cv2.line(img=emptyImage3, pt1=(199, 199), pt2=(199, 1), color=1, thickness=2)
    for i in range(cylinder_number):
        print(c_value_list['c_value_x_list'][i])
        print(c_value_list['c_value_y_list'][i])
        cv2.circle(emptyImage3, (round(c_value_list['c_value_x_list'][i]/resolution), map_shape-(round(c_value_list['c_value_y_list'][i]/resolution))), radius=5, color=1, thickness=2)
        print((c_value_list['c_value_x_list'][i]/resolution))
        print(map_shape-((c_value_list['c_value_y_list'][i]/resolution)))
        print((round(c_value_list['c_value_x_list'][i]/resolution)))
        print(map_shape-(round(c_value_list['c_value_y_list'][i]/resolution)))
        print()
    print(c_value_list)

    msg = "<sdf version='1.6'>\n\
      <world name='default'>\n\
        <light name='sun' type='directional'>\n\
          <cast_shadows>1</cast_shadows>\n\
          <pose frame=''>0 0 10 0 -0 0</pose>\n\
          <diffuse>0.8 0.8 0.8 1</diffuse>\n\
          <specular>0.1 0.1 0.1 1</specular>\n\
          <attenuation>\n\
            <range>1000</range>\n\
            <constant>0.9</constant>\n\
            <linear>0.01</linear>\n\
            <quadratic>0.001</quadratic>\n\
          </attenuation>\n\
          <direction>-0.5 0.5 -1</direction>\n\
        </light>\n\
        <model name='ground_plane'>\n\
          <static>1</static>\n\
          <link name='link'>\n\
            <collision name='collision'>\n\
              <geometry>\n\
                <plane>\n\
                  <normal>0 0 1</normal>\n\
                  <size>100 100</size>\n\
                </plane>\n\
              </geometry>\n\
              <surface>\n\
                <friction>\n\
                  <ode>\n\
                    <mu>100</mu>\n\
                    <mu2>50</mu2>\n\
                  </ode>\n\
                  <torsional>\n\
                    <ode/>\n\
                  </torsional>\n\
                </friction>\n\
                <contact>\n\
                  <ode/>\n\
                </contact>\n\
                <bounce/>\n\
              </surface>\n\
              <max_contacts>10</max_contacts>\n\
            </collision>\n\
            <visual name='visual'>\n\
              <cast_shadows>0</cast_shadows>\n\
              <geometry>\n\
                <plane>\n\
                  <normal>0 0 1</normal>\n\
                  <size>100 100</size>\n\
                </plane>\n\
              </geometry>\n\
              <material>\n\
                <script>\n\
                  <uri>file://media/materials/scripts/gazebo.material</uri>\n\
                  <name>Gazebo/Grey</name>\n\
                </script>\n\
              </material>\n\
            </visual>\n\
            <self_collide>0</self_collide>\n\
            <kinematic>0</kinematic>\n\
            <gravity>1</gravity>\n\
          </link>\n\
        </model>\n\
        <gravity>0 0 -9.8</gravity>\n\
        <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>\n\
        <atmosphere type='adiabatic'/>\n\
        <physics name='default_physics' default='0' type='ode'>\n\
          <max_step_size>0.001</max_step_size>\n\
          <real_time_factor>5</real_time_factor>\n\
          <real_time_update_rate>5000</real_time_update_rate>\n\
        </physics>\n\
        <scene>\n\
          <ambient>0.4 0.4 0.4 1</ambient>\n\
          <background>0.7 0.7 0.7 1</background>\n\
          <shadows>1</shadows>\n\
        </scene>\n\
        <spherical_coordinates>\n\
          <surface_model>EARTH_WGS84</surface_model>\n\
          <latitude_deg>0</latitude_deg>\n\
          <longitude_deg>0</longitude_deg>\n\
          <elevation>0</elevation>\n\
          <heading_deg>0</heading_deg>\n\
        </spherical_coordinates>\n\
        <model name='my_wall_01'>\n\
          <pose frame=''>4.84148 4.89701 0 0 -0 0</pose>\n\
          <link name='Wall_0'>\n\
            <collision name='Wall_0_Collision'>\n\
              <geometry>\n\
                <box>\n\
                  <size>9.95377 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <max_contacts>10</max_contacts>\n\
              <surface>\n\
                <contact>\n\
                  <ode/>\n\
                </contact>\n\
                <bounce/>\n\
                <friction>\n\
                  <torsional>\n\
                    <ode/>\n\
                  </torsional>\n\
                  <ode/>\n\
                </friction>\n\
              </surface>\n\
            </collision>\n\
            <visual name='Wall_0_Visual'>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <geometry>\n\
                <box>\n\
                  <size>9.95377 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <material>\n\
                <script>\n\
                  <uri>file://media/materials/scripts/gazebo.material</uri>\n\
                  <name>Gazebo/Wood</name>\n\
                </script>\n\
                <ambient>1 1 1 1</ambient>\n\
              </material>\n\
            </visual>\n\
            <pose frame=''>0.034695 -4.92526 0 0 0 -3.13923</pose>\n\
            <self_collide>0</self_collide>\n\
            <kinematic>0</kinematic>\n\
            <gravity>1</gravity>\n\
          </link>\n\
          <link name='Wall_1'>\n\
            <collision name='Wall_1_Collision'>\n\
              <geometry>\n\
                <box>\n\
                  <size>10.0004 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <max_contacts>10</max_contacts>\n\
              <surface>\n\
                <contact>\n\
                  <ode/>\n\
                </contact>\n\
                <bounce/>\n\
                <friction>\n\
                  <torsional>\n\
                    <ode/>\n\
                  </torsional>\n\
                  <ode/>\n\
                </friction>\n\
              </surface>\n\
            </collision>\n\
            <visual name='Wall_1_Visual'>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <geometry>\n\
                <box>\n\
                  <size>10.0004 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <material>\n\
                <script>\n\
                  <uri>file://media/materials/scripts/gazebo.material</uri>\n\
                  <name>Gazebo/Wood</name>\n\
                </script>\n\
                <ambient>1 1 1 1</ambient>\n\
              </material>\n\
            </visual>\n\
            <pose frame=''>-4.91343 0.011303 0 0 -0 1.58019</pose>\n\
            <self_collide>0</self_collide>\n\
            <kinematic>0</kinematic>\n\
            <gravity>1</gravity>\n\
          </link>\n\
          <link name='Wall_2'>\n\
            <collision name='Wall_2_Collision'>\n\
              <geometry>\n\
                <box>\n\
                  <size>9.95374 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <max_contacts>10</max_contacts>\n\
              <surface>\n\
                <contact>\n\
                  <ode/>\n\
                </contact>\n\
                <bounce/>\n\
                <friction>\n\
                  <torsional>\n\
                    <ode/>\n\
                  </torsional>\n\
                  <ode/>\n\
                </friction>\n\
              </surface>\n\
            </collision>\n\
            <visual name='Wall_2_Visual'>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <geometry>\n\
                <box>\n\
                  <size>9.95374 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <material>\n\
                <script>\n\
                  <uri>file://media/materials/scripts/gazebo.material</uri>\n\
                  <name>Gazebo/Wood</name>\n\
                </script>\n\
                <ambient>1 1 1 1</ambient>\n\
              </material>\n\
            </visual>\n\
            <pose frame=''>-0.057827 4.9363 0 0 -0 0</pose>\n\
            <self_collide>0</self_collide>\n\
            <kinematic>0</kinematic>\n\
            <gravity>1</gravity>\n\
          </link>\n\
          <link name='Wall_3'>\n\
            <collision name='Wall_3_Collision'>\n\
              <geometry>\n\
                <box>\n\
                  <size>10.0004 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <max_contacts>10</max_contacts>\n\
              <surface>\n\
                <contact>\n\
                  <ode/>\n\
                </contact>\n\
                <bounce/>\n\
                <friction>\n\
                  <torsional>\n\
                    <ode/>\n\
                  </torsional>\n\
                  <ode/>\n\
                </friction>\n\
              </surface>\n\
            </collision>\n\
            <visual name='Wall_3_Visual'>\n\
              <pose frame=''>0 0 1.25 0 -0 0</pose>\n\
              <geometry>\n\
                <box>\n\
                  <size>10.0004 0.15 2.5</size>\n\
                </box>\n\
              </geometry>\n\
              <material>\n\
                <script>\n\
                  <uri>file://media/materials/scripts/gazebo.material</uri>\n\
                  <name>Gazebo/Wood</name>\n\
                </script>\n\
                <ambient>1 1 1 1</ambient>\n\
              </material>\n\
            </visual>\n\
            <pose frame=''>4.91343 0.011303 0 0 0 -1.5614</pose>\n\
            <self_collide>0</self_collide>\n\
            <kinematic>0</kinematic>\n\
            <gravity>1</gravity>\n\
          </link>\n\
          <static>1</static>\n\
        </model>\n\
        <state world_name='default'>\n\
          <sim_time>1639 151000000</sim_time>\n\
          <real_time>4470 492795205</real_time>\n\
          <wall_time>1576044024 212025517</wall_time>\n\
          <iterations>1639151</iterations>\n\
          <model name='ground_plane'>\n\
            <pose frame=''>0 0 0 0 -0 0</pose>\n\
            <scale>1 1 1</scale>\n\
            <link name='link'>\n\
              <pose frame=''>0 0 0 0 -0 0</pose>\n\
              <velocity>0 0 0 0 -0 0</velocity>\n\
              <acceleration>0 0 0 0 -0 0</acceleration>\n\
              <wrench>0 0 0 0 -0 0</wrench>\n\
            </link>\n\
          </model>\n\
          <model name='my_wall_01'>\n\
            <pose frame=''>4.84148 4.89701 0 0 0 -0.007021</pose>\n\
            <scale>1 1 1</scale>\n\
            <link name='Wall_0'>\n\
              <pose frame=''>4.84159 -0.028372 0 0 -0 3.13693</pose>\n\
              <velocity>0 0 0 0 -0 0</velocity>\n\
              <acceleration>0 0 0 0 -0 0</acceleration>\n\
              <wrench>0 0 0 0 -0 0</wrench>\n\
            </link>\n\
            <link name='Wall_1'>\n\
              <pose frame=''>-0.07175 4.94281 0 0 -0 1.57317</pose>\n\
              <velocity>0 0 0 0 -0 0</velocity>\n\
              <acceleration>0 0 0 0 -0 0</acceleration>\n\
              <wrench>0 0 0 0 -0 0</wrench>\n\
            </link>\n\
            <link name='Wall_2'>\n\
              <pose frame=''>4.81831 9.83359 0 0 0 -0.007021</pose>\n\
              <velocity>0 0 0 0 -0 0</velocity>\n\
              <acceleration>0 0 0 0 -0 0</acceleration>\n\
              <wrench>0 0 0 0 -0 0</wrench>\n\
            </link>\n\
            <link name='Wall_3'>\n\
              <pose frame=''>9.75487 4.87381 0 0 0 -1.56842</pose>\n\
              <velocity>0 0 0 0 -0 0</velocity>\n\
              <acceleration>0 0 0 0 -0 0</acceleration>\n\
              <wrench>0 0 0 0 -0 0</wrench>\n\
            </link>\n\
          </model>\n\
          <model name='c1'>\n\
            <pose frame=''>" + str(c_value_list['c_value_x_list'][0]) + " "+ str(c_value_list['c_value_y_list'][0]) + " 0 0 -0 0</pose>\n\
            <scale>1 1 1</scale>\n\
            <link name='link'>\n\
              <pose frame=''>" + str(c_value_list['c_value_x_list'][0]) + " "+ str(c_value_list['c_value_y_list'][0]) + " 0 0 -0 0</pose>\n\
              <velocity>0 0 0 0 -0 0</velocity>\n\
              <acceleration>0 0 0 0 -0 0</acceleration>\n\
              <wrench>0 0 0 0 -0 0</wrench>\n\
            </link>\n\
          </model>\n\
          <model name='c2'>\n\
            <pose frame=''>" + str(c_value_list['c_value_x_list'][1]) + " "+ str(c_value_list['c_value_y_list'][1]) + " 0 0 -0 0</pose>\n\
            <scale>1 1 1</scale>\n\
            <link name='link'>\n\
              <pose frame=''>" + str(c_value_list['c_value_x_list'][1]) + " "+ str(c_value_list['c_value_y_list'][1]) + " 0 0 -0 0</pose>\n\
              <velocity>0 0 0 0 -0 0</velocity>\n\
              <acceleration>0 0 0 0 -0 0</acceleration>\n\
              <wrench>0 0 0 0 -0 0</wrench>\n\
            </link>\n\
          </model>\n\
    <model name='c3'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][2]) + " "+ str(c_value_list['c_value_y_list'][2]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][2]) + " "+ str(c_value_list['c_value_y_list'][2]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
    <model name='c4'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][3]) + " "+ str(c_value_list['c_value_y_list'][3]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][3]) + " "+ str(c_value_list['c_value_y_list'][3]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
    <model name='c5'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][4]) + " "+ str(c_value_list['c_value_y_list'][4]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][4]) + " "+ str(c_value_list['c_value_y_list'][4]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
    <model name='c6'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][5]) + " "+ str(c_value_list['c_value_y_list'][5]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][5]) + " "+ str(c_value_list['c_value_y_list'][5]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
    <model name='c7'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][6]) + " "+ str(c_value_list['c_value_y_list'][6]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][6]) + " "+ str(c_value_list['c_value_y_list'][6]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
    <model name='c8'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][7]) + " "+ str(c_value_list['c_value_y_list'][7]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][7]) + " "+ str(c_value_list['c_value_y_list'][7]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
    <model name='c9'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][8]) + " "+ str(c_value_list['c_value_y_list'][8]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][8]) + " "+ str(c_value_list['c_value_y_list'][8]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
    <model name='c10'>\n\
      <pose frame=''>" + str(c_value_list['c_value_x_list'][9]) + " "+ str(c_value_list['c_value_y_list'][9]) + " 0 0 -0 0</pose>\n\
      <scale>1 1 1</scale>\n\
      <link name='link'>\n\
        <pose frame=''>" + str(c_value_list['c_value_x_list'][9]) + " "+ str(c_value_list['c_value_y_list'][9]) + " 0 0 -0 0</pose>\n\
        <velocity>0 0 0 0 -0 0</velocity>\n\
        <acceleration>0 0 0 0 -0 0</acceleration>\n\
        <wrench>0 0 0 0 -0 0</wrench>\n\
      </link>\n\
    </model>\n\
          <light name='sun'>\n\
            <pose frame=''>0 0 10 0 -0 0</pose>\n\
          </light>\n\
        </state>\n\
        <model name='c1'>\n\
          <link name='link'>\n\
            <pose frame=''>0 0 0 0 -0 0</pose>\n\
            <inertial>\n\
              <mass>0.284858</mass>\n\
              <inertia>\n\
                <ixx>0.0288096</ixx>\n\
                <ixy>0</ixy>\n\
                <ixz>0</ixz>\n\
                <iyy>0.0288096</iyy>\n\
                <iyz>0</iyz>\n\
                <izz>0.010143</izz>\n\
              </inertia>\n\
              <pose frame=''>0 0 0 0 -0 0</pose>\n\
            </inertial>\n\
            <self_collide>0</self_collide>\n\
            <kinematic>0</kinematic>\n\
            <gravity>1</gravity>\n\
            <visual name='visual'>\n\
              <geometry>\n\
                <cylinder>\n\
                  <radius>0.266862</radius>\n\
                  <length>1</length>\n\
                </cylinder>\n\
              </geometry>\n\
              <material>\n\
                <script>\n\
                  <name>Gazebo/Grey</name>\n\
                  <uri>file://media/materials/scripts/gazebo.material</uri>\n\
                </script>\n\
                <ambient>0.3 0.3 0.3 1</ambient>\n\
                <diffuse>0.7 0.7 0.7 1</diffuse>\n\
                <specular>0.01 0.01 0.01 1</specular>\n\
                <emissive>0 0 0 1</emissive>\n\
                <shader type='vertex'>\n\
                  <normal_map>__default__</normal_map>\n\
                </shader>\n\
              </material>\n\
              <pose frame=''>0 0 0 0 -0 0</pose>\n\
              <cast_shadows>1</cast_shadows>\n\
              <transparency>0</transparency>\n\
            </visual>\n\
            <collision name='collision'>\n\
              <laser_retro>0</laser_retro>\n\
              <max_contacts>10</max_contacts>\n\
              <pose frame=''>0 0 0 0 -0 0</pose>\n\
              <geometry>\n\
                <cylinder>\n\
                  <radius>0.266862</radius>\n\
                  <length>1</length>\n\
                </cylinder>\n\
              </geometry>\n\
              <surface>\n\
                <friction>\n\
                  <ode>\n\
                    <mu>1</mu>\n\
                    <mu2>1</mu2>\n\
                    <fdir1>0 0 0</fdir1>\n\
                    <slip1>0</slip1>\n\
                    <slip2>0</slip2>\n\
                  </ode>\n\
                  <torsional>\n\
                    <coefficient>1</coefficient>\n\
                    <patch_radius>0</patch_radius>\n\
                    <surface_radius>0</surface_radius>\n\
                    <use_patch_radius>1</use_patch_radius>\n\
                    <ode>\n\
                      <slip>0</slip>\n\
                    </ode>\n\
                  </torsional>\n\
                </friction>\n\
                <bounce>\n\
                  <restitution_coefficient>0</restitution_coefficient>\n\
                  <threshold>1e+06</threshold>\n\
                </bounce>\n\
                <contact>\n\
                  <collide_without_contact>0</collide_without_contact>\n\
                  <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
                  <collide_bitmask>1</collide_bitmask>\n\
                  <ode>\n\
                    <soft_cfm>0</soft_cfm>\n\
                    <soft_erp>0.2</soft_erp>\n\
                    <kp>1e+13</kp>\n\
                    <kd>1</kd>\n\
                    <max_vel>0.01</max_vel>\n\
                    <min_depth>0</min_depth>\n\
                  </ode>\n\
                  <bullet>\n\
                    <split_impulse>1</split_impulse>\n\
                    <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                    <soft_cfm>0</soft_cfm>\n\
                    <soft_erp>0.2</soft_erp>\n\
                    <kp>1e+13</kp>\n\
                    <kd>1</kd>\n\
                  </bullet>\n\
                </contact>\n\
              </surface>\n\
            </collision>\n\
          </link>\n\
          <static>1</static>\n\
          <allow_auto_disable>1</allow_auto_disable>\n\
          <pose frame=''>" + str(c_value_list['c_value_x_list'][0]) + " "+ str(c_value_list['c_value_y_list'][0]) + " 0 0 -0 0</pose>\n\
        </model>\n\
    <model name='c2'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][1]) + " "+ str(c_value_list['c_value_y_list'][1]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c3'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision> \n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][2]) + " "+ str(c_value_list['c_value_y_list'][2]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c4'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][3]) + " "+ str(c_value_list['c_value_y_list'][3]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c5'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][4]) + " "+ str(c_value_list['c_value_y_list'][4]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c6'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][5]) + " "+ str(c_value_list['c_value_y_list'][5]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c7'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][6]) + " "+ str(c_value_list['c_value_y_list'][6]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c8'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][7]) + " "+ str(c_value_list['c_value_y_list'][7]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c9'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][8]) + " "+ str(c_value_list['c_value_y_list'][8]) + " 0 0 -0 0</pose>\n\
     </model>\n\
    <model name='c10'>\n\
       <link name='link'>\n\
         <pose frame=''>0 0 0 0 -0 0</pose>\n\
         <inertial>\n\
           <mass>0.284858</mass>\n\
           <inertia>\n\
             <ixx>0.0288096</ixx>\n\
             <ixy>0</ixy>\n\
             <ixz>0</ixz>\n\
             <iyy>0.0288096</iyy>\n\
             <iyz>0</iyz>\n\
             <izz>0.010143</izz>\n\
           </inertia>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
         </inertial>\n\
         <self_collide>0</self_collide>\n\
         <kinematic>0</kinematic>\n\
         <gravity>1</gravity>\n\
         <visual name='visual'>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <material>\n\
             <script>\n\
               <name>Gazebo/Grey</name>\n\
               <uri>file://media/materials/scripts/gazebo.material</uri>\n\
             </script>\n\
             <ambient>0.3 0.3 0.3 1</ambient>\n\
             <diffuse>0.7 0.7 0.7 1</diffuse>\n\
             <specular>0.01 0.01 0.01 1</specular>\n\
             <emissive>0 0 0 1</emissive>\n\
             <shader type='vertex'>\n\
               <normal_map>__default__</normal_map>\n\
             </shader>\n\
           </material>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <cast_shadows>1</cast_shadows>\n\
           <transparency>0</transparency>\n\
         </visual>\n\
         <collision name='collision'>\n\
           <laser_retro>0</laser_retro>\n\
           <max_contacts>10</max_contacts>\n\
           <pose frame=''>0 0 0 0 -0 0</pose>\n\
           <geometry>\n\
             <cylinder>\n\
               <radius>0.266862</radius>\n\
               <length>1</length>\n\
             </cylinder>\n\
           </geometry>\n\
           <surface>\n\
             <friction>\n\
               <ode>\n\
                 <mu>1</mu>\n\
                 <mu2>1</mu2>\n\
                 <fdir1>0 0 0</fdir1>\n\
                 <slip1>0</slip1>\n\
                 <slip2>0</slip2>\n\
               </ode>\n\
               <torsional>\n\
                 <coefficient>1</coefficient>\n\
                 <patch_radius>0</patch_radius>\n\
                 <surface_radius>0</surface_radius>\n\
                 <use_patch_radius>1</use_patch_radius>\n\
                 <ode>\n\
                   <slip>0</slip>\n\
                 </ode>\n\
               </torsional>\n\
             </friction>\n\
             <bounce>\n\
               <restitution_coefficient>0</restitution_coefficient>\n\
               <threshold>1e+06</threshold>\n\
             </bounce>\n\
             <contact>\n\
               <collide_without_contact>0</collide_without_contact>\n\
               <collide_without_contact_bitmask>1</collide_without_contact_bitmask>\n\
               <collide_bitmask>1</collide_bitmask>\n\
               <ode>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
                 <max_vel>0.01</max_vel>\n\
                 <min_depth>0</min_depth>\n\
               </ode>\n\
               <bullet>\n\
                 <split_impulse>1</split_impulse>\n\
                 <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>\n\
                 <soft_cfm>0</soft_cfm>\n\
                 <soft_erp>0.2</soft_erp>\n\
                 <kp>1e+13</kp>\n\
                 <kd>1</kd>\n\
               </bullet>\n\
             </contact>\n\
           </surface>\n\
         </collision>\n\
       </link>\n\
       <static>1</static>\n\
       <allow_auto_disable>1</allow_auto_disable>\n\
       <pose frame=''>" + str(c_value_list['c_value_x_list'][9]) + " "+ str(c_value_list['c_value_y_list'][9]) + " 0 0 -0 0</pose>\n\
     </model>\n\
        <gui fullscreen='0'>\n\
          <camera name='user_camera'>\n\
            <pose frame=''>7.46544 4.75826 14.2026 4e-06 1.5658 1.56416</pose>\n\
            <view_controller>orbit</view_controller>\n\
            <projection_type>perspective</projection_type>\n\
          </camera>\n\
        </gui>\n\
      </world>\n\
    </sdf>\n\
    "
    cv2.imwrite('/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_10/world_cylinder_10_'+ str(j) + '.png', emptyImage3)
    yaml_file = "image: '/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_10/world_cylinder_10_"+ str(j) + ".png \nresolution: 0.050000\norigin: [0.000000, 0.000000, 0.000000]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    with open('/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_10/world_cylinder_10_' + str(j) + '.yaml', 'w') as f:
        f.write(yaml_file)
    with open( '/home/zlb/dev/ros_collections/catkin_ws_adaptive_switch_algorithm/src/adaptive_algorithm/maps/slim_cylinder_10/world_cylinder_10_'+ str(j) + '.world', 'w' ) as f:
        f.write(msg)