# Nuturtle Description
URDF files for Nuturtle Burger

* `ros2 launch nuturtle_description load_one.launch.xml` to see the robot in rviz.
* `ros2 launch nuturtle_description load_all.launch.xml` to see four copies of the robot in rviz.

    ![RViz Visualization](images/rviz.png)

* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:

    ![rqt_graph](images/rqt_graph.svg)

# Launch File Details
* `ros2 launch nuturtle_description load_one.launch.xml --show-args`

    Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        no description given
        (default: 'true')

    'use_jsp':
        no description given
        (default: 'true')

    'color':
        One of: ['purple', 'blue', 'green', 'red']
        (default: 'purple')

* `ros2 launch nuturtle_description load_all.launch.xml --show-args`

    Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        no description given
        (default: 'true')

    'use_jsp':
        no description given
        (default: 'true')

    'color':
        One of: ['purple', 'blue', 'green', 'red']
        (default: 'purple')