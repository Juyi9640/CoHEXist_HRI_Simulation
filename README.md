# CoHEXist_HRI_Simulation

A Gazebo-based simulation environment replicating the [CoHEXist][3] setup was created to evaluate industrial human-robot interactions (HRI). [actor_collisions][11] plugin allowed actor to have collision properties. [GMapping][4] + [AMCL][5] was selected as 2D LiDAR-based mapping and localization strategies due to its low computational load and high stability. The ROS [move_base][6] framework served as the core navigation system, employing the Dijkstra algorithm as global planner and Time Elastic Band ([TEB][7]) as local planner, and the costmap-based mechanism for obstacle avoidance. The [waterplus_map_tools][9] was used to enable robot waypoint navigation. [leg_tracker][8] package was used to detect People messages and the [social_navigation_layers][10] for social-aware navigation was tested.<br />

### System requirements
[Ubuntu 20.04][1] and [ROS Noetic][2]

[1]: https://releases.ubuntu.com/focal/                                          "Ubuntu 20.04"
[2]: http://wiki.ros.org/noetic/Installation/Ubuntu                              "ROS Noetic"
[3]: https://www.researchgate.net/profile/Sergey-Yurish/publication/378144846_Proceedings_of_the_4th_IFSA_Winter_Conference_on_Automation_Robotics_and_Communications_for_Industry_4050_ARCI_2024/links/65c9fe381bed776ae34ac345/Proceedings-of-the-4th-IFSA-Winter-Conference-on-Automation-Robotics-and-Communications-for-Industry-40-50-ARCI-2024.pdf#page=280         "CoHEXist"
[4]: https://openslam-org.github.io/gmapping.html                                "GMapping"
[5]: https://wiki.ros.org/amcl                                                   "AMCL"
[6]: https://wiki.ros.org/move_base                                              "move_base"
[7]: https://wiki.ros.org/teb_local_planner                                      "TEB"
[8]: https://github.com/angusleigh/leg_tracker                                   "leg_tracker"
[9]: https://github.com/6-robot/waterplus_map_tools                              "waterplus_map_tools"
[10]: https://wiki.ros.org/social_navigation_layers                              "social_navigation_layers"
[11]: https://github.com/JiangweiNEU/actor_collisions                            "actor_collisions"


### Results

Given the same goal pose (position and orientation extracted from RViz), the following images show the 
qualitative comparison between the results: <br />
(first: Jacobian transpose; second: Pseudoinverse; third: Damped least squares)

<div align="center">
  <p style="text-align:center;">IKs with the first goal pose</p>

  <img src="gif folder/11.gif" width="300" alt="First GIF" />
  <img src="gif folder/21.gif" width="300" alt="Second GIF" />
  <img src="gif folder/31.gif" width="300" alt="Third GIF" />

  <p style="text-align:center;">IKs with the second goal pose</p>

  <img src="gif folder/12.gif" width="300" alt="First GIF" />
  <img src="gif folder/22.gif" width="300" alt="Second GIF" />
  <img src="gif folder/32.gif" width="300" alt="Third GIF" />

</div>























