# Potree_web_portal

In this study, Potree (http://www.potree.org) is adopted to create a prototype web portal. However, it lacks a built-in functionality for forward/backward projection between imagery and LiDAR data, as well as tools for displaying intensity profiles and lane width. These functions were developed within the Potree-based web portal. The forward/backward projection functions and intensity profile/lane width displaying tools are discussed in the following paragraphs.

## Forward/Backward projection ###

* The forward projection function projects a selected point from an image onto the corresponding LiDAR point cloud.
* The backward projection function projects an object point (red dot with a white-on-black label for the 3D coordinates) in a point cloud onto the corresponding images.

These projection functions enable users to visualize georeferenced imagery/LiDAR data captured simultaneously or at different times by the same or various MMS. Additionally, this projection function can be employed to assess the accuracy of trajectory and system calibration.

## Intensity profile/Lane width displaying tools ###

* The intensity profile displaying tool allows users to select a point of interest in a LiDAR intensity profile, which will then display the corresponding point on the web portal.
* The lane width displaying tool allows users to select a point of interest in lane width estimates, and then, the corresponding point pair (connected by two red dots with a white-on-black label for the lane width) on the point cloud will be shown on the web portal.

Once the points of interest are projected onto point clouds, users can further utilize the backward projection function to locate the corresponding points in an image.

