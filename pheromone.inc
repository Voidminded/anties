
# Desc: A pheromone model for ant algorithm
# Author: Sepehr 
# Date: 19 September 2016


define pheromone position 
(
  color "gray50"

  obstacle_return 0           	# Can hit things.
  laser_return 0                 # reflects laser beams
  ranger_return 1             	# reflects sonar beams
  blobfinder_return 0         	# Seen by blobfinders  
  fiducial_return 1           	# Seen as "1" fiducial finders

  localization "gps"         	  

  size [0.01 0.03 0.1]
  #origin [-0.25 -0.25 0 0 ]

  block 
  (
    points 4
    point[0] [ 0.1 0.25 ] 
    point[1] [ 0.9 0.25 ] 
    point[2] [ 0.9 0.75 ] 
    point[3] [ 0.1 0.75 ] 
    z [ 0.2 0.6 ]
  )

)
