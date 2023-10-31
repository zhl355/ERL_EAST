# Environments Package
This package contains code for several environments used in search-based planning for robotics. 
The environment is an abstract type that can efficiently compute the set of reachable successor states from a given initial state, subject to system dynamics with discretized 
control input (Motion Primitives) and with support for collision checking via a provided cost map. 
 
The package contains several environment types, each implementing a different system's dynamics for a set of control inputs. The current environments supported are:

Supported Map Type
- 2D Grid
- 2D Costmap
