Even though the main and core functionality that was initially proposed for the scope of this project was almost totally completed, some extra work and features that weren't specified in our documentation, neither in the main or extra features, were great ideas thrown not only by our mentors but by some of the group members as well.  

#### Travel to user's current location
 * One of these ideas, and probably one of the first ones to arise, was the feature of having the chair travel to the user's current location. This consisted on replicating the idea of an uber or a taxi, but it was lately discarded, not because of it's complexity but because our main goal was to first have the chair move to an arbitrary location chosen by the user. Another good reason to discard this feature was the fact that it didn't have an easy fix on how to locate the user on the current existing map.

#### Self-Docking
 * Other feature that we didn't really worry about, and simply specified the system with the non-existance of that specific feature, was the self-docking feature for the wheelchair. This would serve the purpose of, for example, have the chair park itself in a nearby docking station where it would charge the batteries. This was later re-iterated to work as just a "stop and warn the user when out of battery" but that wasn't implemented as well, for the same reason as the feature above.





#### Real-time location of the wheelchair

 * When speaking in terms of knowing where the wheelchair is, we currently have only one way to do this: Using the rviz system to display the chair's footprint. On the mobile application, even though we can set a goal in the current loaded map, we have no information about the chair's location. This functionality would also be pretty good to have in the dashboard, letting the system administrator or manager see where each chair is on their own map. The tools to develop this feature are all in place, only needing some work on sending some basic information through the rosbridge channel to the application and through the constant connection tunnel between the chair and the server.

#### Dynamic object avoidance

 * The object avoidance while autonomously navigating was a feature we had planned since the very beginning of the project. It was added in our extra features, which can be seen in our [MVP](../team/deliv.md), but we never managed to complete it. The basic idea to solve this single problem was to use the depth channel of the kinect camera placed in front of the chair. The main issue was that synchronizing the laser's point cloud in the back with the camera's point cloud in the front is not trivial, and since we had other features prioritized that we still had to finish, we decided to skip this one. Another way to solve this issue would be to add another laser range finder, a cheaper and lower distance one, in the front of the chair.

#### Ramp issues on mapping and navigation

 * One of the challenges on some of the environments we tested the chair on, including IRIS Lab and the DETI Hallways, was the need to handle the ramps present in these places. This was a problem because the range finder, at the height it was placed on, captured the ramp as a wall in the generated map. The fix for this was to generate three distinct maps. One for the first hallway, one for the ramp itself and one for the other end of the hallway (past the ramp). This way, the current map of the wheelchair could be changed to match the current location of it, and the end of the map would work like a "portal" to the other.


