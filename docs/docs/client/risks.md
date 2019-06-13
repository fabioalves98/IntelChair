###Hitting something that was not previously mapped by the wheelchair
* This is called a dynamic object. It could be anything that was not present in the environment when the map was generated. Because it was not present, the wheelchair will have no information of its presence and might hit it. We plan to use the camera positioned in the front of the wheelchair to identify such objects and possibly avoid them.

###Failure of connection between the user and the wheelchair
* The connection between the elements of this project (server, wheelchair and mobile application) will be established through WiFi. If any access point or router in the building fails, the connection between the elements will be lost. If this happens, the user will be notified on the mobile application.

###Making sure only one user controls the wheelchair at a time
* Because any user on the network, if knowing the IP address of the wheelchair's computer, can attempt to connect, access the wheelchair's information and control it, we will have to create certain mechanisms that prevent this from happening and restrict the wheelchair's control to a single user.

###The elements of the enviroment may change from what was previously mapped
* If the environment changes drastically, the wheelchair might not be able to find itself on the map that it possesses. In this scenario, the user will be notified of such behavior and the wheelchair will have to generate another map so it can acommodate the new changes.

###Technical related issues
* More information on these can be found in the [specification](../specification/restrictions.md) page under restrictions.