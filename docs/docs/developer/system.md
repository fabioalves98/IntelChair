# Developer

Looking at the the [architecture](../architecture/system.md) page we can clearly split our main system in three distinct parts. In this section we are creating some directives on how to deploy each of these individual system parts, and even though it is not an in depth, detailed guide, it helps taking the right steps in to re-creating this project from scratch.

---

To guarantee the best possible outcome of the system deployment, we recommend deploying the different parts in the following order:

1. Server
2. Wheelchair
3. Mobile Application

Since the [server](server.md) is the core part of the system, saving all user and chair information and connecting both of the two remaining nodes together, we must deploy the server first. It is responsible of exchanging the IP information needed for the mobile [application](mobile.md) to connect directly to the [wheelchair](wheelchair.md).
Finally we can deploy and start the wheelchair itself, running the wanted nodes independently (basic functionality and navigation) and use the mobile application to connect to it.

If you wish to proceed with the deployment of the project, continue on to [server](server.md).
