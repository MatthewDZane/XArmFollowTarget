# Using XArm Follow Target as a Third Party Extension
1. Clone the repo into the directory where you would like to store Isaac Sim Extensions
2. Open Isaac Sim and go to Windows->Extensions
3. Click the Settings Icon (Gear) and add the path to the parent directory of the repo (not XArm or XArmFollowTarget). Now the XArm Follow Target Extention should show up under the Third Party Extensions.
4. Enable the XArm Follow Target Extension and check the Autoload setting. The XArm Follow Target Extension will now appear on the top menu bar of the Isaac Sim Application.
5. Click the XArm Follow Target to use the Extension

Port Forwar local ports to the Container for the realsense camera client. 
- once you have a XGL container running you will need to use the kubernetes CLI to get the specific pod name. This can be done with 

```
 kubecle get pods -n <your namespace>
```
once you have your pod name we can now prot forward the local ports to the container for communication, 

Run 
```
 kubectl port-forward <running XGL pod> 12345:12345 12346:12346 -n <your namespace>

```
We use these ports by default 
