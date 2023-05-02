1. Clone the repo

2. place in this path to run:
```
/home/user/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples
```

3. Rename the folder **xarm** to **XArm**

4. In the examples folder you will need to add 2 lines to the **__init__.py** file there (not in the XArm folder) 


NOTE: Import here your extension examples to be propagated to ISAAC SIM Extensions startup
```
from omni.isaac.examples.user_examples.XArm.xarm_sample import XArmSample
from omni.isaac.examples.user_examples.XArm.xarm_extension import XArmExtension
```

5.  After this runs you should see the XArm extension in the Example drop down in the Isaac-sim gui

6. To recieve the position output run the **client.py** script found in the folder, the **server.py** runs by default once the example is loaded. 
