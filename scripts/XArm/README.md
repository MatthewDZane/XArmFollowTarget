# Using XArm Follow Target as an Isaac Example
1. cd into the Isaac Sim User examples directory.
```
/home/user/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples/
```

2. Clone the repo as XArmFollowTarget.
```
git clone https://gitlab.nrp-nautilus.io/isaac/xarm.git XArmFollowTarget
```

3. Add the following lines to the .../user_examples/__init__.py file (not in the repo directory):
```
from omni.isaac.examples.user_examples.XArmFollowTarget.scripts.XArm.xarm_sample import XArmSample
from omni.isaac.examples.user_examples.XArmFollowTarget.scripts.XArm.xarm_extension import XArmExtension
```

4.  After this runs you should see the XArm extension in the Example drop down in the Isaac-sim gui

5. To recieve the position output run the **client.py** script found in the folder, the **server.py** runs by default once the example is loaded. 
