import subprocess
sim_root = "~/ardupilot/Tools/autotest/sim_vechicle.py -v Arduplane -f plane --map"
fg_root = "~/ardupilot/Tools/autotest/fg_plane_view.sh"
subprocess.call(sim_root,shell = True)
subprocess.call(fg_root, shell = True)
