# 
# Usage: To re-create this platform project launch xsct with below options.
# xsct D:\Work\projects\zybo_z7_projects\DD_RF\vitis\RFsender_platform\platform.tcl
# 
# OR launch xsct and run below command.
# source D:\Work\projects\zybo_z7_projects\DD_RF\vitis\RFsender_platform\platform.tcl
# 
# To create the platform in a different location, modify the -out option of "platform create" command.
# -out option specifies the output directory of the platform project.

platform create -name {RFsender_platform}\
-hw {D:\Work\projects\zybo_z7_projects\DD_RF\vivado\project_1\topSystem.xsa}\
-proc {ps7_cortexa9_0} -os {standalone} -out {D:/Work/projects/zybo_z7_projects/DD_RF/vitis}

platform write
platform generate -domains 
domain active {zynq_fsbl}
bsp reload
bsp reload
bsp reload
bsp reload
domain active {standalone_domain}
bsp reload
bsp setlib -name xilffs -ver 5.3
bsp write
bsp reload
catch {bsp regenerate}
platform generate
