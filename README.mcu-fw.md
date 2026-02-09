# What I did #

add logs when IMX system manager firmware got PSCI command via SCMI from imx-atf, and ensures how logic machine working to M-core and A-cores

purpose: original from customer requirement needs M-cores FW runtime update and cold reset from running Linux system(A-cores).

see also: imx-atf passthrough PSCI reset command for SCMI trigger LMM reset specific core or all
see also: imx-linux driver/dts power reset node set reboot args passthrough PSCI command to ATF
see also: imx-mkimage generate the AHAB container by given TCM and M-cores FW binaries

## TODO ##
- how to modify the M-cores FW inside of AHAB container in runtime, opposite to offline uuu or flash tool did to OEI

- secure boot behavior on the AHAB container by BL3

