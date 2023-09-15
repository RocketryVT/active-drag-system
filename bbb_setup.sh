#!/bin/sh

# Change based upon device specifics
IFACE_BB=enp0s20f0u5
IFACE_SELF=wlan0

# Forwards all internet traffic requested by beaglebone to proper interface
sudo sh -c "ip link set $IFACE_BB"
sudo sh -c "dhclient $IFACE_BB"

sudo sh -c "iptables --table nat --append POSTROUTING --out-interface $IFACE_SELF -j MASQUERADE"
sudo sh -c "iptables --append FORWARD --in-interface $IFACE_BB -j ACCEPT"
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"

# Mounts beaglebone filesystem for cross-compilation linking
sudo mkdir -p /mnt/bbb-sysroot
sudo chmod o+rwx /mnt/bbb-sysroot
sshfs debian@beaglebone.local:/ /mnt/bbb-sysroot/ -o transform_symlinks
