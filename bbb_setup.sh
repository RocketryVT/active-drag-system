#!/bin/sh

# Change based upon device specifics
IFACE_BB=enp0s20f0u5
IFACE_SELF=wlan0
EXE=ads

echo "Forwarding Traffic to Beaglebone"
# Forwards all internet traffic requested by beaglebone to proper interface
sudo sh -c "ip link set $IFACE_BB"
sudo sh -c "dhclient $IFACE_BB"

sudo sh -c "iptables --table nat --append POSTROUTING --out-interface $IFACE_SELF -j MASQUERADE"
sudo sh -c "iptables --append FORWARD --in-interface $IFACE_BB -j ACCEPT"
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"

echo "Mounting Beaglebone Filesystem"
# Mounts beaglebone filesystem for cross-compilation linking
sudo mkdir -p /mnt/bbb-sysroot
sudo chmod o+rwx /mnt/bbb-sysroot
sshfs debian@beaglebone.local:/ /mnt/bbb-sysroot/ -o transform_symlinks

echo "Compiling Executeable"
vagrant up
vagrant ssh << EOF
    cmake /vagrant;
    cmake --build .;
    cp -r src/$EXE /vagrant/
EOF

echo "Copying Executeable to Beaglebone"
scp -r $EXE debian@beaglebone.local:~/;
rm -r $EXE

echo "Running Program"
ssh debian@beaglebone.local /bin/bash << EOF
    ./$EXE;
    rm -r $EXE
EOF

echo "Unmounting Beaglebone Filesystem"
sync
fusermount -u /mnt/bbb-sysroot
sudo rm -r /mnt/bbb-sysroot
rm -r bbb-sysroot
