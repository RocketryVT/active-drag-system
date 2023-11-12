#!/bin/sh

# Change based upon device specifics
IFACE_BB=enp0s20f0u5
IFACE_SELF=wlan0
EXE=ads

echo "Compiling Executeables"
vagrant up
vagrant ssh << EOF
    cmake /vagrant;
    cmake --build .;
    mkdir -p build
    cp -r out/* /vagrant/build/
EOF

echo "Forwarding Traffic to Beaglebone"
# Forwards all internet traffic requested by beaglebone to proper interface
sudo sh -c "ip link set $IFACE_BB"
sudo sh -c "dhclient $IFACE_BB"

sudo sh -c "iptables --table nat --append POSTROUTING --out-interface $IFACE_SELF -j MASQUERADE"
sudo sh -c "iptables --append FORWARD --in-interface $IFACE_BB -j ACCEPT"
sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"

echo "Copying Executeables to Beaglebone"
scp -r build/* debian@beaglebone.local:~/;
rm -r build

echo "Running Main Program"
ssh debian@beaglebone.local /bin/bash << EOF
    ./$EXE;
EOF
