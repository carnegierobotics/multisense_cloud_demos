# Edit to suit your network configuration
# Default MultiSense IP address is 10.66.171.20 
# Gbit and a big MTU (7200) are important for good performance
sudo service network-manager stop
sudo ifconfig eth0 10.66.171.20 netmask 255.255.255.0 up
sudo ifconfig eth0 mtu 7200
sudo sh -c 'echo 16777215 > /proc/sys/net/core/rmem_max'
sudo sh -c 'echo 16777215 > /proc/sys/net/core/wmem_max'
