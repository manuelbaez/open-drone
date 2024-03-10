export INTERFACE=$1

systemctl stop NetworkManager
ip link set $INTERFACE down
sudo iwconfig wlp6s0 mode managed
ip link set $INTERFACE up
systemctl start NetworkManager