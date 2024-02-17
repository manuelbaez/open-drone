export INTERFACE=$1
export CHANNEL=$2

systemctl stop NetworkManager
ip link set $INTERFACE down
iw $INTERFACE set monitor none 
ip link set $INTERFACE up
iwconfig $INTERFACE channel $CHANNEL
systemctl start NetworkManager
