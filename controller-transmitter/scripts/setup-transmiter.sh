export INTERFACE=$1
export CHANNEL=$1

systemctl stop NetworkManager
ip link set $INTERFACE down
iw $INTERFACE set monitor none 
iwconfig $INTERFACE channel $CHANNEL
systemctl start NetworkManager
