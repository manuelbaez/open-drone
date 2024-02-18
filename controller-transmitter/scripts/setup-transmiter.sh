export INTERFACE=$1
export CHANNEL=$2

systemctl stop NetworkManager
ip link set $INTERFACE down
iw $INTERFACE set monitor none 
iw dev $INTERFACE set txpower fixed 22mBm
ip link set $INTERFACE up
iw dev $INTERFACE set channel $CHANNEL
systemctl start NetworkManager

# iwconfig $INTERFACE txpower 22
