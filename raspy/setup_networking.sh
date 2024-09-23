nmcli connection add \
    con-name Private_Static \
    ifname eth0 \
    type ethernet \
    autoconnect true \
    connection.autoconnect-priority 0 \
    connection.autoconnect-retries -1 \
    ipv6.method disabled \
    ipv4.method manual \
    ip4 192.168.1.1/24 \
    gw4 192.168.1.254 \
    ipv4.dns 192.168.1.254 \
    save yes

nmcli connection reload