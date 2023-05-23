#!/bin/bash

sudo apt update
sudo apt install -y wireguard

wg genkey | sudo tee /etc/wireguard/privatekey | wg pubkey | sudo tee /etc/wireguard/publickey

# allow ipv4 forwarding
sudo sed -i "s/#net.ipv4.ip_forward/net.ipv4.ip_forward/" /etc/sysctl.conf

cat > /tmp/wg0.conf << EOF
[Interface]
Address = 10.0.0.1/24
SaveConfig = true
ListenPort = 51820
PrivateKey = $(sudo cat /etc/wireguard/privatekey)
PostUp = iptables -A FORWARD -i %i -j ACCEPT; iptables -t nat -A POSTROUTING -o wlan0 -j MASQUERADE
PostDown = iptables -D FORWARD -i %i -j ACCEPT; iptables -t nat -D POSTROUTING -o wlan0 -j MASQUERADE
EOF

sudo cp /tmp/wg0.conf /etc/wireguard/

sudo chmod 600 /etc/wireguard/{privatekey,wg0.conf}

cat > ~/add-peer.sh << EOF
#!/bin/bash
sudo wg set wg0 peer PEERPUBKEY allowed-ips 10.0.0.0/24
EOF

chmod +x ~/add-peer.sh

sudo systemctl enable wg-quick@wg0.service
sudo systemctl start wg-quick@wg0.service
