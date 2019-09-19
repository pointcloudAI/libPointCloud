## SET USB Driver on Ubuntu/Linux
```
sudo cp ./config/ubuntu/72-CalculusCDK.rules /etc/udev/rules.d/
sudo cp ./config/ubuntu/60-SonyCDK.rules /etc/udev/rules.d/

sudo chmod a+x /etc/udev/rules.d/72-CalculusCDK.rules
sudo chmod a+x /etc/udev/rules.d/60-SonyCDK.rules

sudo udevadm control --reload
```
