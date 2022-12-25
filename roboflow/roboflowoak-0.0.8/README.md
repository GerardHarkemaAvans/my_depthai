# Setup Instructions

## DepthAI

### Linux

Install DepthAI Dependencies 

```
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash
```  

Set USB Permissions  

```
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Windows

Install Chocolatey  

```
Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))
```  

Install DepthAI Dependencies

```
choco install cmake git python pycharm-community -y
```

### MacOS

Install DepthAI Dependencies

```
bash -c "$(curl -fL https://docs.luxonis.com/install_dependencies.sh)"
```

See M1 specific instructions in `M1Install.md`

## Install Python Dependencies

```
pip3 install -r requirements.txt
```

Importantly, depthai and opencv

## Deploy 

```
rm -rf build
rm -rf dist

python setup.py sdist bdist_wheel
twine check dist/*
twine upload dist/*
```
