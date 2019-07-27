# jetroomba
A repo for getting the Nvidia Jetson / Jetbot running on a Roomba!

# Getting Started

## Hardware
The easiest option is to use a USB->TTL converter.  (e.g. [Amazon](https://www.amazon.com/gp/product/B06XDPMY4Z/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1)) It should be possible to use the JetBot's UART lines, but I haven't tried it.  

We're making use of a custom mount that holds the JetBot and camera.

Todo:  Add STL files.

## Software
Clone this repo!  You'll then need to add the Python files to the installed location of the JetBot Python3 module. (todo: ```/usr/local/lib/python3.6/dist-packages/jetbot-0.3.0-py3.6.egg/jetbot/```)  

To use these motor/robot modules within the JetBot Jupyter Notebooks, you'll need to replace the robot object, e.g.:
```
from jetbot import roombarobot
robot = roombarobot.Robot()
```
