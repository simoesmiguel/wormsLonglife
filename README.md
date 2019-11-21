### wormsLonglife
This repository holds all the required code to run wormsLonglife game.

### Requirements:

To run the game you need:

* Python >= 3.5
* pygame >= 1.9.2b6

Python 3.5+ should be installed in any relatively recent Linux distribution.
To install pygame for python3 you may try using pip3:
    sudo pip3 install pygame


### Commands to run the game:
```
python3 start.py                  //play the game with Agent1 on a random world.
python3 start.py -m mapa4.bmp     // use a specified world map.
python3 start.py -s StudentAgent  // use another agent.
python3 start.py -d 1             // show a log of information messages (and above).
python3 start.py -d 0 -v          // run fast without video, show debug log
```

#### Usage example:
python3 start.py -s StudentAgent -m mapa2.bmp

