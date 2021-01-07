# A Fork of Fork of openpilot

original fork https://github.com/ShaneSmiskol/openpilot/, only tested in car corolla 2020

## Additional Features

### output all op internal data structure over WIFI for my HMI prototype
[![](https://img.youtube.com/vi/rxTK5McUPA4/0.jpg)](https://www.youtube.com/watch?v=rxTK5McUPA4)

### automaticlly adjust in lane offset
It will check if the ego car is in the left most lane or right most lane or middle lane,
if ego is in left most lane, it will adjust the offset 0.2m to the left and vice versa to keep distance away from the big truck.
Sometimes the car is driving too much "dead center", human driver wont do that

![pic1](https://raw.githubusercontent.com/299299/openpilot/stock_additions_new/doc/lateral_offset.png)
