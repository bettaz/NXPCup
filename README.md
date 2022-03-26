## NXPCup
Industrial proj

## Resources
[Gantt Chart](https://docs.google.com/spreadsheets/d/1_ab_1B6ywoD8W_HedOslyi2c5SXox_DHgxD6Ps3EEKg/edit?usp=sharing)  ||   [NXP Issue Log](https://docs.google.com/document/d/1cK3jTuf8C7oN0TY4DB1Q-S80q4x-OqfHRKSxHm6Nyq0/edit)

## Control Algorithm
From the vectors we get from Pixy camera, we do the correction like this:

![](https://postimg.cc/Vr37z8CM)
![](https://encrypted-tbn0.gstatic.com/images?q=tbn:ANd9GcT3jPLBkHYZMJDr-UdfxRsf4-_3PNqLejt8zg&usqp=CAU)

This can help us calulate the average inclination of all the vectors.
Then we can define the maximum cornering speed and angle if we want to go faster:

![](https://a.sidepodcast.com/content/2014/07/car-turning-top.jpg)


## Raspberry configuration
connection command: ssh pi@raspberrypi.local
ssh pwd: raspberrypy
raspberry wifi preset:
- SSID: Betta-fy
- PWD: vannozzo20

