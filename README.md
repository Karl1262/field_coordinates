# Põlluridade Koordinaatide Haldussüsteem

ROS-põhine süsteem mustika istanduse põlluridade koordinaatide määramiseks ja haldamiseks.

## Eesmärk
Süsteem võimaldab:
- Määrata põlluridade täpseid algus- ja lõppkoordinaate
- Visualiseerida põllustruktuuri RViz'is
- Salvestada ja laadida koordinaate JSON formaadis
- Muuta põllu parameetreid dünaamiliselt

## Paigaldamine
### Eeldused
- ROS Noetic
- Python 3
- Ubuntu 20.04

### Paigaldamine
```bash
cd ~/catkin_ws/src
git clone https://github.com/Karl1262/field_coordinates.git
cd ..
catkin_make
source devel/setup.bash
```
## Kasutamine
Süsteemi käivitamiseks:
```bash
roscore
rosrun field_coordinates row_coordinates.py
