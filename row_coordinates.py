#!/usr/bin/env python3

import rospy
import json
import os
import sys
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class RowCoordinates:
   def __init__(self):
       # Initsialiseerime ROS node'i
       rospy.init_node('row_coordinates', anonymous=True)
       
       # Publisherid markerite visualiseerimiseks
       self.marker_pub = rospy.Publisher('row_markers', MarkerArray, queue_size=10)
       
       # Põllu parameetrid (meetrites)
       self.row_spacing = 1.4  # ridade vahe
       self.section_width = 20.0  # sektsiooni laius
       self.row_length = 118.0  # võtame minimaalse pikkuse
       
       # Loome koordinaadid vastavalt põllu struktuurile
       self.test_coordinates = []
       
       # Proovime laadida olemasolevaid koordinaate
       self.load_coordinates()

       # Kui koordinaadid on tühjad, genereerime need
       if not self.test_coordinates:
           self.generate_default_coordinates()

   def generate_default_coordinates(self):
       """Genereerib vaikimisi koordinaadid"""
       # Vasak servarida (3 rida)
       for i in range(3):
           start_y = i * self.row_spacing
           self.test_coordinates.append([(0, start_y, 0), (self.row_length, start_y, 0)])
       
       # Keskmine rida (6 rida)
       center_start = 7.0
       for i in range(6):
           start_y = center_start + (i * self.row_spacing)
           self.test_coordinates.append([(0, start_y, 0), (self.row_length, start_y, 0)])
       
       # Parem servarida (3 rida)
       right_start = 17.2
       for i in range(3):
           start_y = right_start + (i * self.row_spacing)
           self.test_coordinates.append([(0, start_y, 0), (self.row_length, start_y, 0)])

   def save_coordinates(self, filename='row_coordinates.json'):
       """Salvestab koordinaadid JSON faili"""
       coordinates_data = {
           'row_spacing': self.row_spacing,
           'section_width': self.section_width,
           'row_length': self.row_length,
           'coordinates': self.test_coordinates
       }
       
       config_path = os.path.join(os.path.dirname(__file__), '../config')
       if not os.path.exists(config_path):
           os.makedirs(config_path)
           
       filepath = os.path.join(config_path, filename)
       with open(filepath, 'w') as f:
           json.dump(coordinates_data, f, indent=4)
       rospy.loginfo(f"Koordinaadid salvestatud: {filepath}")

   def load_coordinates(self, filename='row_coordinates.json'):
       """Laeb koordinaadid JSON failist"""
       config_path = os.path.join(os.path.dirname(__file__), '../config')
       filepath = os.path.join(config_path, filename)
       
       try:
           with open(filepath, 'r') as f:
               data = json.load(f)
               self.row_spacing = data['row_spacing']
               self.section_width = data['section_width']
               self.row_length = data['row_length']
               self.test_coordinates = data['coordinates']
               rospy.loginfo(f"Koordinaadid laetud: {filepath}")
       except FileNotFoundError:
           rospy.logwarn(f"Koordinaatide faili ei leitud: {filepath}")
           return False
       return True

   def print_coordinates(self):
       """Kuvab kõik koordinaadid"""
       rospy.loginfo("\nPõllu parameetrid:")
       rospy.loginfo(f"Ridade vahe: {self.row_spacing}m")
       rospy.loginfo(f"Sektsiooni laius: {self.section_width}m")
       rospy.loginfo(f"Rea pikkus: {self.row_length}m")
       rospy.loginfo("\nRead:")
       for i, row in enumerate(self.test_coordinates):
           start = row[0]
           end = row[1]
           rospy.loginfo(f"Rida {i+1}: Algus({start[0]}, {start[1]}, {start[2]}) -> Lõpp({end[0]}, {end[1]}, {end[2]})")

   def update_field_parameters(self, spacing=None, width=None, length=None):
       """Uuendab põllu parameetreid"""
       if spacing is not None:
           self.row_spacing = spacing
       if width is not None:
           self.section_width = width
       if length is not None:
           self.row_length = length
       
       # Genereerime uued koordinaadid
       self.test_coordinates = []
       self.generate_default_coordinates()
       self.save_coordinates()
       rospy.loginfo("Põllu parameetrid uuendatud ja salvestatud")

   def create_row_marker(self, start_point, end_point, id_num):
       marker = Marker()
       marker.header.frame_id = "map"
       marker.header.stamp = rospy.Time.now()
       marker.ns = "rows"
       marker.id = id_num
       marker.type = Marker.LINE_STRIP
       marker.action = Marker.ADD
       
       marker.scale.x = 0.1
       marker.color.r = 0.0
       marker.color.g = 1.0
       marker.color.b = 0.0
       marker.color.a = 1.0
       
       start = Point(start_point[0], start_point[1], start_point[2])
       end = Point(end_point[0], end_point[1], end_point[2])
       marker.points = [start, end]
       
       return marker

   def publish_markers(self):
       marker_array = MarkerArray()
       
       for i, row in enumerate(self.test_coordinates):
           marker = self.create_row_marker(row[0], row[1], i)
           marker_array.markers.append(marker)
       
       self.marker_pub.publish(marker_array)

   def process_command(self):
       """Töötleb kasutaja käske"""
       rospy.loginfo("\nKäsud:")
       rospy.loginfo("show - Näita koordinaate")
       rospy.loginfo("update - Uuenda põllu parameetreid")
       rospy.loginfo("quit - Välju programmist")
       
       while not rospy.is_shutdown():
           command = input("\nSisesta käsk: ").strip().lower()
           
           if command == 'show':
               self.print_coordinates()
           
           elif command == 'update':
               try:
                   spacing = float(input("Sisesta uus ridade vahe (või vajuta Enter): ") or self.row_spacing)
                   width = float(input("Sisesta uus sektsiooni laius (või vajuta Enter): ") or self.section_width)
                   length = float(input("Sisesta uus rea pikkus (või vajuta Enter): ") or self.row_length)
                   self.update_field_parameters(spacing, width, length)
               except ValueError:
                   rospy.logwarn("Vigane sisend! Kasuta numbreid.")
           
           elif command == 'quit':
               return
           
           else:
               rospy.logwarn("Tundmatu käsk!")

   def run(self):
       # Käivitame eraldi lõimes markerite avaldamise
       import threading
       pub_thread = threading.Thread(target=self._publish_loop)
       pub_thread.daemon = True
       pub_thread.start()
       
       # Käivitame käskude töötlemise
       self.process_command()

   def _publish_loop(self):
       """Markerite avaldamise põhitsükkel"""
       rate = rospy.Rate(1)
       while not rospy.is_shutdown():
           self.publish_markers()
           rate.sleep()

if __name__ == '__main__':
   try:
       coordinator = RowCoordinates()
       coordinator.run()
   except rospy.ROSInterruptException:
       pass