import skfuzzy as fuzz
from skfuzzy import control as ctrl
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
import sensor_msgs.point_cloud2 as pc2
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from numpy  import array
import io
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from PIL import Image, ImageDraw
import tf
from geometry_msgs.msg import Pose, PointStamped
from matplotlib.patches import Rectangle, Arrow;
import math
from nav_msgs.msg import *
from std_msgs.msg import *
import scipy.misc

# New Antecedent/Consequent objects hold universe variables and membership
# functions
distanciaY = ctrl.Antecedent(np.arange(-10, 10, 0.005), 'distanciaY')
distanciaX = ctrl.Antecedent(np.arange(0, 5, 0.005), 'distanciaX')
vLinear   = ctrl.Consequent(np.arange(0, 2.5, 0.005), 'vLinear')
vAngular   = ctrl.Consequent(np.arange(-1, 1, 0.005), 'vAngular')

# Auto-membership function population is possible with .automf(3, 5, or 7)
#quality.automf(3) #cria automaticamente as variaveis linguisticas
#service.automf(3)

# Custom membership functions can be built interactively with a familiar,
# Pythonic API
distanciaY['ME'] = fuzz.trapmf(distanciaY.universe, [-10, -10,-3.0,-1.5 ]) #trapezoidal
distanciaY['E']  = fuzz.trimf(distanciaY.universe, [-3.0, -1.5, 0])
distanciaY['R']  = fuzz.trimf(distanciaY.universe, [-1.5, 0, 1.5])
distanciaY['D']  = fuzz.trimf(distanciaY.universe, [0, 1.5, 3.0])
distanciaY['MD'] = fuzz.trapmf(distanciaY.universe, [1.5, 3.0, 10, 10]) #trap	


distanciaX['MP'] = fuzz.trapmf(distanciaX.universe, [-1,-1, 1, 2])
distanciaX['P']  = fuzz.trimf(distanciaX.universe, [1, 2, 3])
distanciaX['L']  = fuzz.trimf(distanciaX.universe, [2, 3, 4])
distanciaX['ML'] = fuzz.trapmf(distanciaX.universe, [3 ,4 ,5 ,5 ]) #trapezoidal


#distanciaY.view()

# Custom membership functions can be built interactively with a familiar,
# Pythonic API
vAngular['VME'] = fuzz.trapmf(vAngular.universe, [-10, -10,-0.6,-0.3 ]) #trapezoidal
vAngular['VE']  = fuzz.trimf(vAngular.universe, [-0.6, -0.3, 0])
vAngular['R']   = fuzz.trimf(vAngular.universe, [-0.3, 0, 0.3])
vAngular['VD']  = fuzz.trimf(vAngular.universe, [0, 0.3, 0.6])
vAngular['VMD'] = fuzz.trapmf(vAngular.universe, [0.3, 0.6, 10, 10]) #trapezoidal

vLinear['VMP']  = fuzz.trapmf(vLinear.universe, [-1.0,-1.0, 0.0, 0.0])
vLinear['VP']   = fuzz.trimf(vLinear.universe, [0.1, 0.2, 0.3])
vLinear['VL']  = fuzz.trimf(vLinear.universe, [0.2, 0.3, 0.4])
vLinear['VML'] = fuzz.trimf(vLinear.universe, [0.3, 0.4, 0.5])

#vAngular.view()
#tip.view()

rule1 = ctrl.Rule(distanciaY['ME'], vAngular['VD'])
rule2 = ctrl.Rule(distanciaY['E'], vAngular['VMD'])
rule3 = ctrl.Rule(distanciaY['R'], vAngular['R'])
rule4 = ctrl.Rule(distanciaY['D'], vAngular['VME'])
rule5 = ctrl.Rule(distanciaY['MD'], vAngular['VE'])


rule6 = ctrl.Rule(distanciaX['ML'], vLinear['VML'])
rule7 = ctrl.Rule(distanciaX['L'], vLinear['VL'])
rule8 = ctrl.Rule(distanciaX['P'], vLinear['VP'])
rule9 = ctrl.Rule(distanciaX['MP'], vLinear['VMP'])


#rule1.view()

tipping_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5,rule6,rule7,rule8,rule9])

tipping = ctrl.ControlSystemSimulation(tipping_ctrl)

# Pass inputs to the ControlSystem using Antecedent labels with Pythonic API
# Note: if you like passing many inputs all at once, use .inputs(dict_of_data)

#Codigo acima dessa linha eh o fuzzy

#Codigo abaixo eh o controle que criamos
laser_projector = LaserProjection();

def getLidar():
   msg = rospy.wait_for_message("/scan", LaserScan)
   cloud = laser_projector.projectLaser(msg)
   gen = pc2.read_points(cloud, skip_nans=True, field_names=("x","y"))
   xyz_generator = [];
   for p in gen:
      xyz_generator.append([p[0],p[1]])
   return array(xyz_generator);

def plot3Dpoints(x,y):
	fig = pyplot.figure()
	ax = Axes3D(fig)
	ax.scatter(x,y)
	pyplot.show()


rospy.init_node('listener', anonymous=True)
msg = Twist()
while True:
	xyz = getLidar()
	#plot3Dpoints(xyz[:,0],xyz[:,1]);
	#while True:
	menor = 10
	y = 0
	for x in xyz:
	   if x[0] > 0.0:
	      if x[0] < menor:
		 menor = x[0]
		 y = x[1]
	print menor,y 
	#if xyz[:,0] = x
	 #  y = min(xyz[:,1])
	 # print "menor y" + str(y)
	

	tipping.input['distanciaY'] = y 
	tipping.input['distanciaX'] = menor
	tipping.compute()

	saidaY = tipping.output['vAngular']
	saidaX = tipping.output['vLinear']
	velocity_publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
	
	msg.linear.x = saidaX	
	msg.angular.z = saidaY
	velocity_publisher.publish(msg)
	

