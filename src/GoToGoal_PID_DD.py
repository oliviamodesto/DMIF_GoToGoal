# PID control loop sviluppato in linguaggio Python per un drone su ruota del tipo
# differential drive e controllo del movimento lungo percorso con waypoint predefiniti
# Olivia Modesto
# Udine, 23 Giugno 2024

import matplotlib.pyplot as plt
import numpy as np
import math
import PIL
from PIL import Image

from src.constants import K_v  # proportional coefficient for linear velocity
from src.constants import K_phi  # proportional coefficient for angular velocity
from src.constants import K_i  # integral coefficient for linear velocity
from src.constants import K_w  # integral coefficient for angular velocity
from src.constants import K_dv  # derivative coefficient for linear velocity
from src.constants import K_dw # derivative coefficient for angular velocity
from src.constants import t  # time step
from src.constants import eps  # margin error
from src.constants import l  # wheel-center distance


def main(goal):
  # VARIABILES INITIALIZTION #################################################################

  xr = 0
  yr = 0  # robot's initial position
  xr_old = xr
  yr_old = yr  # robot's previous coordinates
  xr_new = 0
  yr_new = 0  # robot's next coordinates

  phi_old = np.pi / 2  # initial heading
  phi_goal = np.pi  # goal orientation
  phi_old_old = phi_old

  dist_old = 0  # distance between robot position and goal position
  dist_old_old = dist_old

  integrale_velocita = 0  # integral term linear velocity
  integrale_angolare = 0  # integral term angular velocity

  ############################################################################################

  # CONSTANTS ################################################################################

  n_goal = len(goal)  # number of goals

  ############################################################################################

  # PLOT #####################################################################################
  dpi = 72

  path = "./diffdrive.png"
  sprite = Image.open(path)  # robot sprite for plot

  sprite_size = sprite.size[1], sprite.size[0]

  xdata = []
  ydata = []

  fig = plt.figure()
  axes = plt.gca()

  line1, = axes.plot(0, 0, 'b.')
  line2, = axes.plot(0, 0, "bo", mfc="None", mec="None", markersize=sprite_size[0] * (dpi / 96))

  for i in range(0, n_goal):
    x_goal, y_goal = goal[i]
    line0, = axes.plot(x_goal, y_goal, 'r*')

  line1.set_xdata([xr]), line1.set_ydata([yr])
  line2.set_xdata([xr]), line2.set_ydata([yr])

  plt.draw()
  axes.set_xlim(0, 6), axes.set_ylim(0, 6)
  plt.pause(0.01)

  ############################################################################################

  # CICLO ####################################################################################

  for i in range(0, n_goal):

    xg, yg = goal[i]

    while True:

      if (xr_new - xg) ** 2 < eps and (yr_new - yg) ** 2 < eps:
        print("GOAL (", xg, ", ", yg, ") ", "RAGGIUNTO")
        integrale_velocita = 0
        integrale_angolare = 0
        break

      dist_now = np.sqrt((xg - xr_new) ** 2 + (yg - yr_new) ** 2)

      phi_desired = np.arctan2((yg - yr_old),
                               (xg - xr_old))  # arctan2 tiene conto dei segni risultando nel quadrante giusto

      alpha = phi_desired - phi_old
      alpha = np.mod(alpha, 2 * np.pi)
      alpha = alpha - 2 * np.pi

      error_phi = alpha

      # avoid turning the wrong way
      if error_phi > np.pi:
        error_phi = error_phi - 2 * np.pi
      elif error_phi < -np.pi:
        error_phi = error_phi + 2 * np.pi

      # PID begin

      integrale_velocita = dist_now * t * K_i + integrale_velocita

      integrale_angolare = error_phi * t * K_w + integrale_angolare

      derivata_velocita = (dist_old - dist_old_old) * K_dv / t

      derivata_angolare = (phi_old - phi_old_old) * K_dw / t

      v = K_v * dist_now + integrale_velocita + derivata_velocita

      w = K_phi * error_phi + integrale_angolare + derivata_angolare

      # PID end

      # Differential Drive begin

      v_dx = v + ((w * l) / 2)

      v_sx = v - ((w * l) / 2)

      xr_new = xr_old + ((v_sx + v_dx) / 2 * math.cos(phi_old) * t)

      yr_new = yr_old + ((v_sx + v_dx) / 2 * math.sin(phi_old) * t)

      phi_new = phi_old + ((v_dx - v_sx) * t / l)

      # differential drive end

      # update plot

      xdata.append(xr_new), ydata.append(yr_new)

      line1.set_xdata(xdata), line1.set_ydata(ydata)
      line2, = axes.plot(xdata[-1], ydata[-1], "bo", mfc="None", mec="None", markersize=sprite_size[0] * (dpi / 96))
      line2._transform_path()
      path, affine = line2._transformed_path.get_transformed_points_and_affine()
      path = affine.transform_path(path)

      for pixelPoint in path.vertices:
        rotated = sprite.rotate(np.degrees(phi_new), PIL.Image.NEAREST, expand=1)
        fig = plt.figimage(rotated, pixelPoint[0] - sprite_size[0] / 2, pixelPoint[1] - sprite_size[1] / 2,
                           origin="upper")

      plt.draw()
      plt.pause(t)
      fig.remove()

      # update variabiles

      phi_old_old = phi_old
      phi_old = phi_new

      xr_old = xr_new
      yr_old = yr_new

      dist_old_old = dist_old
      dist_old = dist_now

      if plt.waitforbuttonpress(0.01): break

  # add this if you don't want the window to disappear at the end
  plt.show()