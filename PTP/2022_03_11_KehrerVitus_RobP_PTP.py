# Importierte Bibliotheken
from tkinter import W
from typing import Tuple
import matplotlib as mlt
import matplotlib.pyplot as plt
from brokenaxes import brokenaxes
import numpy as np
import pandas as pd
from matplotlib.colors import LightSource
from mpl_toolkits import mplot3d
import math
import scipy as sp

letter_phi_LC = "\u03C6"

# Leerzeilen
print("#######################\n\n\n\n")

#############################################################################################
################################ START HAUPTPROGRAMM ########################################
#############################################################################################
def main():
    # Auflösung für die Berechnung festlegen
    resolution = 200

    #### Festlegen der Roboterdaten
    # phi_deg = 0; phi_rad = np.radians(phi_deg);     L1 = 0.4; L2 = 0.6; L3 = 1.4; L4 = 0.4        # Initialstellung
    phi_deg = 10; phi_rad = np.radians(phi_deg);     L1 = 0.4; L2 = 0.6; L3 = 1.1; L4 = 0.4

    # maximale Geschwindigkeiten
    v_max = 0.8; omega_max = 0.2

    # maximale Beschleunigungen
    a_max = np.inf; alpha_max = np.inf

    # Punkte definieren, zwischen denen Eine PTP-Bewegung erfolgt
    vect_P1_rad = np.array([[np.radians(6)],    # Winkel in [°]
                                        [0.1]]) # länge in [m]
    vect_P2_rad = np.array([[np.radians(50)],    # Winkel in [°]
                                        [0.16]]) # länge in [m]

    # Positionsberechnung
    dict_guideaxis = calc_guideaxis(par_v_max=v_max, par_omega_max=omega_max, par_vect_P1=vect_P1_rad, par_vect_P2=vect_P2_rad)
    time_max = dict_guideaxis["time_max"]
    v_max = dict_guideaxis["velocity_linear"]
    omega_max = dict_guideaxis["omega_rotation"]

    # Berechnen der Bewegungen
    time = np.linspace(0, time_max, resolution)
    vect_q_rot = np.linspace(0, 1, resolution);                 vect_q_rot[:] = np.nan              # Initialisieren der Vektoren
    vect_q_p_rot = np.linspace(0, 1, resolution);               vect_q_p_rot[:] = np.nan            # Initialisieren der Vektoren
    vect_q_pp_rot = np.linspace(0, 1, resolution);              vect_q_pp_rot[:] = np.nan           # Initialisieren der Vektoren
    vect_x_yTCP = np.linspace(0, 1, resolution);                vect_x_yTCP[:] = np.nan             # Initialisieren der Vektoren
    vect_x_p_yTCP = np.linspace(0, 1, resolution);              vect_x_p_yTCP[:] = np.nan           # Initialisieren der Vektoren
    vect_x_pp_yTCP = np.linspace(0, 1, resolution);             vect_x_pp_yTCP[:] = np.nan          # Initialisieren der Vektoren
    vect_q_lin = np.linspace(0, 1, resolution);                 vect_q_lin[:] = np.nan              # Initialisieren der Vektoren
    vect_q_p_lin = np.linspace(0, 1, resolution);               vect_q_p_lin[:] = np.nan            # Initialisieren der Vektoren
    vect_q_pp_lin = np.linspace(0, 1, resolution);              vect_q_pp_lin[:] = np.nan           # Initialisieren der Vektoren
    vect_x_zTCP = np.linspace(0, 1, resolution);                vect_x_zTCP[:] = np.nan             # Initialisieren der Vektoren
    vect_x_p_zTCP = np.linspace(0, 1, resolution);              vect_x_p_zTCP[:] = np.nan           # Initialisieren der Vektoren
    vect_x_pp_zTCP = np.linspace(0,1,resolution);               vect_x_pp_zTCP[:] = np.nan          # Initialisieren der Vektoren
    for i in range(resolution):
        x = calc_motion(  par_L1=L1, par_L2=L2, par_L4=L4, par_accel_max=a_max, par_alpha_max=alpha_max, par_L3_start=vect_P1_rad[1][0], 
                                    par_omega_max=omega_max, par_phi_rad_start=vect_P1_rad[0][0], par_time=time[i], par_time_max=time_max, par_veloc_max=v_max)
        if x["success"] == True:
            vect_q_rot[i] = x["vect_q"][0][0]
            vect_q_p_rot[i] = x["vect_q_p"][0][0]
            vect_q_pp_rot[i] = x["vect_q_pp"][0][0]
            vect_x_yTCP[i] = x["vect_x"][0][0]
            vect_x_p_yTCP[i] = x["vect_x_p"][0][0]
            vect_x_pp_yTCP[i] = x["vect_x_pp"][0][0]
            vect_q_lin[i] = x["vect_q"][1][0]
            vect_q_p_lin[i] = x["vect_q_p"][1][0]
            vect_q_pp_lin[i] = x["vect_q_pp"][1][0]
            vect_x_zTCP[i] = x["vect_x"][1][0]
            vect_x_p_zTCP[i] = x["vect_x_p"][1][0]
            vect_x_pp_zTCP[i] = x["vect_x_pp"][1][0]            

    # Positionsberechnung
    calc_position(par_L1=L1, par_L2=L2, par_L3=L3, par_L4=L4, par_phi_rad=phi_rad)

    # Graphen plotten
    plotgraphs(par_time=time, par_vect_q_rot=vect_q_rot, par_vect_q_p_rot=vect_q_p_rot, par_vect_q_pp_rot=vect_q_pp_rot, 
                par_vect_x_yTCP=vect_x_yTCP, par_vect_x_p_yTCP=vect_x_p_yTCP, par_vect_x_pp_yTCP=vect_x_pp_yTCP, 
                par_vect_q_lin=vect_q_lin, par_vect_q_p_lin=vect_q_p_lin, par_vect_q_pp_lin=vect_q_pp_lin, 
                par_vect_x_zTCP=vect_x_zTCP, par_vect_x_p_zTCP=vect_x_p_zTCP, par_vect_x_pp_zTCP=vect_x_pp_zTCP)


# Funktion für die Positionsberechnung
def calc_guideaxis(par_v_max=None, par_omega_max=None, par_vect_P1=None, par_vect_P2=None):
    # Berechnen der Wegdifferenz
    delta_s = par_vect_P2[1][0] - par_vect_P1[1][0]
    print("Wegdifferenz")
    print(delta_s, "in [m]")

    # Berechnen der Winkeldifferenz
    delta_phi = par_vect_P2[0][0] - par_vect_P1[0][0]
    print("Winkeldifferenz")
    print(delta_phi, "in [rad]  | ", np.degrees(delta_phi), "in [°]")

    # Benötigte Zeit für Linearachse
    time_linear = (4 * delta_s) / (3 * par_v_max)
    print(f"Zeit welche die translatorische Achse benötigt: {time_linear}s")

    # Benötigte Zeit für Rotationsachse
    time_rotation = (4 * delta_phi) / (3 * par_omega_max)
    print(f"Zeit welche die rotatorische Achse benötigt: {time_rotation}s")

    # Wahl der Leitachse
    if time_rotation < time_linear: leitachse = "lineare Achse"; time_max = time_linear; velocity_linear = par_v_max; omega_rotation = ((4 * delta_phi) / (3 * time_max))
    elif time_rotation > time_linear: leitachse = "rotatorische Achse"; time_max = time_rotation; velocity_linear = ((4 * delta_s) / (3 * time_max)); omega_rotation = par_omega_max
    else: leitachse = "beide achsen sind identisch"; time_max = time_rotation
    print(f"Die Leitachse ist die {leitachse}. Die Zeit für die Bewegung ist: {time_max}s.")

    # Werte zurückgeben
    dict_return = {
        "time_max" : time_max,
        "velocity_linear" : velocity_linear,
        "omega_rotation" : omega_rotation
    }
    return dict_return

def calc_position(par_L1=0, par_L2=0, par_L3=0, par_L4=0, par_phi_rad=0):

    # Angaben
    print(f"L1 = {par_L1}m")
    print(f"L2 = {par_L2}m")
    print(f"L3 = {par_L3}m")
    print(f"L4 = {par_L4}m")
    print(f"{letter_phi_LC} = {np.degrees(par_phi_rad)}°")
    # Vektor von A bis B im Initialsystem
    print("Vektor von A bis B im Initialsystem")
    vect_A_B_I = np.array([ [0], 
                            [0], 
                            [par_L1]])
    print(vect_A_B_I)

    # Vektor von B bis TCP im 1er Koordinatensystem
    print("Vektor von B bis TCP im 1er Koordinatensystem")
    vect_B_TCP_1 = np.array([   [0], 
                                [par_L3], 
                                [par_L2 + par_L4]])
    print(vect_B_TCP_1)

    # Drehmatrix (Positive drehung um die X-Achse)
    print("Drehmatrix (Positive Drehung um die X-Achse)")
    matr_A_I1 = np.matrix([   [1,0,0],
                    [0, np.cos(par_phi_rad), -np.sin(par_phi_rad)],
                    [0, np.sin(par_phi_rad), np.cos(par_phi_rad)]])
    print(matr_A_I1)

    # Position A bis TCP im Initialsystem
    print("Vektor von A bis TCP im Initialsystem")
    vect_A_TCP_I = vect_A_B_I + (matr_A_I1 * vect_B_TCP_1)
    print(vect_A_TCP_I)

    return vect_A_TCP_I

def calc_motion(par_L1=1, par_L2=1, par_L4=1, par_time_max=1, par_time=1, par_alpha_max=1, 
                par_accel_max=1, par_omega_max=1, par_veloc_max=1, par_L3_start=1, par_phi_rad_start=1):

    # Berechnen der Beschleunigungszeit
    t_B = par_time_max / 4

    # Berechnung der Profile
    v_max = par_veloc_max; t = par_time; t_max = par_time_max; k_Tr = (par_omega_max / t_B); w_max = par_omega_max
    if (t >= 0) and (t < t_B):                                  #### Beschleunigungsbereich
        # Rotationsachse (Trapezpofil)
        alpha = k_Tr
        omega = k_Tr * t
        phi = k_Tr * ((t**2) / 2)
        # Linearachse (Sinoidenprofil)
        acceleration = ((2*v_max) / t_B) * (np.sin( (np.pi*t)/t_B )**2)
        velocity = ((v_max*t)/(t_B))    -    (    (v_max/(2*np.pi))    *    np.sin((2*np.pi*t)/t_B)   )
        distance = (v_max/(2*t_B))    *    (t**2    -   (    ((t_B**2)/(np.pi**2))    *    (np.sin( (np.pi*t)/t_B )**2)    ))
    elif (t >= t_B) and (t <= (t_max-t_B)):                     #### Konstanter Bereich
        # Rotationsachse (Trapezpofil)
        alpha = 0
        omega = w_max
        phi = ((w_max * t_B) / 2)  +  (w_max * (t-t_B))
        # Linearachse (Sinoidenprofil)
        acceleration = 0
        velocity = v_max
        distance = ((v_max*t_B)/2)    +    v_max*(t-t_B)
    elif (t > (t_max-t_B)) and (t <= t_max):                    #### Bremsbereich
        t_r = t_max-t
        # Rotationsachse (Trapezpofil)
        alpha = -k_Tr
        omega = k_Tr * t_r
        phi = ((w_max * t_B) / 2)  +  (w_max * (2 * t_B))  +  (((w_max*t_B) / 2) - ((t_r * omega)/2))
        # Linearachse (Sinoidenprofil)
        acceleration = -(((2*v_max) / t_B) * (np.sin( (np.pi*t_r)/t_B )**2))
        velocity = ((v_max*t_r)/(t_B))    -    (    (v_max/(2*np.pi))    *    np.sin((2*np.pi*t_r)/t_B)   )
        distance = ((v_max*t_B)/2)    +    v_max*(2*t_B)    +    ((v_max*t_B)/2)-(((v_max/(2*t_B))    *    (t_r**2    -   (    ((t_B**2)/(np.pi**2))    *    (np.sin( (np.pi*t_r)/t_B )**2)    ))))

    # Überprüfen auf Maximale Geschwindigkeiten
    if (velocity > par_veloc_max) or (omega > w_max): 
        return {"success" : False}

    # Berechnung von L3 und phi_rad
    L3 = par_L3_start + distance
    phi_rad = par_phi_rad_start + phi

    # Berechnung des Konfigurationsvektors q
    vect_q = np.array([     [phi],
                            [distance]        ])

    # Berechnen der ersten Ableitung des Konfigurationsvektors (q_p)
    vect_q_p = np.array([   [omega],
                            [velocity]    ])

    # Berechnen der zweiten Ableitung des Konfigurationsvektors (q_pp)
    vect_q_pp = np.array([  [alpha],
                            [acceleration]    ])

    # Berechnung der Jakobimatrix
    matrix_Jakobi = np.matrix([     [   (   -np.sin(phi_rad)*L3 - np.cos(phi_rad)*(par_L2+par_L4)    ), (   np.cos(phi_rad)   )  ], 
                                    [   (    np.cos(phi_rad)*L3 - np.sin(phi_rad)*(par_L2+par_L4)    ), (   np.sin(phi_rad)   )  ]    ])

    # Berechnung der Ableitung Jakobimatrix
    matrix_Jakobi_p = np.matrix([   [   (   -np.cos(phi_rad)*L3 + np.sin(phi_rad)*(par_L2+par_L4)    ), (   0   )  ], 
                                    [   (   -np.sin(phi_rad)*L3 - np.cos(phi_rad)*(par_L2+par_L4)    ), (   0   )  ]    ])

    # Berechnen des Umweltvektors x
    vect_x = np.array([     [   (np.cos(phi_rad) * L3)  -  (np.sin(phi_rad) * (par_L2+par_L4))  ],    # Y-TCP
                            [   (np.sin(phi_rad) * L3)  +  (np.cos(phi_rad) * (par_L2+par_L4) + par_L1)  ]   ]) # Z-TCP

    # Berechnung der ersten Ableitung des Umweltvektors (x_p)
    vect_x_p = matrix_Jakobi * vect_q_p

    # Berechnung der zweiten Ableitung des Umweltvektors (x_pp)
    vect_x_pp = matrix_Jakobi_p * vect_q_p + matrix_Jakobi * vect_q_pp

    # Zurückgeben der Werte
    dict_return = {
        "success" : True,
        "vect_q" : vect_q,
        "vect_q_p": vect_q_p,
        "vect_q_pp" : vect_q_pp,
        "vect_x" : vect_x,
        "vect_x_p" : vect_x_p,
        "vect_x_pp" : vect_x_pp
    }

    return dict_return

def plotgraphs(par_time=None, par_vect_q_rot=None, par_vect_q_p_rot=None, par_vect_q_pp_rot=None, par_vect_x_yTCP=None, par_vect_x_p_yTCP=None, par_vect_x_pp_yTCP=None, 
                par_vect_q_lin=None, par_vect_q_p_lin=None, par_vect_q_pp_lin=None, par_vect_x_zTCP=None, par_vect_x_p_zTCP=None, par_vect_x_pp_zTCP=None):

    # Globale plt.parameter setzen
    parameters = {"xtick.labelsize":8}
    plt.rcParams.update(parameters)
    
    # Figuren definieren
    fig_1 = plt.figure(figsize=(18, 9), constrained_layout=True)
    fig_2 = plt.figure(figsize=(18, 9), constrained_layout=True)
    fig_3 = plt.figure(figsize=(18, 9), constrained_layout=True)
    # fig_4 = plt.figure(figsize=(18, 9), constrained_layout=True)
    # fig_5 = plt.figure(figsize=(18, 9), constrained_layout=True)
    # fig_6 = plt.figure(figsize=(18, 9), constrained_layout=True)
    # fig_7 = plt.figure(figsize=(18, 9), constrained_layout=True)

    # Anzahl Subplots / Figure
    fig_1_GridRows = 2;     fig_1_GridCols = 2
    fig_2_GridRows = 2;     fig_2_GridCols = 2
    fig_3_GridRows = 2;     fig_3_GridCols = 2
    fig_4_GridRows = 2;     fig_4_GridCols = 2
    fig_5_GridRows = 2;     fig_5_GridCols = 2
    fig_6_GridRows = 2;     fig_6_GridCols = 2
    fig_7_GridRows = 2;     fig_7_GridCols = 2

    # Auflösung der Zeitachse
    res_x = 0.2


    ##############################################################################
    ################################ FIGURE 1 ####################################
    ##############################################################################

    ######################### plotting q Phi #########################
    ax_1 = fig_1.add_subplot(fig_1_GridRows, fig_1_GridCols, 1)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_1.set_xlabel("Zeit in [s]")
    ax_1.set_ylabel("Phi in [°]", color=color)
    ax_1.set_title("q")
    # hide spines
    ax_1.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_1.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_1.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_1.plot(par_time, np.degrees(par_vect_q_rot), color=color)
    ######################### plotting q distance #########################
    ax_2 = ax_1.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_2.set_ylabel("Weg in [m]", color=color)
    # hide spines
    ax_2.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_2.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_2.plot(par_time, par_vect_q_lin, color=color)

    ######################### plotting q_p omega #########################
    ax_3 = fig_1.add_subplot(fig_1_GridRows, fig_1_GridCols, 2)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_3.set_xlabel("Zeit in [s]")
    ax_3.set_ylabel("Omega in [°/s]", color=color)
    ax_3.set_title("q_p")
    # hide spines
    ax_3.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_3.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_3.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_3.plot(par_time, np.degrees(par_vect_q_p_rot), color=color)
    ######################### plotting q_p velocity #########################
    ax_4 = ax_3.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_4.set_ylabel("Geschwindigkeit in [m/s]", color=color)
    # hide spines
    ax_4.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_4.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_4.plot(par_time, par_vect_q_p_lin, color=color)

    ######################### plotting q_p omega #########################
    ax_5 = fig_1.add_subplot(fig_1_GridRows, fig_1_GridCols, 3)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_5.set_xlabel("Zeit in [s]")
    ax_5.set_ylabel("Alpha in [°/s²]", color=color)
    ax_5.set_title("q_pp")
    # hide spines
    ax_5.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_5.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_5.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_5.plot(par_time, np.degrees(par_vect_q_pp_rot), color=color)
    ######################### plotting q_p velocity #########################
    ax_6 = ax_5.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_6.set_ylabel("Beschleunigung in [m/s²]", color=color)
    # hide spines
    ax_6.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_6.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_6.plot(par_time, par_vect_q_pp_lin, color=color)


    ##############################################################################
    ################################ FIGURE 2 ####################################
    ##############################################################################

    ######################### plotting x y-TCP #########################
    ax_1 = fig_2.add_subplot(fig_2_GridRows, fig_2_GridCols, 1)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_1.set_xlabel("Zeit in [s]")
    ax_1.set_ylabel("TCP-Y-Weg in [m]", color=color)
    ax_1.set_title("x")
    # hide spines
    ax_1.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_1.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_1.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_1.plot(par_time, par_vect_x_yTCP, color=color)
    ######################### plotting x z-TCP #########################
    ax_2 = ax_1.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_2.set_ylabel("TCP-Z-Weg in [m]", color=color)
    # hide spines
    ax_2.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_2.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_2.plot(par_time, par_vect_x_zTCP, color=color)

    ######################### plotting x_p y-TCP #########################
    ax_3 = fig_2.add_subplot(fig_2_GridRows, fig_2_GridCols, 2)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_3.set_xlabel("Zeit in [s]")
    ax_3.set_ylabel("TCP-Y-Geschwindigkeit in [m/s]", color=color)
    ax_3.set_title("x_p")
    # hide spines
    ax_3.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_3.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_3.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_3.plot(par_time, par_vect_x_p_yTCP, color=color)
    ######################### plotting x_p z-TCP #########################
    ax_4 = ax_3.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_4.set_ylabel("TCP-Z-Geschwindigkeit in [m/s]", color=color)
    # hide spines
    ax_4.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_4.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_4.plot(par_time, par_vect_x_p_zTCP, color=color)

    ######################### plotting x_pp y-TCP #########################
    ax_5 = fig_2.add_subplot(fig_2_GridRows, fig_2_GridCols, 3)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_5.set_xlabel("Zeit in [s]")
    ax_5.set_ylabel("TCP-Y-Beschleunigung in [m/s²]", color=color)
    ax_5.set_title("x_pp")
    # hide spines
    ax_5.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_5.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_5.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_5.plot(par_time, par_vect_x_pp_yTCP, color=color)
    ######################### plotting x_pp z-TCP #########################
    ax_6 = ax_5.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_6.set_ylabel("TCP-Z-Beschleunigung in [m/s²]", color=color)
    # hide spines
    ax_6.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_6.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_6.plot(par_time, par_vect_x_pp_zTCP, color=color)


    ##############################################################################
    ################################ FIGURE 3 ####################################
    ##############################################################################

    ######################### plotting x #########################
    ax_1 = fig_3.add_subplot(fig_3_GridRows, fig_3_GridCols, 1)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_1.set_xlabel("TCP-Y-Weg in [m]")
    ax_1.set_ylabel("TCP-Z-Weg in [m]", color=color)
    ax_1.set_title("x")
    # hide spines
    ax_1.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_1.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_1.plot(par_vect_x_yTCP, par_vect_x_zTCP, color=color)

    ######################### plotting x_p #########################
    ax_2 = fig_3.add_subplot(fig_3_GridRows, fig_3_GridCols, 2)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_2.set_xlabel("TCP-Y-Geschwindigkeit in [m/s]")
    ax_2.set_ylabel("TCP-Z-Geschwindigkeit in [m/s]", color=color)
    ax_2.set_title("x_p")
    # hide spines
    ax_2.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_2.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_2.plot(par_vect_x_p_yTCP, par_vect_x_p_zTCP, color=color)
    ######################### plotting x_p z-TCP #########################

    ######################### plotting x_pp y-TCP #########################
    ax_3 = fig_3.add_subplot(fig_3_GridRows, fig_3_GridCols, 3)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_3.set_xlabel("TCP-Y-Beschleunigung in [m/s²]")
    ax_3.set_ylabel("TCP-Z-Beschleunigung in [m/s²]", color=color)
    ax_3.set_title("x_pp")
    # hide spines
    ax_3.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_3.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_3.plot(par_vect_x_pp_yTCP, par_vect_x_pp_zTCP, color=color)


    ##############################################################################
    ################################ FIGURE 4 ####################################
    ##############################################################################

    ##############################################################################
    ################################ FIGURE 5 ####################################
    ##############################################################################

    ##############################################################################
    ################################ FIGURE 6 ####################################
    ##############################################################################

    ##############################################################################
    ################################ FIGURE 7 ####################################
    ##############################################################################

    ##############################################################################
    ################################ SHOW Figures ################################
    ##############################################################################
    plt.show()

if __name__ == "__main__":
    main()
    # Leerzeilen
    print("\n\n\n\n#######################")
