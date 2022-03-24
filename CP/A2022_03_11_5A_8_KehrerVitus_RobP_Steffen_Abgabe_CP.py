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
    phi_deg = 0; phi_rad = np.radians(phi_deg);     L1 = 0.4; L2 = 0.6; L3 = 1.4; L4 = 0.4

    # maximale TCP Geschwindigkeit
    v_max_TCP = 1

    # Punkte definieren, zwischen denen Eine PTP-Bewegung erfolgt
    vect_P1_rad = np.array([[np.radians(6)],    # Winkel in [°]
                                        [0.1]]) # länge in [m]
    vect_P2_rad = np.array([[np.radians(50)],    # Winkel in [°]
                                        [0.16]]) # länge in [m]
    # Punkte definieren, zwischen denen Eine PTP-Bewegung erfolgt
    vect_P1_rad = np.array([[np.radians(-20)],    # Winkel in [°]
                                        [0.4]]) # länge in [m]
    vect_P2_rad = np.array([[np.radians(30)],    # Winkel in [°]
                                        [1.4]]) # länge in [m]

    # Punkte im als Vektoren im initialsystem
    vect_Point1_I = calc_points(par_Point=vect_P1_rad, par_L1=L1, 
                par_L2=L2, par_L4=L4)
    vect_Point2_I = calc_points(par_Point=vect_P2_rad, par_L1=L1, 
                par_L2=L2, par_L4=L4)

    # Einheitsvektor und Vektordifferenz Berechnen
    vect_deltaP_I = vect_Point2_I - vect_Point1_I
    vect_Einheit = vect_deltaP_I / np.linalg.norm(vect_deltaP_I)

    # Zeit als Array definieren
    time_max = calc_time(par_v_max=v_max_TCP, par_vect_delP=vect_deltaP_I)
    time = np.linspace(0, time_max, resolution)

    # Berechnen des Umweltvektors x und x_p
    P1_phi = vect_P1_rad[0][0]
    P1_L3 = vect_P1_rad[1][0]
    vect_x_Y = np.linspace(0,1,resolution); vect_x_Y[:]=np.nan;     ## X Arrays Initialisieren
    vect_x_Z = np.linspace(0,1,resolution); vect_x_Z[:]=np.nan;     ## X Arrays Initialisieren
    vect_x_p_Z = np.linspace(0,1,resolution); vect_x_p_Z[:]=np.nan; vect_x_p_Z[0]=0;     ## X_P Arrays Initialisieren
    vect_x_p_Y = np.linspace(0,1,resolution); vect_x_p_Y[:]=np.nan; vect_x_p_Y[0]=0;     ## X_P Arrays Initialisieren
    vect_x_Y[0] = (np.cos(P1_phi) * P1_L3)  -  (np.sin(P1_phi) * (L2+L4))       # Vector X: Y-TCP
    vect_x_Z[0] = (np.sin(P1_phi) * P1_L3)  +  (np.cos(P1_phi) * (L2+L4) + L1)  # Vector X: Z-TCP

    for i in range(0, resolution-1, 1):
        
        #### Delta S und Delta V Berechnen
        # Erster Zeitpunkt
        dict_motion_0 = calc_motion(par_time=time[i], 
                        par_time_max=time_max, par_veloc_max=v_max_TCP)
        if dict_motion_0["success"] == True: 
            s_0 = dict_motion_0["distance"]
            v_0 = dict_motion_0["velocity"]
        # Zweiter Zeitpunkt
        dict_motion_1 = calc_motion(par_time=time[i+1], 
                        par_time_max=time_max, par_veloc_max=v_max_TCP)
        if dict_motion_1["success"] == True: 
            s_1 = dict_motion_1["distance"]
            v_1 = dict_motion_1["velocity"]
        delta_s = s_1 - s_0                                             # Delta S Berechnen
        delta_v = v_1 - v_0                                             # Delta V Berechnen
        vect_x_Y[i+1] = vect_x_Y[i] + (vect_Einheit[1][0] * delta_s)
        vect_x_Z[i+1] = vect_x_Z[i] + (vect_Einheit[2][0] * delta_s)
        vect_x_p_Y[i+1] = vect_x_p_Y[i] + (vect_Einheit[1][0] * delta_v)
        vect_x_p_Z[i+1] = vect_x_p_Z[i] + (vect_Einheit[2][0] * delta_v)

    # Berechnen der Konfigurationsvektoren
    dict_configvekt = calc_configvect(vect_P1=vect_P1_rad, par_L2=L2, par_L4=L4, resolution=resolution, 
                                    vect_x_Y=vect_x_Y, vect_x_Z=vect_x_Z,
                                    vect_x_p_Y=vect_x_p_Y, vect_x_p_Z=vect_x_p_Z)
    vect_q_phi = dict_configvekt["vect_q_phi"] 
    vect_q_L3 = dict_configvekt["vect_q_L3"] 
    vect_q_p_phi = dict_configvekt["vect_q_p_phi"] 
    vect_q_p_L3 = dict_configvekt["vect_q_p_L3"] 

    # Berechnen der Fehler
    dict_error = calc_error(vect_q_phi=vect_q_phi, vect_q_L3=vect_q_L3, 
                            vect_x_Y_motion=vect_x_Y, vect_x_Z_motion=vect_x_Z, 
                            L1=L1, L2=L2,L4=L4,resolution=resolution)
    Y_Error = dict_error["Y_Error"]
    Z_Error = dict_error["Z_Error"]


    # Plotten der Graphen
    plotgraphs(time=time, vect_x_Y=vect_x_Y, vect_x_Z=vect_x_Z, vect_x_p_Y= vect_x_p_Y, 
                vect_x_p_Z=vect_x_p_Z, vect_q_phi=vect_q_phi, vect_q_L3=vect_q_L3,
                vect_q_p_phi=vect_q_p_phi, vect_q_p_L3=vect_q_p_L3, Y_Error=Y_Error,
                Z_Error=Z_Error
                )



def calc_points(par_Point=None, par_L1=None, 
                par_L2=None, par_L4=None):

    # Werte Auslesen
    phi = par_Point[0][0]
    L3 = par_Point[1][0]

    # Vektor von A bis B im Initialsystem
    vect_A_B_I = np.array([ [0], 
                            [0], 
                            [par_L1]])

    # Vektor von B bis TCP im 1er Koordinatensystem
    vect_B_TCP_1 = np.array([   [0], 
                                [L3], 
                                [par_L2 + par_L4]])

    # Drehmatrix (Positive drehung um die X-Achse)
    matr_A_I1 = np.matrix([   [1,0,0],
                    [0, np.cos(phi), -np.sin(phi)],
                    [0, np.sin(phi), np.cos(phi)]])

    # Position A bis TCP im Initialsystem
    vect_A_TCP_I = vect_A_B_I + (matr_A_I1 * vect_B_TCP_1)

    return vect_A_TCP_I

def calc_time(par_v_max=None, par_vect_delP=None):

    # Wegdifferenz berechnen
    delta_s = np.linalg.norm(par_vect_delP)

    # Gesamtzeit berechnen
    time_max = (4 * delta_s) / (3 * par_v_max)

    return time_max

def calc_motion(par_time=None, par_time_max=None, par_veloc_max=None):

    # Berechnen der Beschleunigungszeit
    t_B = par_time_max / 4

    v_max = par_veloc_max; t = par_time; t_max = par_time_max

    if (t >= 0) and (t < t_B):                                  #### Beschleunigungsbereich
        # Linearachse (Sinoidenprofil)
        #acceleration = ((2*v_max) / t_B) * (np.sin( (np.pi*t)/t_B )**2)
        velocity = ((v_max*t)/(t_B))    -    (    (v_max/(2*np.pi))    *    np.sin((2*np.pi*t)/t_B)   )
        distance = (v_max/(2*t_B))    *    (t**2    -   (    ((t_B**2)/(np.pi**2))    *    (np.sin( (np.pi*t)/t_B )**2)    ))
    elif (t >= t_B) and (t <= (t_max-t_B)):                     #### Konstanter Bereich
        # Linearachse (Sinoidenprofil)
        #acceleration = 0
        velocity = v_max
        distance = ((v_max*t_B)/2)    +    v_max*(t-t_B)
    elif (t > (t_max-t_B)) and (t <= t_max):                    #### Bremsbereich
        t_r = t_max-t
        # Linearachse (Sinoidenprofil)
        #acceleration = -(((2*v_max) / t_B) * (np.sin( (np.pi*t_r)/t_B )**2))
        velocity = ((v_max*t_r)/(t_B))    -    (    (v_max/(2*np.pi))    *    np.sin((2*np.pi*t_r)/t_B)   )
        distance = ((v_max*t_B)/2)    +    v_max*(2*t_B)    +    ((v_max*t_B)/2)-(((v_max/(2*t_B))    *    (t_r**2    -   (    ((t_B**2)/(np.pi**2))    *    (np.sin( (np.pi*t_r)/t_B )**2)    ))))
        
    # Überprüfen auf Maximale Geschwindigkeiten
    if (velocity > par_veloc_max): 
        return {"success" : False}
    
    # Werte zurückgeben
    dict_return = { "success": True,
                    "distance": distance,
                    "velocity": velocity}
    return dict_return

def calc_configvect(vect_P1=None, par_L2=None, par_L4=None, resolution=None, 
            vect_x_Y=None, vect_x_Z=None, vect_x_p_Y=None, vect_x_p_Z=None):
    # Winkel und L3 auslesen
    P1_phi = vect_P1[0][0]
    P1_L3 = vect_P1[1][0]
    
    vect_q_phi = np.linspace(0,1,resolution);   vect_q_phi[:]=np.nan;   vect_q_phi[0]=P1_phi;     ## Q Arrays Initialisieren
    vect_q_L3 = np.linspace(0,1,resolution);    vect_q_L3[:]=np.nan;    vect_q_L3[0]=P1_L3;     ## Q Arrays Initialisieren
    vect_q_p_phi = np.linspace(0,1,resolution); vect_q_p_phi[:]=np.nan; vect_q_p_phi[0]=0;     ## Q_P Arrays Initialisieren
    vect_q_p_L3 = np.linspace(0,1,resolution);  vect_q_p_L3[:]=np.nan;  vect_q_p_L3[0]=0;     ## Q_P Arrays Initialisieren

    for i in range(0, resolution-1, 1):
        # Berechnen der Jakobimatrix
        matrix_Jakobi = np.matrix(
            [ [ ( -np.sin(vect_q_phi[i])*vect_q_L3[i] - np.cos(vect_q_phi[i])*(par_L2+par_L4)  ), ( np.cos(vect_q_phi[i]) )], 
            [   (  np.cos(vect_q_phi[i])*vect_q_L3[i] - np.sin(vect_q_phi[i])*(par_L2+par_L4)  ), ( np.sin(vect_q_phi[i]) )] ])

        # Berechnen der Inversen Jakobimatrix
        matrix_Jakobi_Inv = np.linalg.inv(matrix_Jakobi)
        # detj = matrix_Jakobi[0,0]*matrix_Jakobi[1,1]-matrix_Jakobi[1,0]*matrix_Jakobi[0,1]
        # matrix_Jakobi_Inv[0,0] = matrix_Jakobi[1,1]
        # matrix_Jakobi_Inv[0,1] = -matrix_Jakobi[0,1]
        # matrix_Jakobi_Inv[1,0] = -matrix_Jakobi[1,0]
        # matrix_Jakobi_Inv[1,1] = matrix_Jakobi[0,0]
        # matrix_Jakobi_Inv = (1/detj) * matrix_Jakobi_Inv

        #### Q Vektor
        # Delta X Berechnen
        delta_x_Y = vect_x_Y[i+1] - vect_x_Y[i]
        delta_x_Z = vect_x_Z[i+1] - vect_x_Z[i]
        vect_delt_x = np.array([    [delta_x_Y],
                                    [delta_x_Z]  ])
        # Konfigurationsvektor Q Berechnen
        vect_q_i = np.array([   [vect_q_phi[i]],
                                [vect_q_L3[i]]  ])
        vect_q_ip1 = (matrix_Jakobi_Inv * vect_delt_x)  +  vect_q_i # Konfigurationsvektor Q Berechnen (i + 1)
        vect_q_phi[i+1] = vect_q_ip1[0][0]
        vect_q_L3[i+1] = vect_q_ip1[1][0]
        #### Q_P Vektor
        vect_x_p = np.array([   [vect_x_p_Y[i+1]],
                                [vect_x_p_Z[i+1]]  ])
        vect_q_p = matrix_Jakobi_Inv * vect_x_p
        vect_q_p_phi[i+1] = vect_q_p[0][0]
        vect_q_p_L3[i+1] = vect_q_p[1][0]



    dict_return = { "vect_q_phi": vect_q_phi,
                    "vect_q_L3": vect_q_L3,
                    "vect_q_p_phi": vect_q_p_phi,
                    "vect_q_p_L3": vect_q_p_L3
    }
    return dict_return

def calc_error(vect_q_phi=None, vect_q_L3=None, vect_x_Y_motion=None, vect_x_Z_motion=None, 
                L1=None, L2=None,L4=None,resolution=None):

    vect_x_Y = np.linspace(0,1,resolution); vect_x_Y[:]=np.nan;     ## X Arrays Initialisieren
    vect_x_Z = np.linspace(0,1,resolution); vect_x_Z[:]=np.nan;     ## X Arrays Initialisieren
    for i in range(0, resolution, 1):
        Phi = vect_q_phi[i]
        L3 = vect_q_L3[i]
        # Berechnen der TCP Position aus den Konfigurationsvektoren
        vect_x_Y[i] = (np.cos(Phi) * L3)  -  (np.sin(Phi) * (L2+L4))           # Y-TCP
        vect_x_Z[i] = (np.sin(Phi) * L3)  +  (np.cos(Phi) * (L2+L4) + L1)      # Z-TCP
    Y_Error = vect_x_Y_motion - vect_x_Y
    Z_Error = vect_x_Z_motion - vect_x_Z

    dict_return = {
        "Y_Error" : Y_Error,
        "Z_Error" : Z_Error
    }
    return dict_return

def plotgraphs(time=None, vect_x_Y=None, vect_x_Z=None, vect_x_p_Y= None, 
                vect_x_p_Z=None, vect_q_phi=None, vect_q_L3=None
                , vect_q_p_phi=None, vect_q_p_L3=None, Y_Error=None,
                Z_Error=None):
    # Globale plt.parameter setzen
    parameters = {"xtick.labelsize":8}
    plt.rcParams.update(parameters)
    
    # Figuren definieren
    fig_1 = plt.figure(figsize=(18, 9), constrained_layout=True)
    fig_2 = plt.figure(figsize=(18, 9), constrained_layout=True)
    # fig_3 = plt.figure(figsize=(18, 9), constrained_layout=True)
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

    ######################### plotting X_VECT: Y #########################
    ax_1 = fig_1.add_subplot(fig_1_GridRows, fig_1_GridCols, 1)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_1.set_xlabel("Zeit in [s]")
    ax_1.set_ylabel("TCP-Y-Weg in [m]", color=color)
    ax_1.set_title("TCP Weg (X-Vektor)")
    # hide spines
    ax_1.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_1.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_1.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_1.plot(time, vect_x_Y, color=color)
    ######################### plotting X_VECT: Z #########################
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
    ax_2.plot(time, vect_x_Z, color=color)
    ######################### plotting X_VECT #########################
    ax_3 = fig_1.add_subplot(fig_1_GridRows, fig_1_GridCols, 2)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_3.set_xlabel("TCP-Y-Weg in [m]")
    ax_3.set_ylabel("TCP-Z-Weg in [m]", color=color)
    ax_3.set_title("TCP Weg (X-Vektor)")
    # hide spines
    ax_3.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_3.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_3.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_3.plot(vect_x_Y, vect_x_Z, color=color)
    ######################### plotting X_P VECT: Y #########################
    ax_4 = fig_1.add_subplot(fig_1_GridRows, fig_1_GridCols, 3)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_4.set_xlabel("Zeit in [s]")
    ax_4.set_ylabel("TCP-Y-Geschwindigkeit in [m/s]", color=color)
    ax_4.set_title("TCP Geschwindigkeit (X_p-Vektor)")
    # hide spines
    ax_4.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_4.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_4.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_4.plot(time, vect_x_p_Y, color=color)
    ######################### plotting X_P VECT: Z #########################
    ax_5 = ax_4.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_5.set_ylabel("TCP-Z-Geschwindigkeit in [m/s]", color=color)
    # hide spines
    ax_5.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_5.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_5.plot(time, vect_x_p_Z, color=color)
    ######################### plotting X_P VECT #########################
    ax_6 = fig_1.add_subplot(fig_1_GridRows, fig_1_GridCols, 4)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_6.set_xlabel("Zeit in [s]")
    ax_6.set_ylabel("TCP-Geschwindigkeit in [m/s]", color=color)
    ax_6.set_title("TCP Geschwindigkeit")
    # hide spines
    ax_6.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_6.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_6.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_6.plot(time, np.sqrt(vect_x_p_Y**2+vect_x_p_Z**2), color=color)


    ##############################################################################
    ################################ FIGURE 2 ####################################
    ##############################################################################

    ######################### plotting Q_VECT: Phi #########################
    ax_1 = fig_2.add_subplot(fig_2_GridRows, fig_2_GridCols, 1)
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
    ax_1.plot(time, np.degrees(vect_q_phi), color=color)
    ######################### plotting Q_VECT: L3 #########################
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
    ax_2.plot(time, vect_q_L3, color=color)
    ######################### plotting Q_p_VECT: Phi #########################
    ax_3 = fig_2.add_subplot(fig_2_GridRows, fig_2_GridCols, 2)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_3.set_xlabel("Zeit in [s]")
    ax_3.set_ylabel("Winkelgeschwindigkeit in [°/s]", color=color)
    ax_3.set_title("q_p")
    # hide spines
    ax_3.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_3.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_3.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_3.plot(time, np.degrees(vect_q_p_phi), color=color)
    ######################### plotting Q_p_VECT: L3 #########################
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
    ax_4.plot(time, vect_q_p_L3, color=color)
######################### Y-Fehler #########################
    ax_5 = fig_2.add_subplot(fig_2_GridRows, fig_2_GridCols, 3)
    # def color
    color = 'tab:red'
    # axes labeling and title
    ax_5.set_xlabel("Zeit in [s]")
    ax_5.set_ylabel("Y-Fehler in [mm]", color=color)
    ax_5.set_title("Fehler")
    # hide spines
    ax_5.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_5.xaxis.set_major_locator(mlt.ticker.MultipleLocator(base=res_x))
    ax_5.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_5.plot(time, Y_Error*1000, color=color)    
    ######################### Z-Fehler #########################
    ax_6 = ax_5.twinx()
    # def color
    color = 'tab:green'
    # axes labeling and title
    ax_6.set_ylabel("Z-Fehler in [mm]", color=color)
    # hide spines
    ax_6.tick_params(axis="y", labelcolor=color)
    # setting grid options
    ax_6.grid(visible = True, color =color, linestyle ='-.', linewidth = 0.5, alpha = 0.6)
    # plot
    ax_6.plot(time, Z_Error*1000, color=color)

    ##############################################################################
    ################################ SHOW Figures ################################
    ##############################################################################
    plt.show()


if __name__ == "__main__":
    main()
    # Leerzeilen
    print("\n\n\n\n#######################")