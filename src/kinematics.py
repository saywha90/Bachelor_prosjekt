"""
kinematics.py

Ansvarlig for å beregne vinkler ut fra koordinater (Invers Kinematikk - IK)
og koordinater ut fra vinkler (Forover Kinematikk - FK).

Designvalg:
Denne klassen er designet for å være "hjernen" som oversetter et menneskelig ønske 
(flytt hånden til X, Y, Z) til maskininstruksjoner (sett motor 1 til 45 grader).

Modulerbarhet:
For øyeblikket implementerer vi en analytisk (geometrisk) løsning for 3 ledd.
Når vi oppgraderer til 6 akser, vil den analytiske løsningen bli ekstremt kompleks.
Derfor er koden strukturert slik at vi enkelt kan bytte ut innmaten i `solve_ik` 
med et bibliotek som 'ikpy' (Inverse Kinematics Python) uten å endre resten av koden.
"""

import math
import numpy as np
import config

class KinematicsSolver:
    def __init__(self):
        # Henter lenkelengder fra konfigurasjonsfilen
        self.l1 = config.LINK_LENGTHS['L1']
        self.l2 = config.LINK_LENGTHS['L2']
        self.l3 = config.LINK_LENGTHS['L3']
        
        print(f"KinematicsSolver initialisert for {config.NUM_JOINTS} ledd.")

    def solve_ik(self, x, y, z):
        """
        Beregner Invers Kinematikk.
        Input: Ønsket posisjon (x, y, z) i rommet.
        Output: Liste med vinkler [q1, q2, q3, ...] i grader.
        """
        
        # Sjekk hvor mange ledd vi er konfigurert for
        if config.NUM_JOINTS == 3:
            return self._solve_geometric_3dof(x, y, z)
        elif config.NUM_JOINTS == 6:
            return self._solve_6dof_placeholder(x, y, z)
        else:
            raise ValueError(f"Ingen IK-løsning implementert for {config.NUM_JOINTS} ledd.")

    def _solve_geometric_3dof(self, x, y, z):
        """
        Geometrisk løsning for en 3-ledds arm (Base, Skulder, Albue).
        
        Matematisk tilnærming:
        1. Base-vinkel (q1) beregnes ved å se på projeksjonen i XY-planet.
        2. Vi reduserer problemet til 2D ved å se på planet definert av armen og Z-aksen.
        3. Vi bruker Cosinussetningen for å finne vinklene i trekanten som dannes av L2, L3 
           og vektoren fra skulderen til sluttpunktet.
        """
        
        # --- Steg 1: Base Rotasjon (q1) ---
        # atan2(y, x) gir vinkelen i XY-planet. 
        # Dette roterer hele arm-planet mot målet.
        q1 = math.atan2(y, x)

        # --- Steg 2: Forberedelse for 2D beregning ---
        # R er radius i XY-planet (avstand fra origo til punktet projisert ned på gulvet).
        # Vi trekker ikke fra noe offset her, men hvis skulderen ikke er sentrert må det justeres.
        r = math.sqrt(x**2 + y**2)
        
        # z_effektiv er høyden relativt til skulderleddet (L1 er basehøyde).
        z_eff = z - self.l1
        
        # D er avstanden fra skulderleddet til målet (hypotenusen i 2D-planet).
        D = math.sqrt(r**2 + z_eff**2)

        # Sjekk om målet er utenfor rekkevidde
        # Hvis D er lengre enn armen (L2 + L3), kan vi ikke nå det.
        max_reach = self.l2 + self.l3
        if D > max_reach:
            raise ValueError(
                f"Målet ({x}, {y}, {z}) er utenfor rekkevidde. "
                f"Avstand: {D:.1f}mm, Maks rekkevidde: {max_reach:.1f}mm"
            )

        # --- Steg 3: Cosinussetningen for Albue (q3) ---
        # Vi har en trekant med sider L2, L3 og D.
        # Cosinussetningen: D^2 = L2^2 + L3^2 - 2*L2*L3*cos(pi - q3)
        # Vi løser for cos_angle_albue.
        
        # (L2^2 + L3^2 - D^2) / (2 * L2 * L3)
        cos_angle_albue = (self.l2**2 + self.l3**2 - D**2) / (2 * self.l2 * self.l3)
        
        # Sikring mot numeriske feil (hvis verdien er rett over 1.0 pga avrunding)
        cos_angle_albue = np.clip(cos_angle_albue, -1.0, 1.0)
        
        # Vinkel innvendig i trekanten ved albuen
        angle_albue_inner = math.acos(cos_angle_albue)
        
        # q3 er ofte definert som avviket fra rett linje, eller vinkel relativt til overarm.
        # Her antar vi at q3=0 betyr at armen er bøyd 90 grader eller rett ut, avhengig av servo-oppsett.
        # Standard geometrisk definisjon gir ofte vinkelen 'nedover'. 
        # La oss si q3 er vinkelen selve leddet beveger seg.
        q3 = math.pi - angle_albue_inner

        # --- Steg 4: Skulder (q2) ---
        # q2 består av to vinkler:
        # a) Vinkelen opp til vektoren D: atan2(z_eff, r)
        # b) Vinkelen internt i trekanten ved skulderen (vha cosinussetningen igjen eller sinussetningen)
        
        angle_to_target = math.atan2(z_eff, r)
        
        cos_angle_shoulder_inner = (self.l2**2 + D**2 - self.l3**2) / (2 * self.l2 * D)
        cos_angle_shoulder_inner = np.clip(cos_angle_shoulder_inner, -1.0, 1.0)
        angle_shoulder_inner = math.acos(cos_angle_shoulder_inner)
        
        q2 = angle_to_target + angle_shoulder_inner

        # --- Konvertering til Grader ---
        q1_deg = math.degrees(q1)
        q2_deg = math.degrees(q2)
        q3_deg = math.degrees(q3)
        
        # Merk: Her må dere kanskje mappe vinklene til servoenes koordinatsystem.
        # F.eks. hvis 90 grader er "rett opp" for servoen, må dere legge til/trekke fra offset.
        # Dette er "Fase 1"-kode, så vi returnerer de rå geometriske vinklene.
        
        return [q1_deg, q2_deg, q3_deg]

    def _solve_6dof_placeholder(self, x, y, z):
        """
        Placeholder for 6-akset løsning.
        Når dere bytter til 6 akser:
        1. Installer et bibliotek som 'ikpy' eller 'roboticstoolbox-python'.
        2. Last inn en URDF-fil (beskrivelse av roboten).
        3. Kall chain.inverse_kinematics(target).
        """
        print("IKKE IMPLEMENTERT: 6-akset kinematikk krever en numerisk løser.")
        print("Anbefaling: Bruk biblioteket 'ikpy'.")
        # Returnerer dummy-verdier så koden ikke krasjer under testing
        return [0, 0, 0, 0, 0, 0]
