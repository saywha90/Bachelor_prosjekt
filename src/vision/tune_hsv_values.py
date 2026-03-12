"""
Interaktivt HSV-tuning script for å finne optimale fargeverdier.

Dette lar deg justere HSV-verdier i sanntid for perfekt fargedeteksjon.
"""

import cv2
import numpy as np
import argparse

def nothing(x):
    """Callback for trackbar (må eksistere selv om vi ikke bruker den)"""
    pass

def tune_hsv(camera_index=0, color='red'):
    """
    Interaktiv HSV-tuning med live preview.
    
    Args:
        camera_index: Kamera ID
        color: 'red' eller 'blue'
    """
    # Åpne kamera
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    
    if not cap.isOpened():
        print("❌ Kunne ikke åpne kamera!")
        return
    
    # Start-verdier basert på farge
    if color == 'red':
        # Rød - to områder
        h_low1, s_low1, v_low1 = 0, 100, 100
        h_high1, s_high1, v_high1 = 10, 255, 255
        h_low2, s_low2, v_low2 = 170, 100, 100
        h_high2, s_high2, v_high2 = 179, 255, 255
    else:  # blue
        h_low1, s_low1, v_low1 = 100, 100, 100
        h_high1, s_high1, v_high1 = 130, 255, 255
        h_low2, s_low2, v_low2 = 0, 0, 0  # Ikke brukt for blå
        h_high2, s_high2, v_high2 = 0, 0, 0
    
    # Lag vinduer
    cv2.namedWindow('Original')
    cv2.namedWindow('Mask')
    cv2.namedWindow('Result')
    cv2.namedWindow('HSV Trackbars')
    
    # Lag trackbars - Range 1
    cv2.createTrackbar('H Low 1', 'HSV Trackbars', h_low1, 179, nothing)
    cv2.createTrackbar('S Low 1', 'HSV Trackbars', s_low1, 255, nothing)
    cv2.createTrackbar('V Low 1', 'HSV Trackbars', v_low1, 255, nothing)
    cv2.createTrackbar('H High 1', 'HSV Trackbars', h_high1, 179, nothing)
    cv2.createTrackbar('S High 1', 'HSV Trackbars', s_high1, 255, nothing)
    cv2.createTrackbar('V High 1', 'HSV Trackbars', v_high1, 255, nothing)
    
    if color == 'red':
        # Range 2 kun for rød
        cv2.createTrackbar('H Low 2', 'HSV Trackbars', h_low2, 179, nothing)
        cv2.createTrackbar('S Low 2', 'HSV Trackbars', s_low2, 255, nothing)
        cv2.createTrackbar('V Low 2', 'HSV Trackbars', v_low2, 255, nothing)
        cv2.createTrackbar('H High 2', 'HSV Trackbars', h_high2, 179, nothing)
        cv2.createTrackbar('S High 2', 'HSV Trackbars', s_high2, 255, nothing)
        cv2.createTrackbar('V High 2', 'HSV Trackbars', v_high2, 255, nothing)
    
    # Morfologi-parametere
    cv2.createTrackbar('Morph Kernel', 'HSV Trackbars', 5, 21, nothing)
    cv2.createTrackbar('Min Area', 'HSV Trackbars', 100, 5000, nothing)
    
    print(f"\n{'='*60}")
    print(f"🎨 HSV TUNING FOR {color.upper()}")
    print(f"{'='*60}")
    print("Kontroller:")
    print("  q - Avslutt og lagre verdier")
    print("  r - Reset til standardverdier")
    print("  s - Skriv ut nåværende verdier")
    print(f"{'='*60}\n")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Les trackbar-verdier
        h_low1 = cv2.getTrackbarPos('H Low 1', 'HSV Trackbars')
        s_low1 = cv2.getTrackbarPos('S Low 1', 'HSV Trackbars')
        v_low1 = cv2.getTrackbarPos('V Low 1', 'HSV Trackbars')
        h_high1 = cv2.getTrackbarPos('H High 1', 'HSV Trackbars')
        s_high1 = cv2.getTrackbarPos('S High 1', 'HSV Trackbars')
        v_high1 = cv2.getTrackbarPos('V High 1', 'HSV Trackbars')
        
        kernel_size = cv2.getTrackbarPos('Morph Kernel', 'HSV Trackbars')
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel_size = max(3, kernel_size)
        min_area = cv2.getTrackbarPos('Min Area', 'HSV Trackbars')
        
        # Konverter til HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Lag maske
        lower1 = np.array([h_low1, s_low1, v_low1])
        upper1 = np.array([h_high1, s_high1, v_high1])
        mask = cv2.inRange(hsv, lower1, upper1)
        
        if color == 'red':
            h_low2 = cv2.getTrackbarPos('H Low 2', 'HSV Trackbars')
            s_low2 = cv2.getTrackbarPos('S Low 2', 'HSV Trackbars')
            v_low2 = cv2.getTrackbarPos('V Low 2', 'HSV Trackbars')
            h_high2 = cv2.getTrackbarPos('H High 2', 'HSV Trackbars')
            s_high2 = cv2.getTrackbarPos('S High 2', 'HSV Trackbars')
            v_high2 = cv2.getTrackbarPos('V High 2', 'HSV Trackbars')
            
            lower2 = np.array([h_low2, s_low2, v_low2])
            upper2 = np.array([h_high2, s_high2, v_high2])
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask, mask2)
        
        # Morfologi
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Finn konturer
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Tegn konturer og tell baller
        result = frame.copy()
        ball_count = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
                # Få bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Tegn
                cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)
                cv2.rectangle(result, (x, y), (x+w, y+h), (255, 0, 0), 2)
                
                # Tell
                ball_count += 1
                cv2.putText(result, str(ball_count), (x, y-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        # Info-tekst
        cv2.putText(result, f'Baller: {ball_count}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(result, f'Min Area: {min_area}', (10, 70), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Vis
        cv2.imshow('Original', frame)
        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        
        # Tastatur-input
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Skriv ut verdier
            print(f"\n{'='*60}")
            print(f"NÅVÆRENDE {color.upper()} HSV-VERDIER:")
            print(f"{'='*60}")
            print(f"Range 1:")
            print(f"  lower_1 = np.array([{h_low1}, {s_low1}, {v_low1}])")
            print(f"  upper_1 = np.array([{h_high1}, {s_high1}, {v_high1}])")
            if color == 'red':
                print(f"Range 2:")
                print(f"  lower_2 = np.array([{h_low2}, {s_low2}, {v_low2}])")
                print(f"  upper_2 = np.array([{h_high2}, {s_high2}, {v_high2}])")
            print(f"Morph kernel: {kernel_size}")
            print(f"Min area: {min_area}")
            print(f"{'='*60}\n")
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    # Print final verdier
    print(f"\n{'='*60}")
    print(f"✅ OPTIMALISERTE {color.upper()} HSV-VERDIER:")
    print(f"{'='*60}")
    print(f"\nKopier disse verdiene til ball_detection.py:\n")
    print(f"# {color.upper()} FARGE - Optimalisert")
    if color == 'red':
        print(f"self.red_lower_1 = np.array([{h_low1}, {s_low1}, {v_low1}])")
        print(f"self.red_upper_1 = np.array([{h_high1}, {s_high1}, {v_high1}])")
        print(f"self.red_lower_2 = np.array([{h_low2}, {s_low2}, {v_low2}])")
        print(f"self.red_upper_2 = np.array([{h_high2}, {s_high2}, {v_high2}])")
    else:
        print(f"self.blue_lower = np.array([{h_low1}, {s_low1}, {v_low1}])")
        print(f"self.blue_upper = np.array([{h_high1}, {s_high1}, {v_high1}])")
    print(f"\nMorfologi:")
    print(f"self.morph_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, ({kernel_size}, {kernel_size}))")
    print(f"min_area_threshold = {min_area}  # Legg til i detect_balls()")
    print(f"{'='*60}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tune HSV values for ball detection")
    parser.add_argument('--camera', type=int, default=0, help='Camera index')
    parser.add_argument('--color', type=str, default='red', choices=['red', 'blue'],
                       help='Color to tune')
    
    args = parser.parse_args()
    
    tune_hsv(args.camera, args.color)
